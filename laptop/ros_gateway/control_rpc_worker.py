"""Bounded serial RPC work, separate from the TCP heartbeat receive loop."""
import queue
import threading


class SessionControlRpcWorker:
    def __init__(self, *, handle_rpc, send, session_id, used_request_ids, stopped):
        self.handle_rpc = handle_rpc
        self.send = send
        self.session_id = session_id
        self.used_request_ids = used_request_ids
        self.stopped = stopped
        self.pending = queue.Queue(maxsize=1)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, payload):
        if self.stopped.is_set():
            return
        try:
            self.pending.put_nowait(payload)
        except queue.Full:
            self.send("control_result", {
                "session_id": self.session_id,
                "request_id": str(payload.get("request_id", ""))[:128],
                "accepted": False,
                "decision": "REJECTED_CONTROL_QUEUE_FULL",
                "reason": "session RPC worker has one pending request already",
                "can_tx_calls": 0,
            })

    def _run(self):
        while not self.stopped.is_set():
            try:
                payload = self.pending.get(timeout=0.02)
            except queue.Empty:
                continue
            if self.stopped.is_set():
                return
            try:
                result = self.handle_rpc(payload, session_id=self.session_id,
                                         used_request_ids=self.used_request_ids)
                if not self.stopped.is_set():
                    self.send("control_result", result)
            except Exception:
                # A handler exception cannot be represented as zero hardware writes.
                self.stopped.set()
                return

    def close(self):
        self.stopped.set()
        while True:
            try:
                self.pending.get_nowait()
            except queue.Empty:
                break
        self.thread.join(timeout=0.5)
