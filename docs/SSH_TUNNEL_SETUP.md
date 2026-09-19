# SSH tunnel for the localhost-only policy server

The server wrapper binds to `127.0.0.1:8000` only. The intended bring-up
transport is an SSH local forward:

```bash
ssh -N \
  -L 8000:127.0.0.1:8000 \
  <USER>@<SERVER>
```

The laptop client must connect to `host=127.0.0.1`, `port=8000`. Do not put the
server public IP and port 8000 into a client default, and do not expose the
WebSocket server on `0.0.0.0`.

P0/P1 status: `REMOTE_TUNNEL = READY_FOR_LAPTOP_TEST`. The server-side agent
cannot operate the user's laptop, so no remote tunnel pass is claimed here.
