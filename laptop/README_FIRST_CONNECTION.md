# First laptop connection

This is a P2 handoff document. It is not executed by the server agent in P0/P1.

1. On the laptop, install only `laptop/client/requirements.txt` in a dedicated
   virtual environment.
2. Establish the frozen SSH tunnel:

   ```bash
   ssh -N -L 8000:127.0.0.1:8000 <USER>@<SERVER>
   ```

3. In a second laptop terminal, use `127.0.0.1:8000`, never the server public
   IP and port 8000.
4. Run `python test_policy_tunnel.py` and then the client smoke script.
5. Run `diagnostics/probe_piper_readonly.py` only as a read-only hardware audit
   after explicit P2 approval. It must not be combined with enable, reset,
   emergency-stop, joint, gripper, or motion operations.

Expected P2 handoff status from this package: `REMOTE_TUNNEL =
READY_FOR_LAPTOP_TEST`. P0/P1 does not claim that the laptop tunnel or any
camera/robot hardware was accessed.
