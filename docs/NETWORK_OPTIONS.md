# Network options and diagnostics

Status: `USER_DECISION_REQUIRED`; no option is selected by this plan.

## Security invariant

The raw OpenPI WebSocket must not be exposed to the public Internet. An API
key is secondary protection only and never replaces an encrypted/private
transport. The server must not be reachable as a direct robot-control path.

## NET-A — SSH tunnel

Laptop-side template:

```bash
ssh -N -L <local_port>:127.0.0.1:<server_port> <user>@<server>
```

The client then connects to `127.0.0.1:<local_port>`. The server policy port
is kept loopback/firewall restricted. This is the recommended P0/P1 bring-up
option because it minimizes infrastructure and provides encryption.

Risks: tunnel lifecycle, reconnect policy, SSH session loss. P3 must measure
reconnect time and ensure a lost tunnel rejects actions.

## NET-B — private overlay VPN

Laptop and server use a private overlay address. This is the recommended later
option for repeated experiments if the user accepts installing and managing a
VPN client. Firewall rules still restrict the policy service to the private
interface, and the laptop validates server identity/metadata.

## NET-C — self-managed WireGuard

This offers an independently managed private network but adds key rotation,
routing, firewall and recovery work. It is not selected automatically.

## Required P3 test matrix

Run 1000 requests per payload after the user selects a mode:

| Payload | Content |
|---|---|
| small dummy | metadata only |
| one RGB | one model-sized uint8 RGB |
| dual RGB | base + wrist model-sized uint8 RGB |
| dual RGB + state | base + wrist + STATE_7 |

Record median, p90, p95, p99 RTT; request/packet/connection failures; timeout
counts; and reconnect time. Store the result in
`reports/network_latency_report.json`. Test results are not permission to
execute the robot.

## Action freshness

Every response has a `request_id`. If a newer observation exists when a
response arrives, the response is discarded. The laptop never drains a stale
action chunk after timeout or reconnect. Initial execution uses
`EXECUTION_HORIZON=1` so that no open-loop horizon is silently inherited.
