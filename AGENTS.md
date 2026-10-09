# 07_pi0.5 project rules

## GitHub push policy

The canonical project remote is:

```text
https://github.com/dadaniejiangjun/pi0.5.git
```

Push only the project root repository. `third_party/openpi` must remain the
official `Physical-Intelligence/openpi` submodule at the pinned commit recorded
in `provenance/openpi_git.json`; never replace it with a fork, mirror, copied
snapshot, or a modified upstream checkout.

Before every push:

1. Confirm `git remote -v` and the current branch.
2. Review `git status`, `git diff --cached`, and `git diff --check`.
3. Check the staged file list for credentials, CAN/hardware data, private keys,
   `.venv`, caches, checkpoints, raw datasets, and files larger than 100 MiB.
4. Preserve provenance files and project documentation; do not rewrite history
   or force-push unless the user explicitly authorizes it.
5. Push the reviewed commit to the named remote and verify the remote branch
   and commit ID afterward.

The following remain local-only unless the user explicitly creates a separate
release procedure: `.venv/`, `cache/`, `checkpoints/`, runtime logs, raw or
incoming datasets, machine secrets, and real-robot captures. No Git operation
may connect to CAN, enable/reset a PiPER, or start robot motion.

Credentials must be supplied by the user's configured Git credential helper,
SSH agent, or an interactive HTTPS prompt. Never place tokens or private keys
in this repository, commit message, remote URL, or project reports.
## Phase status authority

Current human-readable deployment status is `docs/CURRENT_STATUS.md`.
Formal acceptance criteria are `docs/PHASE_GATES.md`; operational commands are
`docs/RUNBOOK.md`. Update their affected summaries when new verified evidence
changes the project status. README links to these authorities.

`docs/history/PI05_PIPER_PHASE_GATES*.json` and the initial
`provenance/deployment_manifest.json` retain dated P0/P1 provenance. Do not use
these historical snapshots to override current deployment evidence or infer
new phase PASS statuses. Source organization and a GitHub push are not new
training, inference, hardware, data-collection, or autonomous-task evidence.
