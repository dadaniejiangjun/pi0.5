# Third-party source policy

The only OpenPI source allowed for this project is:

`https://github.com/Physical-Intelligence/openpi`

The planned checkout is `third_party/openpi/`. It is intentionally absent in
the plan-only turn. When P0 begins, clone with submodules, record the exact
commit/submodule SHAs, and write them to `provenance/openpi_contract.json`.

Do not copy OpenPI code from `06_maniskill_tactile`, `03_gripper2`, an old
virtual environment or an unpinned local cache.
