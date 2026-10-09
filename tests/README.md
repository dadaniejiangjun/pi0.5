# Current-chain regression tests

此目录仅保留当前通信、相机、采集、数据处理、训练契约、服务身份、推理与现有控制 authority 的回归，以及它们所需的 test support。33 个只验证旧 proxy、ALOHA/public-data、raw HIL 或旧 formal-policy 路线的测试已随旧实现归档。

默认 pytest 仅发现 tests/；archive 不参与自动发现。离线回归使用已有 Conda 环境与 [guarded runner](../reports/active_chain_pure_test_runner.py)，它拒绝真实相机、USB、网络 policy 与训练执行，允许本地录制视频和测试注入的 fake adapter。

```powershell
& 'E:\anaconda3\envs\pi05_laptop\python.exe' -B reports/active_chain_pure_test_runner.py tests
```

保留测试不是实机授权，也不证明机器人任务成功。历史测试复现限制见 [snapshot](../archive/legacy_snapshot_20261009/README.md)。

公开仓库使用人工构造的 `phase_a_synthetic_feedback.json`。本地真实反馈 fixture 不上传；若本地存在，golden 测试仍可使用它。设置 `PIPER_TEST_FEEDBACK_FIXTURE` 可以显式选择测试样本。原数据/视频/报告不包含在源码快照中。
