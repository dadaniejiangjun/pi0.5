# Archive

历史源码与复现入口；不属于当前运行入口。最新整组归档见 [legacy snapshot](legacy_snapshot_20261009/README.md)。当前部署见 [RUNBOOK](../docs/RUNBOOK.md)，每次归档的来源、替代和限制见 [ARCHIVE_MANIFEST](ARCHIVE_MANIFEST.md)。

ARCHIVE != DELETE。归档文件保持原始字节，active code 不得 import archive。不要为了复现历史补丁覆盖当前 gate、manifest 或运行证据；设备探针也不能被当成离线脚本直接运行。

| 类别 | 内容 |
|---|---|
| legacy_hil | 满足依赖准入条件后的历史发送实现；首批为空；后续 7 个旧 sender 已随历史测试进入 legacy_snapshot_20261009 |
| deprecated_scripts | 不再是当前入口的一次性历史补丁 |
| old_diagnostics | 已由可复用工具覆盖日常用途的 dated runner |
| old_policy_experiments | 审核后归档的旧策略实验及其复现入口；本轮为空 |
| old_dataset_experiments | 审核后归档的旧数据实验；本轮为空 |
| historical_configs | 不再参与当前路径且有明确替代的配置；本轮为空 |
| one_off_tools | 审核后归档的一次性工具；本轮为空 |

历史文档继续放在 [docs/history](../docs/history/README.md)。训练报告、数据集、视频、CAN 日志、checkpoint 和 provenance 留在原证据位置。空类别预留目录，不表示已迁移代码。
