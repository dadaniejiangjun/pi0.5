# Historical implementation and test snapshot

2026-10-09，按当前实际采集 → 转换 → 训练 → 服务 → 推理 → Bridge/CAN 链路收口。此快照保留原相对路径，包含旧实现和仅验证旧实现的测试，避免旧测试成为 active code 的保留理由。没有永久删除源码，也没有修改归档 Python 的内容。

当前入口见 [ACTIVE_CODE_INDEX](../../docs/ACTIVE_CODE_INDEX.md)，逐文件来源、SHA256 和替代关系见 [manifest](../ARCHIVE_MANIFEST.md)。默认 pytest 不发现此目录；active code 不依赖本快照。

## Reproduction

这不是可直接运行的独立工作树。归档源码保留原 import 与 ROOT/cwd 语义；不要从新路径直接运行旧硬件脚本，也不要让当前代码 import archive。需要历史复现时，在独立工作树中按 manifest 恢复原路径，补齐对应历史环境、配置、报告、数据和 checkpoint；先复核旧硬件入口的 guard 与授权条件。旧测试也需在该历史工作树恢复后运行。本轮未执行任何归档入口。

历史 runner 的原 JSON、视频、CAN 日志和训练/数据证据继续留在原位置。目录 README 的旧版本保存在本快照相同路径，只用于阅读历史命令。
