---
Document status: REFERENCE
Current authority: docs/ARCHITECTURE.md
Last reviewed: 2026-10-08
---

# OpenPI provenance

本文件解释来源事实；当前运行架构见 [ARCHITECTURE](../ARCHITECTURE.md)。
依据是 [openpi_git.json](../../provenance/openpi_git.json)、root gitlink 与服务器 HEAD。

## Verified pin

本项目保留官方 submodule，不使用 fork 或复制快照替换上游。

| 项目 | 已记录值 |
|---|---|
| Repository | `https://github.com/Physical-Intelligence/openpi` |
| OpenPI commit | `215abfb217dbac7d5f1273282331b9b1866c0479` |
| ALOHA submodule | `d1dc83afd89ded4379851257fe5d85632d31d5ec` |
| LIBERO submodule | `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c` |

OpenPI pin 已有记录，不是当前待选项。本轮依据来源文件/gitlink与服务器HEAD；
不把旧 clean 状态推定成所有当前未提交项目代码都 clean。
实际实验代码还由 training provenance/source hashes 追踪。

## Historical planning contract

[openpi_contract.json](../../provenance/openpi_contract.json) 的 LOCAL_PIN_PENDING 与
local_commit=null 是 HISTORICAL 初始规划快照，保留原文件。它不覆盖后来
openpi_git.json 的实际 pin，也不代表训练尚未开始。
权重、数据、norm、checkpoint 身份见各 training report 和 service identity；
本页不复制维护多份 checkpoint hash。
