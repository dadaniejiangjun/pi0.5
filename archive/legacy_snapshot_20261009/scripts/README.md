---
Document status: REFERENCE
Current authority: docs/RUNBOOK.md
Last reviewed: 2026-10-09
---

# Offline tools and historical report helpers

这些工具各有独立用途，不是机器人运行入口。当前操作 authority 是
[RUNBOOK](../docs/RUNBOOK.md)。不能用工具重写 gate 状态来代替验收。

## Tool index

以下列出实际文件，是否已运行及结果需要查相应日期报告。

| Tool | 用途 |
|---|---|
| `audit_laptop_probe.py` | 笔记本探针证据审计 |
| `refresh_environment_audit.py` | 环境来源/依赖审计 |
| `run_local_policy_server_smoke.py` | 本地策略服务 smoke |
| `run_pi05_base_smoke.py` | 官方 base 模型 smoke |
| `run_server_performance.py` | 服务器性能测量 |
| `validate_piper_continuous.py` | 连续控制相关验证工具，存在不代表当前 scheduler 已实现 |
| `validate_piper_contract.py` | 机器契约校验 |
| `write_checkpoint_provenance.py` | checkpoint 来源记录 |
| `write_deployment_manifest.py` | 部署 manifest 生成 |
| `write_socket_binding_report.py` | socket 绑定报告 |

## Deprecated scripts

以下两项 DEPRECATED，只保留来源/旧实验复现，不作为当前部署或状态更新工具。
Phase A.5 已归档以下两项，文件内容未改，未运行脚本。

| File | 原用途 | Archived location |
|---|---|---|
| `add_final_gate_aliases.py` | 对初始manifest一次性写 gate alias；重放会写PASS/YES | [add_final_gate_aliases.py](../archive/deprecated_scripts/add_final_gate_aliases.py) |
| `annotate_pi05_smoke_metadata.py` | 给固定旧ALOHA smoke补元数据 | [annotate_pi05_smoke_metadata.py](../archive/deprecated_scripts/annotate_pi05_smoke_metadata.py) |

来源与复现限制见 [Archive Manifest](../archive/ARCHIVE_MANIFEST.md)。
