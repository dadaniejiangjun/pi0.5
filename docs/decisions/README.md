---
Document status: REFERENCE
Current authority: docs/CURRENT_STATUS.md
Last reviewed: 2026-10-08
---

# Decision references

本索引追踪已存在的决策来源，不为旧pending字段伪造新的批准记录。
当前状态见 [CURRENT_STATUS](../CURRENT_STATUS.md)，运行profile见 [RUNBOOK](../RUNBOOK.md)。

## Existing evidence

实际使用与当初规划分开读取。

| 主题 | 来源与当前边界 |
|---|---|
| 官方OpenPI来源 | [OPENPI_PROVENANCE](../reference/OPENPI_PROVENANCE.md)，实际pin不再pending |
| SSH / loopback部署 | 现有service argv、RUNBOOK named profiles；旧备选见历史NETWORK_OPTIONS |
| π0.5 base及checkpoint | training report/provenance和service identity；旧待定权重计划保留 |
| 未来状态代理 | [ACTION_SEMANTICS](../ACTION_SEMANTICS.md)，不能改称原厂command标签 |
| 现场运动授权 | [保留的按日期执行历史](../reference/CURRENT_PROJECT_PLAN.md)与trial记录；历史授权不是任意新动作授权 |
| 旧初始决策清单 | [USER_DECISIONS](../history/USER_DECISIONS.md)，HISTORICAL |

## Needs human decision

以下事项尚不能由本轮文档整理自行决定。

1. 后续连续控制试验的范围与正式机械/夹爪标定验收。
2. 更多示教、held-out任务场景和任务成功验收标准。
3. 哪些旧实验继续支持、生成物保留期限及未来归档/迁移范围。

统一机器可读deployment profile和versioned action schema是后续工程建议，
本轮没有新建、替换配置，也没有形成新的ADR批准记录。
