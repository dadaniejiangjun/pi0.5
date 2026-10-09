---
Document status: REFERENCE
Current authority: docs/RUNBOOK.md
Last reviewed: 2026-10-08
---

# Laptop bring-up reference

当前操作入口唯一见 [RUNBOOK](../docs/RUNBOOK.md)，本页给出首次检查顺序。

1. 使用已配置的 `pi05_laptop` 解释器，检查依赖与设备身份。
2. 复用 `laptop.camera.dual_camera` 验证两路画面，目视确认 Base/Wrist。
3. 复用 `laptop.robot_client.piper_tool` 检查反馈/模式/电机状态。
4. 按 RUNBOOK 选择 service profile、取得匹配 identity lock 并建立 SSH tunnel。
5. 先 `live_model_preview`，再在当前授权与现场确认下决定是否执行 bounded trial。

相机可用不代表曝光同步，shadow成功不代表机器人任务成功。
复位、使能、回零和试跑是不同操作，不能由首次连接自动触发。
历史 P2-only / 8000 handoff 不再作为默认入口；按日期的记录仍留在报告中。
