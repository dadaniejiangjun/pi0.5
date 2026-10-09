---
Document status: REFERENCE
Current authority: docs/CURRENT_STATUS.md
Last reviewed: 2026-10-09
---

# GitHub source snapshot

目标仓库：[dadaniejiangjun/pi0.5](https://github.com/dadaniejiangjun/pi0.5)。本次保存本地当前代码、回归测试、配置模板、官方 OpenPI pin、项目文档和历史源码，不代表新增实机部署或训练结果。

## Structure

- laptop/：当前相机、通信、数采、推理及受监督控制支持。
- server/：当前数据转换、训练、评估、服务及实际底层依赖。
- tests/：当前链路回归；公开反馈 fixture 是 synthetic。
- configs/、provenance/：配置/模板与版本来源；template 不是现场已验收配置。
- docs/：状态、架构、语义、操作指南与历史。
- archive/：历史实现与测试，默认不运行。
- reports/：选定代码审计、重构和清理说明；不是原始运行日志。
- third_party/openpi：官方 Physical-Intelligence/openpi 子模块，固定 commit `215abfb217dbac7d5f1273282331b9b1866c0479`。

## Local-only artifacts

真实 CAN/视频/相机采集、raw/incoming/derived dataset、capture sessions、validation 运行产物、模型 checkpoint、cache、环境、凭据和临时日志保持本地。文档中的本地证据链接在 GitHub 上可能不可访问；它们不是已上传附件，实机/训练历史仍由本地及服务器的原报告证明。

公开测试使用人工构造协议样本，本地可继续使用未发布的 recorded feedback fixture。仅克隆仓库不会获得三条示教、训练权重或可直接启动的当前机器配置。运行前按 RUNBOOK 配置环境、数据与服务身份。

## Verification

已核对官方 submodule remote、HEAD 和 clean 状态。代码清理回归通过；本次强制使用 public synthetic fixture 核验：537 项测试、61 个子测试通过，3 项条件跳过。公开样本含 10 帧，比本地 5 帧样本多 5 项参数化检查；生产代码没有变化。远端旧 P0/P1 整理提交保留历史，当前状态以本地当前 authority 文档为准。实际保存 commit 与远端结果由 git log/远端分支及本地上传报告确认，不用本页假定推送已完成。
