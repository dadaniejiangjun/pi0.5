---
Document status: REFERENCE
Current authority: docs/ACTION_SEMANTICS.md
Last reviewed: 2026-10-08
---

> **REFERENCE**：以下是技术参考或有日期范围的规格/证据，不独立维护当前部署状态。 当前领域主文档见 [authority](../../docs/ACTION_SEMANTICS.md)。

# 官方上位机 5 秒命令记录

程序 `capture_oem_can.py` 观察指定 release 上位机的 `candle_frame_send@12`。
无需打开第二个 CAN 连接。观察钩子会临时修改进程内存以记录函数调用，但不会
修改参数、替换函数行为或调用发送接口；会带来运行开销，不能宣称零影响。
退出后卸载钩子，磁盘 EXE 不变。只支持已核对的 classic candle_frame_t 布局，
不包含 FD 接口；通过该接口调用不能证明机器人已执行或已经成功送达 CAN 总线。

## 开始记录

确认现场安全，准备在官方上位机手动操作夹爪。先获取当前进程 PID：

```powershell
Get-Process -Name 'ArmRobotUA_V1.6.3.260828_release' | Select-Object Id
```

在项目根目录运行（将 PID 替换为实际值，输出目录必须是新的）：

```powershell
& 'E:\anaconda3\envs\pi05_laptop\python.exe' -B -m laptop.diagnostics.capture_oem_can --pid 54444 --output-dir validation/oem_gripper_capture_20261007_r1
```

看到 READY 后按 Enter；看到 CAPTURE_START 后在上位机操作一次夹爪。
记录窗口固定为 **5 秒**，结束输出 CAPTURE_END 并卸载钩子。不自动操作上位机。
Codex 调用须在沙箱外运行；进程重启后重新读取 PID。

输出 `send_frames.jsonl` 包含发送入口 QPC 时钟、CAN ID、8 字节数据和 API 返回值。
`report.json` 包含 QPC 频率、上位机哈希、发送帧计数和夹爪参数。
相邻同类帧 QPC ticks 差除以 frequency 即调用间隔；这不是 CAN 硬件总线时间戳。
帧布局参考 candle API 的 packed 24 字节 classic 结构，首次实测仍需与上位机
显示值核对，不能把无数据或解析异常称为有效命令采集。

本地依赖在 `cache/tooldeps/oem_capture`，已安装 Frida 17.22.2；不上传该目录。
`--probe-hook` 仅测试未启用的挂接，不开始记录。目前此项验证通过，实际夹爪
操作记录尚未执行。

接口参考：[Frida API](https://frida.re/docs/javascript-api/)；
[candle API 结构](https://github.com/HubertD/cangaroo/blob/master/src/driver/CandleApiDriver/api/candle.h)。
