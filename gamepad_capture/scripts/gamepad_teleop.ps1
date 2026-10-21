param(
    [ValidateSet('Help','NewScope','Status','CanMode','Enable','Zero','HomeService','ClearJ5','Control','Cameras','Deploy','Check','Sources','Bridge','InputPreview','XInputProbe','XInputPreview','XInputSend','Preview','XInputOperatorPreview','Execute','Record','AuditInput','AuditCapture')]
    [string]$Mode = 'Check',
    [string]$Operator = $env:USERNAME,
    [string]$Task = 'Pick up the black rod and place it on the green desk mat.',
    [ValidateSet('Windows','Native')][string]$SensorSource = 'Windows',
    [switch]$CheckEnvironment,
    [switch]$SiteConfirmed,
    [switch]$DryRun,
    [ValidateRange(1,3600)][int]$ControlDuration = 600,
    [string]$PythonPath = 'E:\anaconda3\envs\pi05_laptop\python.exe',
    [ValidateSet('yuyv','mjpeg2rgb')][string]$WristPixelFormat = 'mjpeg2rgb',
    [ValidateRange(0,3600)][int]$SourceDuration = 0,
    [switch]$Cameras,
    [ValidateRange(1,120)][int]$Duration = 20,
    [string]$TaskScopeId = '',
    [switch]$DisableJointSmoothing,
    [string]$JoyDeviceName = '',
    [ValidateRange(0,3)][int]$XInputIndex = 0,
    [ValidateRange(1,300)][int]$XInputProbeDuration = 15,
    [ValidateRange(1,100)][int]$XInputPollHz = 50,
    [ValidateRange(1,65535)][int]$XInputPort = 18901,
    [string]$XInputHost = '127.0.0.1',
    [ValidateRange(0,3600)][int]$XInputDuration = 0,
    [switch]$WaitForOperatorStart,
    [ValidateSet('Rosbag','LeRobot')][string]$RecorderBackend = 'Rosbag',
    [string]$LeRobotPythonPath = 'E:\anaconda3\envs\pi05_lerobot_dataset\python.exe',
    [ValidateSet(231,222)][int]$RosDomainId = 231,
    [switch]$ManualLifecycle,
    [switch]$ContinuousEpisodes,
    [switch]$HomeBeforeEachEpisode,
    [ValidateRange(0,3600)][int]$MaxCaptureDuration = 0,
    [ValidateRange(0,10)][int]$LifecycleTailSeconds = 1,
    [ValidateRange(1,300)][int]$FinishTimeoutSeconds = 180
)
$ErrorActionPreference = 'Stop'
if ($Mode -eq 'Help') {
    Write-Output @'
Status       只读机械臂状态（WSL SocketCAN）
NewScope     本机生成一个新的 TaskScopeId；不访问 WSL 或设备
Enable       待机且六轴全失能时单次使能；不清故障，需 -SiteConfirmed
CanMode      示教/待机切CAN，需 -SiteConfirmed
Zero         分段回零，需 -SiteConfirmed
HomeService  通过当前控制节点在同一 CAN owner 内分段回零，需 -SiteConfirmed
ClearJ5      仅J5故障恢复，需 -SiteConfirmed
Control      原生CAN控制服务，需 -SiteConfirmed -TaskScopeId NAME
Execute      手柄执行器，原生链路加 -SensorSource Native -TaskScopeId NAME
XInputSend   Windows手柄输入，默认持续运行；没有 TaskScopeId 参数
Cameras      原生双相机源，不另开CAN；默认 Wrist mjpeg2rgb，延迟8秒启动
Record       录制；原生链路加 -SensorSource Native -Operator NAME -Duration 20
             -RecorderBackend LeRobot 使用锁定Windows LeRobot Python + WSL只读ROS适配器
             -ManualLifecycle 等待 CAPTURE_READY 后输入 START；RB上升沿或 STOP 结束
             -ContinuousEpisodes 可在常驻数据源下连续录多个独立 run_id
             -FinishTimeoutSeconds 控制 LeRobot episode 写入收尾等待，默认180秒、最大300秒
              尾录结束后可标记 SUCCESS、FAILURE、RETAKE 或 SAVE；标签后等 EPISODE_SAVED
             连续模式在 SEQUENCE_READY 时输入 OVER 结束序列；默认不设时长上限
             -MaxCaptureDuration 秒可选；控制scope到期不结束录制
LeRobot 手动模式先显示 CAPTURE_START_PUBLISH_CALLED；等 operator 的
CAPTURE_OPERATOR_ACK: status=RECORDING_RESUMED_SOFTWARE_ONLY 后再操作。
-DryRun 仅显示将调用的参数，不启动 WSL、ROS、相机或控制进程，也不要求现场确认。
Control 与 Execute 必须使用同一个 TaskScopeId；XInputSend 无 scope 参数。
'@
    exit 0
}
if ($Mode -eq 'NewScope') {
    $scopeRng = [System.Security.Cryptography.RandomNumberGenerator]::Create()
    $scopeBytes = New-Object byte[] 8
    $scopeRng.GetBytes($scopeBytes)
    $scopeRng.Dispose()
    $scopeSuffix = [BitConverter]::ToString($scopeBytes).Replace('-', '').ToLowerInvariant()
    Write-Output "gp_$(Get-Date -Format 'yyyyMMdd_HHmmss')_$scopeSuffix"
    exit 0
}
if ($Mode -in @('CanMode','Enable','Zero','HomeService','ClearJ5','Control') -and -not $SiteConfirmed -and -not $DryRun) {
    throw '现场确认后传入 -SiteConfirmed；先停止旧执行器/CAN owner，机械臂无受力卡住、姿态稳定、急停可用。'
}
if ($Mode -in @('Control','Execute') -and $TaskScopeId -notmatch '^[A-Za-z0-9_-]{1,128}$') {
    throw 'TaskScopeId 必须为 1–128 个 ASCII 字母、数字、下划线或连字符。'
}
if ($WaitForOperatorStart -and $Mode -ne 'Record') {
    throw '-WaitForOperatorStart 仅可与 -Mode Record 一起使用。'
}
if (($ManualLifecycle -or $ContinuousEpisodes -or $MaxCaptureDuration -ne 0) -and $Mode -ne 'Record') {
    throw 'ManualLifecycle、ContinuousEpisodes 和 MaxCaptureDuration 仅可与 -Mode Record 一起使用。'
}
if ($RecorderBackend -ne 'Rosbag' -and $Mode -ne 'Record') {
    throw '-RecorderBackend 仅可与 -Mode Record 一起使用。'
}
if ($Mode -eq 'Record' -and $RecorderBackend -eq 'LeRobot' -and
    $RosDomainId -eq 222 -and -not $SiteConfirmed -and -not $DryRun) {
    throw '实时 LeRobot 录制使用 ROS domain 222 时必须传入 -SiteConfirmed。'
}
if ($ContinuousEpisodes -and -not $ManualLifecycle) {
    throw '-ContinuousEpisodes 必须与 -ManualLifecycle 一起使用。'
}
if ($ManualLifecycle -and $WaitForOperatorStart) {
    throw '人工生命周期模式已等待 CAPTURE_READY 和 START；不要再传 -WaitForOperatorStart。'
}
if ($Mode -in @('Status','CanMode','Enable','Zero','HomeService','ClearJ5','Control','Cameras')) { $SensorSource = 'Native' }
if ($Mode -in @('XInputSend','XInputProbe')) {
    if (-not (Test-Path -LiteralPath $PythonPath)) { throw "找不到 Windows Python：$PythonPath" }
    if ($Mode -eq 'XInputSend') {
        $pythonArgs = @('-B',(Join-Path $PSScriptRoot '..\windows\xinput_sender.py'),
            '--host',$XInputHost,'--port',"$XInputPort",'--index',"$XInputIndex",
            '--poll-hz',"$XInputPollHz",'--duration-s',"$XInputDuration")
    } else {
        $pythonArgs = @('-B',(Join-Path $PSScriptRoot '..\windows\xinput_probe.py'),
            '--duration-s',"$XInputProbeDuration",'--poll-hz',"$XInputPollHz")
    }
    if ($DryRun) { @{ executable=$PythonPath; arguments=$pythonArgs } | ConvertTo-Json -Depth 4; exit 0 }
    & $PythonPath @pythonArgs
    exit $LASTEXITCODE
}
if ($Mode -eq 'Sources' -and $SensorSource -ne 'Native') {
    throw 'Sources 会复用 P4 原生只读数据源；请指定 -SensorSource Native。'
}
if ($SensorSource -eq 'Native' -and $Mode -eq 'Bridge') {
    throw 'Native 仅选择现有 Linux 传感器话题用于预览或录制；不要用此来源启动 Windows CAN bridge。'
}
$wslProject = '/mnt/e/01_NUS/02_5003/12_pi0.5'
$argsForWsl = @('-d','Ubuntu-22.04','--exec','env',
    "PI05_GAMEPAD_SENSOR_SOURCE=$($SensorSource.ToLowerInvariant())",
    "PI05_GAMEPAD_SOURCE_DURATION_S=$SourceDuration",'/bin/bash')
switch ($Mode) {
    { $_ -in @('Status','CanMode','Enable','Zero','ClearJ5') } {
        $operation = @{ Status='status'; CanMode='can-mode'; Zero='zero'; ClearJ5='clear-j5' }[$Mode]
        if ($Mode -eq 'Enable') { $operation = 'enable' }
        $moduleArgs = @('-B','-m','laptop.robot_client.native_piper_tool',$operation)
        if ($Mode -eq 'Enable') { $moduleArgs = @('-B','-m','gamepad_capture.robot.enable_standby') }
        if ($SiteConfirmed) { $moduleArgs += '--site-confirmed' }
        $argsForWsl = @('-d','Ubuntu-22.04','--cd',$wslProject,'--exec','/usr/bin/python3') + $moduleArgs
    }
    'HomeService' {
        $argsForWsl = @('-d','Ubuntu-22.04','--exec','env',"ROS_DOMAIN_ID=$RosDomainId",'/bin/bash',
            "$wslProject/gamepad_capture/scripts/request_native_home.sh")
        if ($TaskScopeId) { $argsForWsl += @('--task-scope-id',$TaskScopeId) }
    }
    'Control' {
        $argsForWsl += "$wslProject/gamepad_capture/scripts/launch_native_manual_control.sh"
        if ($SiteConfirmed) { $argsForWsl += @('--onsite-confirmed',$TaskScopeId) }
        $argsForWsl += "$ControlDuration"
    }
    'Cameras' {
        $argsForWsl += @("$wslProject/gamepad_capture/scripts/launch_native_capture_sources.sh",
            '--cameras-only')
        if ($CheckEnvironment) { $argsForWsl += '--check-environment' }
        $argsForWsl += "wrist_pixel_format:=$WristPixelFormat"
    }
    'Deploy' { $argsForWsl += "$wslProject/gamepad_capture/scripts/setup_gamepad_capture.sh" }
    'Check' { $argsForWsl += "$wslProject/gamepad_capture/scripts/check_gamepad.sh" }
    'Sources' {
        $argsForWsl += "$wslProject/gamepad_capture/scripts/launch_native_capture_sources.sh"
        if ($CheckEnvironment) { $argsForWsl += '--check-environment' }
        $argsForWsl += "wrist_pixel_format:=$WristPixelFormat"
    }
    'Bridge' {
        $argsForWsl += "$wslProject/gamepad_capture/scripts/launch_gamepad_bridge.sh"
        if ($Cameras) { $argsForWsl += '--cameras' }
    }
    'InputPreview' {
        $argsForWsl += @("$wslProject/gamepad_capture/scripts/launch_gamepad_input_preview.sh")
        if ($JoyDeviceName) { $argsForWsl += "joy_device_name:=$JoyDeviceName" }
    }
    'XInputPreview' {
        $argsForWsl += @("$wslProject/gamepad_capture/scripts/launch_gamepad_xinput_preview.sh",
            "host:=$XInputHost", "port:=$XInputPort")
    }
    'Preview' {
        $argsForWsl += @("$wslProject/gamepad_capture/scripts/launch_gamepad_capture.sh")
        if ($JoyDeviceName) { $argsForWsl += "joy_device_name:=$JoyDeviceName" }
        $argsForWsl += @('execute:=false')
    }
    'XInputOperatorPreview' {
        $argsForWsl += @("$wslProject/gamepad_capture/scripts/launch_gamepad_capture.sh",
            'input_source:=xinput','execute:=false')
    }
    'Execute' {
        if ($TaskScopeId -notmatch '^[A-Za-z0-9_-]{1,128}$') {
            throw '执行前必须提供本机生成且经现场确认的 TaskScopeId。'
        }
        $argsForWsl += @("$wslProject/gamepad_capture/scripts/launch_gamepad_capture.sh")
        if ($JoyDeviceName) { $argsForWsl += "joy_device_name:=$JoyDeviceName" }
        $argsForWsl += @('input_source:=xinput','execute:=true',"task_scope_id:=$TaskScopeId")
        if ($DisableJointSmoothing) { $argsForWsl += 'joint_target_smoothing:=false' }
    }
    'Record' {
        if ($RecorderBackend -eq 'LeRobot') {
            $projectRoot = 'E:\01_NUS\02_5003\12_pi0.5'
            $leRobotArgs = @('-B','-m','gamepad_capture.live_dataset.cli','live',
                '--project-root',$projectRoot,'--operator',$Operator,'--task',$Task,
                '--domain-id',"$RosDomainId",
                '--finish-timeout',"$FinishTimeoutSeconds")
            if ($SensorSource -eq 'Native') { $leRobotArgs += @('--sensor-source','native') }
            if ($SiteConfirmed) { $leRobotArgs += '--site-confirmed' }
            if ($ManualLifecycle) {
                if ($HomeBeforeEachEpisode) {
                    $leRobotArgs += '--home-before-start'
                    if ($TaskScopeId) { $leRobotArgs += @('--home-task-scope-id',$TaskScopeId) }
                }
                $leRobotArgs += @('--manual-lifecycle','--lifecycle-tail',"$LifecycleTailSeconds")
                if ($MaxCaptureDuration -gt 0) {
                    $leRobotArgs += @('--max-capture-duration',"$MaxCaptureDuration")
                }
                if ($ContinuousEpisodes) { $leRobotArgs += '--continuous-episodes' }
            } else {
                $leRobotArgs += @('--duration',"$Duration")
            }
        } else {
            $recordArgs = @("$wslProject/gamepad_capture/scripts/record_gamepad_demo.sh",
                '--operator',$Operator,'--task',$Task)
            if ($ManualLifecycle) {
                $recordArgs += @('--manual-lifecycle','--lifecycle-tail',"$LifecycleTailSeconds")
                if ($MaxCaptureDuration -gt 0) {
                    $recordArgs += @('--max-capture-duration',"$MaxCaptureDuration")
                }
                if ($ContinuousEpisodes) { $recordArgs += '--continuous-episodes' }
            } else {
                $recordArgs += @('--duration',"$Duration")
            }
            $argsForWsl += $recordArgs
        }
    }
    'AuditInput' {
        $argsForWsl += @("$wslProject/gamepad_capture/scripts/audit_gamepad.sh",
            '--mode','input','--duration',"$Duration")
    }
    'AuditCapture' {
        $argsForWsl += @("$wslProject/gamepad_capture/scripts/audit_gamepad.sh",
            '--mode','capture','--duration',"$Duration")
    }
}
if ($DryRun) {
    if ($Mode -eq 'Record' -and $RecorderBackend -eq 'LeRobot') {
        $dryRunReport = @{ executable=$LeRobotPythonPath; arguments=$leRobotArgs
            backend='lerobot_live'; ros_domain_id=$RosDomainId
            ros_connected=$false; can_tx=0; camera_source_started=$false }
        if ($RosDomainId -eq 222 -and -not $SiteConfirmed) {
            $dryRunReport.execution_precondition_missing = '-SiteConfirmed'
        }
    } else {
        $dryRunReport = @{ executable='wsl.exe'; arguments=$argsForWsl }
    }
    if ($Mode -in @('CanMode','Enable','Zero','HomeService','ClearJ5','Control') -and -not $SiteConfirmed) {
        $dryRunReport.execution_precondition_missing = '-SiteConfirmed'
    }
    $dryRunReport | ConvertTo-Json -Depth 4
    exit 0
}
if ($Mode -eq 'Record' -and $WaitForOperatorStart) {
    Write-Host ''
    Write-Host "录制器尚未启动；窗口时长：$Duration 秒。" -ForegroundColor Yellow
    Write-Host '手柄保持中立。现场准备好后，在下方输入 START。'
    Write-Host '确认后会检查数据源并等待录制器倒计时；看到绿色 CAPTURE_START 再开始操作。'
    $operatorStart = Read-Host '输入 START 启动录制器；输入其他内容取消'
    if ($operatorStart -cne 'START') {
        Write-Host '已取消：录制器尚未启动，没有创建录制数据。' -ForegroundColor Yellow
        exit 0
    }
    Write-Host '已确认；开始检查数据源并准备录制。' -ForegroundColor Green
}
if ($Mode -eq 'Record') {
    # Forward recorder output line by line and make the actual start event hard to miss.
    if ($RecorderBackend -eq 'LeRobot') {
        if (-not (Test-Path -LiteralPath $LeRobotPythonPath)) {
            throw "找不到锁定版本的 LeRobot Python：$LeRobotPythonPath"
        }
        Push-Location 'E:\01_NUS\02_5003\12_pi0.5'
        try {
            & $LeRobotPythonPath @leRobotArgs
        } finally {
            Pop-Location
        }
    } else {
        & wsl.exe @argsForWsl 2>&1 | ForEach-Object {
        $recordLine = [string]$_
        if ($recordLine -match '^CAPTURE_START:' -or
            $recordLine -match '^CAPTURE_START_PUBLISH_CALLED:') {
            Write-Host ''
            Write-Host '============================================' -ForegroundColor Green
            Write-Host 'CAPTURE STARTED - BEGIN THE DEMONSTRATION' -ForegroundColor Black -BackgroundColor Green
            Write-Host '============================================' -ForegroundColor Green
            try { [Console]::Beep(1000, 180) } catch { }
        } elseif ($recordLine -match '^CAPTURE_SCOPE_(WARNING|EXPIRED):') {
            Write-Host $recordLine -ForegroundColor Yellow
            continue
        } elseif ($recordLine -match '^CAPTURE_STOP_REQUESTED:') {
            Write-Host $recordLine -ForegroundColor Yellow
            continue
        } elseif ($recordLine -match '^CAPTURE_FINALIZED: status=CAPTURE_COMPLETE;') {
            Write-Host $recordLine -ForegroundColor Green
            try { [Console]::Beep(740, 180) } catch { }
            continue
        } elseif ($recordLine -match '^CAPTURE_FINALIZED:') {
            Write-Host $recordLine -ForegroundColor Yellow
            continue
        }
        Write-Host $recordLine
        }
    }
} else {
    & wsl.exe @argsForWsl
}
exit $LASTEXITCODE
