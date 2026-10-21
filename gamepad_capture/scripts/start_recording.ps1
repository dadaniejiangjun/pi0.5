param(
    [Alias('Duration')]
    [ValidateRange(0,3600)][int]$MaxCaptureDuration = 0,
    [string]$Task = 'Pick up the red cube and place it on the green desk mat.',
    [string]$Operator = $env:USERNAME,
    [switch]$SingleEpisode,
    [switch]$SkipReview,
    [switch]$Review,
    [Alias('AutoHomeAfterSave')]
    [switch]$HomeBeforeEachEpisode,
    [switch]$SkipHomeBeforeRecording,
    [string]$TaskScopeId = ''
)
$ErrorActionPreference = 'Stop'
$Project = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$Cache = Join-Path $Project 'cache/live_lerobot'
foreach ($Subdir in @('hf','datasets','tmp')) {
    New-Item -ItemType Directory -Force (Join-Path $Cache $Subdir) | Out-Null
}
$env:HF_HOME = Join-Path $Cache 'hf'
$env:HF_DATASETS_CACHE = Join-Path $Cache 'datasets'
$env:HF_HUB_OFFLINE = '1'
$env:HF_DATASETS_OFFLINE = '1'
$env:TEMP = Join-Path $Cache 'tmp'
$env:TMP = $env:TEMP
Write-Host 'Recorder: manual START; press RB to stop; then enter SUCCESS or FAILURE.' -ForegroundColor Green
if ($MaxCaptureDuration -gt 0) {
    Write-Host "Safety time limit: $MaxCaptureDuration s. RB remains the normal stop." -ForegroundColor Yellow
} else {
    Write-Host 'No automatic duration stop. RB is the normal stop.' -ForegroundColor Yellow
}
Write-Host 'Keep sticks neutral until CAPTURE_START. Do not start a second recorder.' -ForegroundColor Yellow
if (-not $SkipHomeBeforeRecording) {
    Write-Host 'Every new recording will return to zero first; a failed home blocks recording.' -ForegroundColor Cyan
} else {
    Write-Host 'Pre-recording home is explicitly skipped. Do not use this mode for a normal demonstration.' -ForegroundColor Yellow
}
$ShouldHomeBeforeRecording = -not $SkipHomeBeforeRecording
if ($HomeBeforeEachEpisode -and $SkipHomeBeforeRecording) {
    throw 'Cannot combine -HomeBeforeEachEpisode/-AutoHomeAfterSave with -SkipHomeBeforeRecording.'
}
$WrapperExit = 0
Push-Location $Project
try {
    if (-not $SingleEpisode) {
        Write-Host 'Persistent session: load once; START/NEXT -> home -> capture; RB -> label -> save. OVER exits.' -ForegroundColor Cyan
        Write-Host 'Review videos are deferred until OVER; do not start a second recorder.' -ForegroundColor Yellow
        $Before = @(Get-ChildItem (Join-Path $Project 'gamepad_capture/data/lerobot') -Directory -Filter 'live_*' | ForEach-Object FullName)
        $SessionArgs = @{
            Mode='Record'; RecorderBackend='LeRobot'; SensorSource='Native';
            RosDomainId=222; SiteConfirmed=$true; Operator=$Operator; Task=$Task;
            ManualLifecycle=$true; ContinuousEpisodes=$true
        }
        if ($ShouldHomeBeforeRecording) { $SessionArgs.HomeBeforeEachEpisode = $true }
        if ($TaskScopeId) { $SessionArgs.TaskScopeId = $TaskScopeId }
        if ($MaxCaptureDuration -gt 0) { $SessionArgs.MaxCaptureDuration = $MaxCaptureDuration }
        & (Join-Path $PSScriptRoot 'gamepad_teleop.ps1') @SessionArgs
        $WrapperExit = $LASTEXITCODE
        $New = @(Get-ChildItem (Join-Path $Project 'gamepad_capture/data/lerobot') -Directory -Filter 'live_*' | Where-Object { $_.FullName -notin $Before })
        if ($Review -and -not $SkipReview -and $New.Count -eq 1) {
            $ReviewDir = Join-Path $Project ('gamepad_capture/data/visualizations/' + $New[0].Name)
            & 'E:\anaconda3\envs\pi05_lerobot_dataset\python.exe' -B -m gamepad_capture.tools.visualization.live_review `
                --dataset $New[0].FullName --output-dir $ReviewDir
            if ($LASTEXITCODE -ne 0 -and $WrapperExit -eq 0) { $WrapperExit = 4 }
        }
        exit $WrapperExit
    }
    do {
        # A new recording always starts from a known pose. The service runs inside
        # the existing single CAN owner; a rejected home blocks this episode.
        $HomeExit = 0
        if ($ShouldHomeBeforeRecording) {
            Write-Host 'Returning to zero before this recording through the existing single CAN owner...' -ForegroundColor Cyan
            $HomeArgs = @{ Mode='HomeService'; SensorSource='Native'; RosDomainId=222; SiteConfirmed=$true }
            if ($TaskScopeId) { $HomeArgs.TaskScopeId = $TaskScopeId }
            & (Join-Path $PSScriptRoot 'gamepad_teleop.ps1') @HomeArgs
            $HomeExit = $LASTEXITCODE
            if ($HomeExit -ne 0) {
                $WrapperExit = $HomeExit
                Write-Warning 'Pre-recording home failed; this recording was not started and no second CAN owner was opened.'
                break
            }
        }

        $Before = @(Get-ChildItem (Join-Path $Project 'gamepad_capture/data/lerobot') -Directory -Filter 'live_*' | ForEach-Object FullName)
        $RecordArgs = @{
            Mode = 'Record'
            RecorderBackend = 'LeRobot'
            SensorSource = 'Native'
            RosDomainId = 222
            SiteConfirmed = $true
            Operator = $Operator
            Task = $Task
            ManualLifecycle = $true
        }
        if ($MaxCaptureDuration -gt 0) {
            $RecordArgs.MaxCaptureDuration = $MaxCaptureDuration
        }
        & (Join-Path $PSScriptRoot 'gamepad_teleop.ps1') @RecordArgs
        $CaptureExit = $LASTEXITCODE
        $New = @(Get-ChildItem (Join-Path $Project 'gamepad_capture/data/lerobot') -Directory -Filter 'live_*' | Where-Object { $_.FullName -notin $Before })
        if ($CaptureExit -ne 0) {
            $WrapperExit = $CaptureExit
            Write-Warning "Recorder exit=$CaptureExit; this capture is incomplete and is not a successful demonstration."
        }
        if ($New.Count -ne 1) {
            if ($WrapperExit -eq 0) { $WrapperExit = 2 }
            Write-Warning 'Cannot uniquely identify this dataset; review was not launched.'
            break
        }

        if ($CaptureExit -eq 0) {
            Write-Host 'Dataset saved with online q(t+100ms) labels. No post-recording conversion is required. Video encoding is completed by the writer.' -ForegroundColor Green
        }
        if ($Review -and -not $SkipReview) {
            $ReviewDir = Join-Path $Project ('gamepad_capture/data/visualizations/' + $New[0].Name)
            & 'E:\anaconda3\envs\pi05_lerobot_dataset\python.exe' -B -m gamepad_capture.tools.visualization.live_review `
                --dataset $New[0].FullName --output-dir $ReviewDir
            if ($LASTEXITCODE -ne 0) {
                if ($WrapperExit -eq 0) { $WrapperExit = 4 }
                Write-Warning 'Review failed; dataset preserved. Inspect error before next episode.'
                break
            }
            Write-Host "Review video and diagnostics: $ReviewDir" -ForegroundColor Green
        }
        if ($CaptureExit -ne 0) {
            Write-Warning 'Incomplete capture: review generated where possible; ensure the next new recording starts with a successful pre-home.'
            break
        }
        if ($SingleEpisode) { break }
        $Next = Read-Host 'Task objects ready. Enter NEXT for another episode; the next recording will home first. Anything else exits'
    } while ($Next -ceq 'NEXT')
} finally { Pop-Location }
exit $WrapperExit
