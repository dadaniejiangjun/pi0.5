[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Dataset,

    [string]$OutputDataset = '',

    [string]$ReviewDir = ''
)

$ErrorActionPreference = 'Stop'
$Project = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
Push-Location $Project
try {
    $Python = 'E:\anaconda3\envs\pi05_lerobot_dataset\python.exe'
    if (-not (Test-Path -LiteralPath $Python -PathType Leaf)) {
        throw "Pinned Python environment is missing: $Python"
    }
    $SourceDataset = (Resolve-Path -LiteralPath $Dataset).Path
    if ($OutputDataset) {
        $DerivedDataset = [System.IO.Path]::GetFullPath($OutputDataset)
    }
    else {
        $Name = Split-Path -Leaf $SourceDataset
        $DerivedDataset = Join-Path $Project "gamepad_capture\data\lerobot\derived\${Name}_proxy100ms"
    }
    if ($ReviewDir) {
        $ReviewRoot = [System.IO.Path]::GetFullPath($ReviewDir)
    }
    else {
        $Name = Split-Path -Leaf $DerivedDataset
        $ReviewRoot = Join-Path $Project "gamepad_capture\data\visualizations\$Name\episode_000000"
    }

    $CacheRoot = Join-Path $Project 'tmp\prepare_live_training'
    New-Item -ItemType Directory -Force -Path `
        (Join-Path $CacheRoot 'hf'), `
        (Join-Path $CacheRoot 'datasets'), `
        (Join-Path $CacheRoot 'tmp') | Out-Null
    $env:HF_HOME = Join-Path $CacheRoot 'hf'
    $env:HF_DATASETS_CACHE = Join-Path $CacheRoot 'datasets'
    $env:HF_HUB_OFFLINE = '1'
    $env:HF_DATASETS_OFFLINE = '1'
    $env:TEMP = Join-Path $CacheRoot 'tmp'
    $env:TMP = $env:TEMP

    $BridgeReport = "$DerivedDataset.bridge_report.json"
    if (-not (Test-Path -LiteralPath $DerivedDataset -PathType Container)) {
        & $Python -B -m server.training.live_training_bridge `
            --source-dataset $SourceDataset `
            --output-dataset $DerivedDataset
        if ($LASTEXITCODE -ne 0) {
            throw "live_training_bridge failed with exit code $LASTEXITCODE"
        }
    }
    if (-not (Test-Path -LiteralPath $BridgeReport -PathType Leaf)) {
        throw "Derived dataset exists but its bridge report is missing: $BridgeReport"
    }

    $ReviewMovie = Join-Path $ReviewRoot 'review.mp4'
    $ReviewDiagnostics = Join-Path $ReviewRoot 'diagnostics.json'
    if (-not (Test-Path -LiteralPath $ReviewMovie -PathType Leaf)) {
        & $Python -B -m gamepad_capture.tools.visualization.live_review `
            --dataset $DerivedDataset `
            --output-dir $ReviewRoot
        if ($LASTEXITCODE -ne 0) {
            throw "Derived live_review failed with exit code $LASTEXITCODE"
        }
    }

    $Report = Get-Content -LiteralPath $BridgeReport -Raw | ConvertFrom-Json
    if ($Report.schema -ne 'piper_live_next_state_bridge_report_v1') {
        throw "Derived bridge report has an unexpected schema: $BridgeReport"
    }
    $ReportSource = [System.IO.Path]::GetFullPath([string]$Report.source.dataset)
    $ReportDerived = [System.IO.Path]::GetFullPath(
        [string]$Report.derived_dataset.path
    )
    if (-not [string]::Equals(
        $ReportSource,
        $SourceDataset,
        [System.StringComparison]::OrdinalIgnoreCase
    )) {
        throw "Derived report belongs to a different source dataset: $ReportSource"
    }
    if (-not [string]::Equals(
        $ReportDerived,
        $DerivedDataset,
        [System.StringComparison]::OrdinalIgnoreCase
    )) {
        throw "Derived report describes a different output dataset: $ReportDerived"
    }
    if (
        $Report.training_label.status -ne 'COMPLETE' -or
        $Report.training_label.valid_rows -ne $Report.training_label.total_rows -or
        $Report.training_label.final_source_row_action_written -ne $false -or
        $Report.derived_dataset.all_rows_action_equals_next_state -ne $true -or
        $Report.derived_dataset.source_unchanged -ne $true -or
        $Report.source.source_tree_sha256_before -ne
            $Report.source.source_tree_sha256_after
    ) {
        throw "Derived report does not satisfy the next-state training contract: $BridgeReport"
    }
    if (-not (Test-Path -LiteralPath $ReviewDiagnostics -PathType Leaf)) {
        throw "Existing review is missing diagnostics.json; use a new -ReviewDir: $ReviewRoot"
    }
    $ReviewReport = Get-Content -LiteralPath $ReviewDiagnostics -Raw | ConvertFrom-Json
    if (
        $ReviewReport.mode -ne 'derived_next_state' -or
        $ReviewReport.training_label_valid_rows -ne $Report.training_label.valid_rows -or
        $ReviewReport.training_label_invalid_rows -ne 0
    ) {
        throw "Existing review does not match the derived dataset: $ReviewRoot"
    }
    $ReviewDataset = [System.IO.Path]::GetFullPath([string]$ReviewReport.dataset)
    if (-not [string]::Equals(
        $ReviewDataset,
        $DerivedDataset,
        [System.StringComparison]::OrdinalIgnoreCase
    )) {
        throw "Existing review belongs to a different derived dataset: $ReviewDataset"
    }

    [pscustomobject]@{
        source_dataset = $SourceDataset
        derived_dataset = $DerivedDataset
        bridge_report = $BridgeReport
        review_root = $ReviewRoot
        source_frames = $Report.source.frames
        training_rows = $Report.training_label.valid_rows
        training_label_status = $Report.training_label.status
        command_audit = $Report.command_evidence.status
        formal_training_eligible = $Report.formal_training_eligible
        offline_training_allowed = $Report.offline_training_allowed
    } | Format-List
}
finally {
    Pop-Location
}
