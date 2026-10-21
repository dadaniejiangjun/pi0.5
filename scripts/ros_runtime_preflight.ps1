[CmdletBinding()]
param(
    [ValidateSet('p2_control', 'p3_p4_shadow', 'p4_shadow_only', 'p6_camera_record', 'p6_operator_demo')]
    [string]$Profile = 'p2_control',
    [string]$ProjectRoot = 'E:\01_NUS\02_5003\12_pi0.5',
    [string]$Python = 'E:\anaconda3\envs\pi05_laptop\python.exe'
)

$ErrorActionPreference = 'Stop'
$ProfileLoader = Join-Path $ProjectRoot 'laptop\ros_gateway\runtime_profiles.py'
if (-not (Test-Path -LiteralPath $Python -PathType Leaf)) {
    throw "Configured Python interpreter not found: $Python"
}
if (-not (Test-Path -LiteralPath $ProfileLoader -PathType Leaf)) {
    throw "ROS runtime profile loader not found: $ProfileLoader"
}

$ProfileArgs = @('-B', $ProfileLoader, '--profile', $Profile, '--format', 'json')
$ProfileText = & $Python @ProfileArgs
if ($LASTEXITCODE -ne 0) {
    throw "Could not load runtime profile '$Profile' (exit $LASTEXITCODE)."
}
$ProfileRecord = ($ProfileText -join "`n") | ConvertFrom-Json
$ProfileBody = $ProfileRecord.profile
$Distribution = $ProfileRecord.project.wsl_distribution
$Ports = @(
    $ProfileBody.gateway.control_port,
    $ProfileBody.gateway.base_image_port,
    $ProfileBody.gateway.wrist_image_port
) | Where-Object { $null -ne $_ -and [int]$_ -gt 0 } | Sort-Object -Unique

$NeedsWslRoute = ($Ports.Count -gt 0) -or (
    $ProfileBody.policy.host_resolution -eq 'wsl_default_route_gateway'
)
$GatewayHost = $null
$HostRecord = $null
$AddressResolutionStatus = 'NOT_REQUIRED'
if ($NeedsWslRoute) {
    $HostText = @(& $Python -B $ProfileLoader --profile $Profile --format json `
        --resolve-host --distribution $Distribution 2>&1)
    $HostExitCode = $LASTEXITCODE
    if ($HostExitCode -eq 0) {
        $HostRecord = ($HostText -join "`n") | ConvertFrom-Json
        $GatewayHost = if ($HostRecord.gateway_host) {
            $HostRecord.gateway_host
        } else {
            $HostRecord.policy_host
        }
        $AddressResolutionStatus = 'RESOLVED_FROM_WSL_DEFAULT_ROUTE'
    } else {
        $HostFailureText = (($HostText | ForEach-Object { [string]$_ }) -join "`n") -replace "`0", ''
        $HostFailureText = [regex]::Replace($HostFailureText, '[^\x20-\x7E]', '')
        $HostFailureCode = if ($HostFailureText -match '(Wsl/[A-Za-z0-9_]+)') { $Matches[1] } else { $null }
        if ($HostFailureCode -eq 'Wsl/E_ACCESSDENIED') { $HostFailureCode = 'Wsl/E_ACCESS_DENIED' }
        $AddressResolutionStatus = if ($HostFailureCode) {
            "UNKNOWN: $HostFailureCode"
        } else {
            "UNKNOWN: WSL route resolver exited $HostExitCode"
        }
    }
}
$PolicyHost = $null
if ($ProfileBody.policy) {
    if ($HostRecord -and $HostRecord.policy_host) {
        $PolicyHost = $HostRecord.policy_host
    } else {
        $PolicyHost = $ProfileBody.policy.host
    }
}

$Revision = $null
$Dirty = $null
try {
    $Revision = (& git -C $ProjectRoot rev-parse HEAD 2>$null | Select-Object -First 1)
    if ($LASTEXITCODE -eq 0) {
        $Dirty = [bool](@(& git -C $ProjectRoot status --short --ignore-submodules=all 2>$null).Count)
    }
} catch {
    $Revision = 'UNKNOWN'
    $Dirty = 'UNKNOWN'
}

$PortInventory = foreach ($Port in $Ports) {
    try {
        $Listeners = @(Get-NetTCPConnection -State Listen -LocalPort ([int]$Port) -ErrorAction SilentlyContinue)
        if ($Listeners.Count -eq 0) {
            [pscustomobject]@{ Port = [int]$Port; LocalAddress = $null; PID = $null; Process = 'NO_LISTENER'; Source = 'Get-NetTCPConnection' }
            continue
        }
        foreach ($Listener in $Listeners) {
            $ProcessName = 'UNKNOWN'
            $CommandLine = $null
            try {
                $Owner = Get-CimInstance Win32_Process -Filter "ProcessId = $($Listener.OwningProcess)" -ErrorAction Stop
                if ($Owner) {
                    $ProcessName = $Owner.Name
                    $CommandLine = $Owner.CommandLine
                }
            } catch {
                $ProcessName = 'PID_VISIBLE_COMMAND_LINE_UNAVAILABLE'
            }
            [pscustomobject]@{
                Port = [int]$Port
                LocalAddress = $Listener.LocalAddress
                PID = [int]$Listener.OwningProcess
                Process = $ProcessName
                CommandLine = $CommandLine
                Source = 'Get-NetTCPConnection'
            }
        }
    } catch {
        $FallbackListeners = @()
        try {
            $NetstatLines = @(& "$env:SystemRoot\System32\netstat.exe" -ano -p tcp 2>$null)
            foreach ($NetstatLine in $NetstatLines) {
                $Line = [string]$NetstatLine
                if ($Line -match '^\s*TCP\s+(\S+):([0-9]+)\s+\S+\s+LISTENING\s+([0-9]+)\s*$' -and
                    [int]$Matches[2] -eq [int]$Port) {
                    $FallbackListeners += [pscustomobject]@{
                        Port = [int]$Port
                        LocalAddress = $Matches[1]
                        PID = [int]$Matches[3]
                        Process = 'PID_VISIBLE_COMMAND_LINE_UNAVAILABLE'
                        CommandLine = $null
                        Source = 'netstat.exe -ano'
                    }
                }
            }
        } catch {
            $FallbackListeners = @()
        }
        if ($FallbackListeners.Count -gt 0) {
            $FallbackListeners
        } elseif ($null -ne $NetstatLines) {
            [pscustomobject]@{ Port = [int]$Port; LocalAddress = $null; PID = $null; Process = 'NO_LISTENER'; Source = 'netstat.exe -ano' }
        } else {
            [pscustomobject]@{ Port = [int]$Port; LocalAddress = $null; PID = $null; Process = 'UNKNOWN_LISTENER_INVENTORY'; Source = 'Get-NetTCPConnection_and_netstat_failed' }
        }
    }
}

$OwnerRegistryDirectory = if ($env:PI05_RUNTIME_OWNER_DIR) {
    $env:PI05_RUNTIME_OWNER_DIR
} else {
    Join-Path $ProjectRoot 'cache\ros_runtime\owners'
}
$OwnerRegistryByProcess = @{}
$OwnerRegistryReadStatus = 'DIRECTORY_NOT_FOUND'
if (Test-Path -LiteralPath $OwnerRegistryDirectory -PathType Container) {
    $OwnerRegistryReadStatus = 'READ_ONLY_QUERY_OK'
    foreach ($OwnerFile in Get-ChildItem -LiteralPath $OwnerRegistryDirectory -Filter '*.json' -File -ErrorAction SilentlyContinue) {
        try {
            $OwnerRecord = Get-Content -LiteralPath $OwnerFile.FullName -Raw | ConvertFrom-Json
            if ($OwnerRecord.schema -ne 'pi05_runtime_owner_v1' -or
                $null -eq $OwnerRecord.pid -or -not $OwnerRecord.role) {
                continue
            }
            $OwnerKey = ([string]$OwnerRecord.role) + ':' + ([string][int]$OwnerRecord.pid)
            $OwnerRegistryByProcess[$OwnerKey] = [pscustomobject]@{
                Role = [string]$OwnerRecord.role
                PID = [int]$OwnerRecord.pid
                Status = [string]$OwnerRecord.status
                SessionId = if ($OwnerRecord.session_id) { [string]$OwnerRecord.session_id } else { 'NO_ACTIVE_SESSION' }
                Profile = if ($OwnerRecord.profile) { [string]$OwnerRecord.profile } elseif ($OwnerRecord.deployment_profile) { [string]$OwnerRecord.deployment_profile } else { $null }
                ProfileSha256 = if ($OwnerRecord.profile_sha256) { [string]$OwnerRecord.profile_sha256 } elseif ($OwnerRecord.deployment_profile_sha256) { [string]$OwnerRecord.deployment_profile_sha256 } else { $null }
                UpdatedUtc = [string]$OwnerRecord.updated_utc
                Endpoint = if ($OwnerRecord.gateway_host) { "$($OwnerRecord.gateway_host):$($OwnerRecord.control_port)" } elseif ($OwnerRecord.host) { "$($OwnerRecord.host):$($OwnerRecord.control_port)" } else { $null }
                Workspace = [string]$OwnerRecord.workspace
                Overlay = [string]$OwnerRecord.overlay
                RecordPath = $OwnerFile.FullName
            }
        } catch {
            $OwnerRegistryReadStatus = 'READ_WITH_INVALID_RECORDS'
        }
    }
}

$WindowsCandidateProcesses = @()
$WindowsProcessInventoryStatus = 'READ_ONLY_QUERY_OK'
try {
    $WindowsCandidateProcesses = @(Get-CimInstance Win32_Process -ErrorAction Stop |
        Where-Object {
            $_.Name -match '^(Camera|WindowsCamera|obs64|Zoom|ms-teams|NVIDIA Broadcast|python|pythonw)\.exe$' -and
            ($_.Name -notmatch '^python' -or $_.CommandLine -match 'camera|capture|record.*bag|manual_teach|ros_gateway|windows_.*gateway|piper_session|piper_control|supervised_joint|bounded_model_trial')
        } |
        ForEach-Object {
            $ProcessId = [int]$_.ProcessId
            $Role = if ($_.CommandLine -match 'windows_(read_only|p2_deterministic)_gateway') {
                'windows_gateway'
            } elseif ($_.Name -match '^(Camera|WindowsCamera|obs64|Zoom|ms-teams|NVIDIA Broadcast)\.exe$' -or
                      $_.CommandLine -match 'camera|capture|record.*bag|manual_teach') {
                'camera_owner_candidate'
            } else {
                'possible_can_owner_candidate'
            }
            $Owner = $OwnerRegistryByProcess[$Role + ':' + ([string]$ProcessId)]
            [pscustomobject]@{
                PID = $ProcessId
                Name = $_.Name
                Role = $Role
                CommandLine = $_.CommandLine
                SessionId = if ($Owner) { $Owner.SessionId } else { 'UNKNOWN_NO_LIVE_SESSION_RECORD' }
                OwnerStatus = if ($Owner) { $Owner.Status } else { 'UNKNOWN' }
                OwnerProfile = if ($Owner) { $Owner.Profile } else { $null }
                OwnerEndpoint = if ($Owner) { $Owner.Endpoint } else { $null }
                OwnerRecord = if ($Owner) { $Owner.RecordPath } else { $null }
            }
        })
} catch {
    $WindowsProcessInventoryStatus = "UNKNOWN: $($_.Exception.Message)"
    $WindowsCandidateProcesses = @([pscustomobject]@{ Status = "UNKNOWN: $($_.Exception.Message)" })
}

# A listener PID is stronger evidence of a gateway owner than process-name
# inspection alone, and remains useful when CIM hides its command line.
$KnownGatewayPids = @(
    $WindowsCandidateProcesses |
        Where-Object { $_.Role -eq 'windows_gateway' -and $null -ne $_.PID } |
        ForEach-Object { [int]$_.PID }
)
foreach ($Listener in $PortInventory | Where-Object {
    $null -ne $_.PID -and [int]$_.Port -eq [int]$ProfileBody.gateway.control_port
}) {
    $ListenerPid = [int]$Listener.PID
    if ($KnownGatewayPids -contains $ListenerPid) { continue }
    $Owner = $OwnerRegistryByProcess['windows_gateway:' + ([string]$ListenerPid)]
    $WindowsCandidateProcesses += [pscustomobject]@{
        PID = $ListenerPid
        Name = 'UNKNOWN_LISTENER_OWNER'
        Role = 'windows_gateway'
        CommandLine = $null
        SessionId = if ($Owner) { $Owner.SessionId } else { 'UNKNOWN_NO_LIVE_SESSION_RECORD' }
        OwnerStatus = if ($Owner) { $Owner.Status } else { 'UNKNOWN' }
        OwnerProfile = if ($Owner) { $Owner.Profile } else { $null }
        OwnerEndpoint = if ($Owner) { $Owner.Endpoint } else { $null }
        OwnerRecord = if ($Owner) { $Owner.RecordPath } else { $null }
    }
    $KnownGatewayPids += $ListenerPid
}

$WslProcesses = @()
$WslInventoryStatus = 'NOT_QUERIED'
try {
    $WslLines = @(& wsl.exe -d $Distribution -- ps -eo pid=,args= 2>&1)
    if ($LASTEXITCODE -eq 0) {
        $WslInventoryStatus = 'READ_ONLY_QUERY_OK'
        $WslProcesses = @($WslLines | ForEach-Object {
            $Line = ([string]$_).Trim()
            if ($Line -match '^([0-9]+)\s+(.*)$' -and $Matches[2] -match 'read_only_bridge|policy_shadow|target_preview|run_task_action|ros2 launch') {
                $ProcessId = [int]$Matches[1]
                $CommandLine = $Matches[2]
                $Role = if ($CommandLine -match 'read_only_bridge') { 'ros_bridge' } else { 'ros_process' }
                $Owner = $OwnerRegistryByProcess[$Role + ':' + ([string]$ProcessId)]
                [pscustomobject]@{
                    PID = $ProcessId
                    Role = $Role
                    CommandLine = $CommandLine
                    SessionId = if ($Owner) { $Owner.SessionId } else { 'UNKNOWN_NO_LIVE_SESSION_RECORD' }
                    OwnerStatus = if ($Owner) { $Owner.Status } else { 'UNKNOWN' }
                    OwnerProfile = if ($Owner) { $Owner.Profile } else { $null }
                    OwnerEndpoint = if ($Owner) { $Owner.Endpoint } else { $null }
                    OwnerRecord = if ($Owner) { $Owner.RecordPath } else { $null }
                    Overlay = if ($Owner) { $Owner.Overlay } else { $null }
                }
            }
        } | Where-Object { $null -ne $_ })
    } else {
        $WslExitCode = $LASTEXITCODE
        $WslErrorText = (($WslLines | ForEach-Object { [string]$_ }) -join "`n") -replace "`0", ''
        $WslErrorText = [regex]::Replace($WslErrorText, '[^\x20-\x7E]', '')
        $WslErrorText = [regex]::Replace($WslErrorText, 'Wsl/E_?ACCESS_?DENIED', 'Wsl/E_ACCESS_DENIED', 'IgnoreCase')
        $WslErrorCode = if ($WslErrorText -match '(Wsl/[A-Za-z0-9_]+)') { $Matches[1] } else { $null }
        $WslInventoryStatus = if ($WslErrorCode) {
            "UNKNOWN: $WslErrorCode (exit $WslExitCode)"
        } else {
            "UNKNOWN: wsl ps exited $WslExitCode"
        }
        # Native command diagnostics are not process rows. Do not leak them into
        # WslRosProcesses, where they could be mistaken for detected ROS nodes.
        $WslProcesses = @()
    }
} catch {
    $WslInventoryStatus = "UNKNOWN: $($_.Exception.Message)"
}

$WindowsGatewayProcessCount = @($WindowsCandidateProcesses | Where-Object { $_.Role -eq 'windows_gateway' }).Count
$WindowsPossibleCanOwnerCount = @($WindowsCandidateProcesses | Where-Object { $_.Role -eq 'possible_can_owner_candidate' }).Count
$WindowsPossibleCameraOwnerCount = @($WindowsCandidateProcesses | Where-Object { $_.Role -eq 'camera_owner_candidate' }).Count
$WslBridgeProcessCount = if ($WslInventoryStatus -like 'UNKNOWN:*') {
    $null
} else {
    @($WslProcesses | Where-Object { $_.Role -eq 'ros_bridge' }).Count
}
$RuntimeOwnerAssociations = @(
    $WindowsCandidateProcesses | Where-Object { $_.Role -eq 'windows_gateway' } | ForEach-Object {
        $GatewayProcess = $_
        $GatewayListeners = @($PortInventory | Where-Object { $_.PID -eq $GatewayProcess.PID })
        $GatewayEndpoints = @($GatewayListeners | ForEach-Object { "$($_.LocalAddress):$($_.Port)" }) -join ', '
        [pscustomobject]@{
            Host = 'Windows'; Role = $GatewayProcess.Role; PID = $GatewayProcess.PID; Process = $GatewayProcess.Name
            SessionId = $GatewayProcess.SessionId; Status = $GatewayProcess.OwnerStatus; Profile = $GatewayProcess.OwnerProfile
            Endpoint = if ($GatewayProcess.OwnerEndpoint) { $GatewayProcess.OwnerEndpoint } else { $GatewayEndpoints }
            OwnerRecord = $GatewayProcess.OwnerRecord
        }
    }
    $WslProcesses | Where-Object { $_.Role -eq 'ros_bridge' } | ForEach-Object {
        [pscustomobject]@{
            Host = 'WSL'; Role = $_.Role; PID = $_.PID; Process = $_.CommandLine
            SessionId = $_.SessionId; Status = $_.OwnerStatus; Profile = $_.OwnerProfile
            Endpoint = if ($_.OwnerEndpoint) { $_.OwnerEndpoint } else { $_.CommandLine }
            OwnerRecord = $_.OwnerRecord; Overlay = $_.Overlay
        }
    }
)

$Result = [pscustomobject]@{
    Schema = 'pi05_ros_runtime_preflight_v1'
    ReadOnly = $true
    Profile = $ProfileRecord.profile_name
    ProfileSha256 = $ProfileRecord.profile_sha256
    RosDomainId = $ProfileBody.ros_domain_id
    WindowsAddressFromWslDefaultRoute = $GatewayHost
    WindowsAddressResolutionStatus = $AddressResolutionStatus
    ExpectedGateway = [pscustomobject]@{
        control_port = $ProfileBody.gateway.control_port
        base_image_port = $ProfileBody.gateway.base_image_port
        wrist_image_port = $ProfileBody.gateway.wrist_image_port
    }
    ExpectedPolicyEndpoint = if ($ProfileBody.policy) {
        [pscustomobject]@{ host = $PolicyHost; port = $ProfileBody.policy.port }
    } else { $null }
    ExpectedPolicyIdentity = $ProfileRecord.expected_policy_identity
    PolicyIdentityLockWslPath = $ProfileRecord.policy_identity_lock_wsl_path
    DefaultRosWorkspace = $ProfileRecord.project.default_workspace
    ActiveOverlaySource = 'reported per live WSL bridge owner record; not inferred when no bridge is running'
    ExpectedWindowsPorts = @($Ports)
    ProjectRoot = (Resolve-Path -LiteralPath $ProjectRoot).Path
    GitRevision = $Revision
    WorkingTreeDirty = $Dirty
    WindowsPortListeners = @($PortInventory)
    RuntimeOwnerRegistryDirectory = $OwnerRegistryDirectory
    RuntimeOwnerRegistryReadStatus = $OwnerRegistryReadStatus
    RuntimeOwnerAssociations = @($RuntimeOwnerAssociations)
    WindowsPossibleCameraOwners = @($WindowsCandidateProcesses)
    WindowsGatewayProcessCount = $WindowsGatewayProcessCount
    MultipleWindowsGatewaysDetected = ($WindowsGatewayProcessCount -gt 1)
    WindowsPossibleCameraOwnerProcessCount = if ($WindowsProcessInventoryStatus -like 'UNKNOWN:*') { $null } else { $WindowsPossibleCameraOwnerCount }
    WindowsProcessInventoryStatus = $WindowsProcessInventoryStatus
    WindowsPossibleCanOwnerProcessCount = if ($WindowsProcessInventoryStatus -like 'UNKNOWN:*') { $null } else { $WindowsPossibleCanOwnerCount }
    WslProcessInventoryStatus = $WslInventoryStatus
    WslRosProcesses = @($WslProcesses)
    WslBridgeProcessCount = $WslBridgeProcessCount
    MultipleWslBridgesDetected = if ($null -eq $WslBridgeProcessCount) { $null } else { $WslBridgeProcessCount -gt 1 }
    ActionsTaken = @('profile read', 'local process and listener inventory', 'WSL process inventory')
    ActionsNotTaken = @('no CAN or USB access', 'no TCP connection attempt', 'no process termination', 'no camera open', 'no ROS launch', 'no robot command')
    Note = 'Owner rows associate live PIDs with process-keyed session records when available. Candidate camera processes are not proof of an open camera handle; missing session records are reported as unknown, not inferred.'
}
$Result | ConvertTo-Json -Depth 8
