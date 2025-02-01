# Set paths
$airsimExePath      = "C:\Users\LocalUser\Desktop\SWARM_Repo\Gautam\SWARMEnv\AirSim\AirSim\AirsimExe.exe"
$environmentsPath   = "C:\Users\LocalUser\Desktop\SWARM_Repo\Gautam\AirSimRepo\Environments"
$traversalPath      = "C:\Users\LocalUser\Desktop\SWARM_Repo\Gautam\AirSimRepo\Traversal"
$airsimProcessName  = "AirsimExe"
$pythonVenvPath     = "C:\Users\LocalUser\Desktop\SWARM_Repo\Gautam\airsimvenv\Scripts\activate"

# Function to check if a process is running
function Is-ProcessRunning($processName) {
    return Get-Process -Name $processName -ErrorAction SilentlyContinue
}

# Function to let the user select a file from a directory
function Select-File($path, $description) {
    $files = Get-ChildItem -Path $path -File | Select-Object -ExpandProperty FullName
    if ($files.Count -eq 0) {
        Write-Output "No files found in $path"
        return $null
    }

    # Use the backtick for a newline
    Write-Output "`nSelect a $description file to run (or enter 0 to skip):"
    for ($i = 0; $i -lt $files.Count; $i++) {
        Write-Output "$($i+1): $(Split-Path -Leaf $files[$i])"
    }

    $selection = Read-Host "Enter the number of the file you want to run"
    if ($selection -eq "0") {
        Write-Output "Skipping $description step."
        return $null
    } elseif ($selection -match '^\d+$' -and [int]$selection -ge 1 -and [int]$selection -le $files.Count) {
        return $files[[int]$selection - 1]
    } else {
        Write-Output "Invalid selection. Skipping..."
        return $null
    }
}

# Step 0: Activate Python Virtual Environment (Only if Not Already Active)
if (-not $env:VIRTUAL_ENV) {
    Write-Output "Activating Python virtual environment..."
    # Using escaped quotes for the call command in case of spaces in the path
    cmd.exe /c "call `"$pythonVenvPath`""
} else {
    Write-Output "Virtual environment is already active."
}

# Step 1: Start AirSim (if not already running)
if (Is-ProcessRunning $airsimProcessName) {
    $startAirsim = Read-Host "AirSim is already running. Do you want to restart it? (y/n)"
    if ($startAirsim -eq "y") {
        Write-Output "Restarting AirSim..."
        Stop-Process -Name $airsimProcessName -Force
        Start-Process -NoNewWindow -FilePath $airsimExePath -ArgumentList "-windowed"
    } else {
        Write-Output "Skipping AirSim startup."
    }
} else {
    $startAirsim = Read-Host "Do you want to start AirSim? (y/n)"
    if ($startAirsim -eq "y") {
        Write-Output "Starting AirSim..."
        Start-Process -NoNewWindow -FilePath $airsimExePath -ArgumentList "-windowed"
    } else {
        Write-Output "Skipping AirSim startup."
    }
}

# Step 2: Select and run an Environment file (or skip)
$envFile = Select-File $environmentsPath "Environment"
if ($envFile) {
    Write-Output "Running $envFile..."
    # Pass the file path without extra quotes
    Start-Process -NoNewWindow -FilePath "python" -ArgumentList $envFile
}

# Step 3: Select and run a Traversal file (allows multiple runs)
do {
    $traversalFile = Select-File $traversalPath "Traversal"
    if ($traversalFile) {
        Write-Output "Running $traversalFile..."
        Start-Process -NoNewWindow -FilePath "python" -ArgumentList $traversalFile
    }
    $runAnother = Read-Host "Do you want to run another traversal? (y/n)"
} while ($runAnother -eq "y")

Write-Output "Script execution complete."
