###############################################################################
# RunTasks.ps1
#
# This script performs the following:
# 1. Activates the virtual environment at:
#    C:\Users\LocalUser\Desktop\SWARM_Repo\Gautam\airsimvenv\Scripts\activate
#    (if not already activated)
#
# 2. Prompts the user whether to run:
#    C:\Users\LocalUser\Desktop\SWARM_Repo\Gautam\SWARMEnv\AirSim\AirSim\AirsimExe.exe
#
# 3. Lists the files in the "Environments" directory and lets the user choose one
#    to run (or skip by entering "n").
#
# 4. Lists the files in the "Traversal" directory and lets the user pick a file to run.
#
# 5. Continues to prompt the user if they wish to run additional files in the
#    "Traversal" directory until they choose to exit (by entering "n").
###############################################################################

#-----------------------------
# Step 1: Activate virtual environment
#-----------------------------
# Check if the virtual environment is already active by testing the VIRTUAL_ENV variable.
if (-not $env:VIRTUAL_ENV) {
    Write-Host "Activating virtual environment..."
    # Dot-source the activation script so that its environment changes affect the current session.
    . "C:\Users\LocalUser\Desktop\SWARM_Repo\Gautam\airsimvenv\Scripts\activate"
} else {
    Write-Host "Virtual environment already activated."
}

#-----------------------------
# Step 2: Optionally run AirSimExe.exe
#-----------------------------
$airSimExePath = "C:\Users\LocalUser\Desktop\SWARM_Repo\Gautam\SWARMEnv\AirSim\AirSim\AirsimExe.exe"
if (Test-Path $airSimExePath) {
    $userInput = Read-Host "Do you want to run AirSimExe.exe? (Press Enter to run or type 'n' to skip)"
    if ($userInput -ne "n") {
        Write-Host "Starting AirSimExe.exe..."
        Start-Process $airSimExePath
    } else {
        Write-Host "Skipping AirSimExe.exe."
    }
} else {
    Write-Host "AirSimExe.exe not found at $airSimExePath"
}

#-----------------------------
# Step 3: Pick a file from the Environments directory
#-----------------------------
$environmentsDir = "C:\Users\LocalUser\Desktop\SWARM_Repo\Gautam\AirSimRepo\Environments"
if (Test-Path $environmentsDir) {
    $envFiles = Get-ChildItem -Path $environmentsDir -File
    if ($envFiles.Count -gt 0) {
        Write-Host "Files in the Environments directory:"
        for ($i = 0; $i -lt $envFiles.Count; $i++) {
            Write-Host "  [$i] $($envFiles[$i].Name)"
        }
        $choice = Read-Host "Enter the index of the file to run, or type 'n' to skip"
        if ($choice -ne "n") {
            if ($choice -match '^\d+$' -and [int]$choice -ge 0 -and [int]$choice -lt $envFiles.Count) {
                $selectedEnvFile = $envFiles[[int]$choice].FullName
                Write-Host "Running file: $selectedEnvFile"
                & $selectedEnvFile
            }
            else {
                Write-Host "Invalid selection. Skipping execution in Environments."
            }
        }
        else {
            Write-Host "Skipping file execution in Environments."
        }
    }
    else {
        Write-Host "No files found in the Environments directory."
    }
}
else {
    Write-Host "Environments directory not found: $environmentsDir"
}

#-----------------------------
# Steps 4 & 5: Pick files from the Traversal directory in a loop
#-----------------------------
$traversalDir = "C:\Users\LocalUser\Desktop\SWARM_Repo\Gautam\AirSimRepo\Traversal"
if (Test-Path $traversalDir) {
    while ($true) {
        $traversalFiles = Get-ChildItem -Path $traversalDir -File
        if ($traversalFiles.Count -eq 0) {
            Write-Host "No files found in the Traversal directory."
            break
        }
        
        Write-Host "`nFiles in the Traversal directory:"
        for ($i = 0; $i -lt $traversalFiles.Count; $i++) {
            Write-Host "  [$i] $($traversalFiles[$i].Name)"
        }
        
        $choiceTraversal = Read-Host "Enter the index of the file to run, or type 'n' to exit the Traversal loop"
        if ($choiceTraversal -eq "n") {
            Write-Host "Exiting the Traversal file execution loop."
            break
        }
        
        if ($choiceTraversal -match '^\d+$' -and [int]$choiceTraversal -ge 0 -and [int]$choiceTraversal -lt $traversalFiles.Count) {
            $selectedTraversalFile = $traversalFiles[[int]$choiceTraversal].FullName
            Write-Host "Running file: $selectedTraversalFile"
            & $selectedTraversalFile
        }
        else {
            Write-Host "Invalid selection. Please try again."
            continue
        }
        
        # Ask if the user wants to run another file from the Traversal directory.
        $runAnother = Read-Host "Do you want to run another file from the Traversal directory? (y/n)"
        if ($runAnother -eq "n") {
            Write-Host "Exiting the Traversal file execution loop."
            break
        }
    }
}
else {
    Write-Host "Traversal directory not found: $traversalDir"
}
