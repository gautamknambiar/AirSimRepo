###############################################################################
# run_airsim.ps1
#
# This script performs the following:
#
# 1. Checks if the Python virtual environment is activated.
#    If not, it instructs the user how to activate it and then exits.
#
# 2. Optionally runs AirSimExe.exe.
#
# 3. Lists Python files in the "Environments" directory and allows the user
#    to select one to run with: "python <selected file>".
#
# 4. Lists Python files in the "Traversal" directory and allows the user
#    to select files to run with: "python <selected file>", repeating until
#    the user decides to exit.
###############################################################################

#-----------------------------
# Step 1: Check if the virtual environment is activated
#-----------------------------
if (-not $env:VIRTUAL_ENV) {
    Write-Host "The Python virtual environment is not activated."
    Write-Host ""
    Write-Host "Please activate it by running the following command in your PowerShell session:"
    Write-Host ""
    Write-Host "../airsimvenv/Scripts/activate"
    Write-Host ""
    Write-Host "After activating the virtual environment, please re-run this script."
    exit
}

Write-Host "Python virtual environment is activated."
Write-Host ""

#-----------------------------
# Step 2: Optionally run AirSimExe.exe
#-----------------------------
$airSimExePath = "C:\Users\sidew\SWARM\AirSim\AirSim\AirsimExe.exe"
if (Test-Path $airSimExePath) {
    $userInput = Read-Host "Do you want to run AirSimExe.exe? (Press Enter to run or type 'n' to skip)"
    if ($userInput -ne "n") {
        Write-Host "Starting AirSimExe.exe..."
        Start-Process -FilePath $airSimExePath -ArgumentList "-windowed"
    }
    else {
        Write-Host "Skipping AirSimExe.exe."
    }
}
else {
    Write-Host "AirSimExe.exe not found at $airSimExePath"
}

Write-Host ""

#-----------------------------
# Step 3: Pick a Python file from the Environments directory to run
#-----------------------------
$environmentsDir = "C:\Users\sidew\SWARM\AirSimRepo\Environments"
if (Test-Path $environmentsDir) {
    # Get only Python files (i.e. with a .py extension)
    $envFiles = Get-ChildItem -Path $environmentsDir -File | Where-Object { $_.Extension -eq ".py" }
    if ($envFiles.Count -gt 0) {
        Write-Host "Python files in the Environments directory:"
        for ($i = 0; $i -lt $envFiles.Count; $i++) {
            Write-Host "  [$i] $($envFiles[$i].Name)"
        }
        $choice = Read-Host "Enter the index of the file to run with Python, or type 'n' to skip"
        if ($choice -ne "n") {
            if ($choice -match '^\d+$' -and [int]$choice -ge 0 -and [int]$choice -lt $envFiles.Count) {
                $selectedEnvFile = $envFiles[[int]$choice].FullName
                Write-Host "Running: python $selectedEnvFile"
                python $selectedEnvFile
            }
            else {
                Write-Host "Invalid selection. Skipping execution in the Environments directory."
            }
        }
        else {
            Write-Host "Skipping file execution in the Environments directory."
        }
    }
    else {
        Write-Host "No Python files found in the Environments directory."
    }
}
else {
    Write-Host "Environments directory not found: $environmentsDir"
}

Write-Host ""

#-----------------------------
# Step 4: Pick Python files from the Traversal directory in a loop
#-----------------------------
$traversalDir = "C:\Users\sidew\SWARM\AirSimRepo\Traversal"
if (Test-Path $traversalDir) {
    while ($true) {
        # Get only Python files (i.e. with a .py extension)
        $traversalFiles = Get-ChildItem -Path $traversalDir -File | Where-Object { $_.Extension -eq ".py" }
        if ($traversalFiles.Count -eq 0) {
            Write-Host "No Python files found in the Traversal directory."
            break
        }
        
        Write-Host "`nPython files in the Traversal directory:"
        for ($i = 0; $i -lt $traversalFiles.Count; $i++) {
            Write-Host "  [$i] $($traversalFiles[$i].Name)"
        }
        
        $choiceTraversal = Read-Host "Enter the index of the file to run with Python, or type 'n' to exit the Traversal loop"
        if ($choiceTraversal -eq "n") {
            Write-Host "Exiting the Traversal file execution loop."
            break
        }
        
        if ($choiceTraversal -match '^\d+$' -and [int]$choiceTraversal -ge 0 -and [int]$choiceTraversal -lt $traversalFiles.Count) {
            $selectedTraversalFile = $traversalFiles[[int]$choiceTraversal].FullName
            Write-Host "Running: python $selectedTraversalFile"
            python $selectedTraversalFile
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
