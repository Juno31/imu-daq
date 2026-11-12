# Creates a desktop shortcut "IMU DAQ" that runs start.bat with the app icon.
$root = $PSScriptRoot
$desk = [Environment]::GetFolderPath("Desktop")
$s = (New-Object -ComObject WScript.Shell).CreateShortcut((Join-Path $desk "IMU DAQ.lnk"))
$s.TargetPath = Join-Path $root "start.bat"
$s.WorkingDirectory = $root
$s.IconLocation = (Join-Path $root "assets\imu_daq.ico") + ",0"
$s.Description = "Start the IMU DAQ server and open the GUI"
$s.Save()
Write-Host "Shortcut created on the desktop: IMU DAQ"
