@echo off
setlocal ENABLEDELAYEDEXPANSION

set "filename = gpiow_scene.blend"
set "pyScript=gpiow_import_ply.py"
set "compDiv=cpu"

"C:\Users\...\Documents\blender-cyc-2.93.4-windows-x64\blender.exe" -b "C:\Users\...\Documents\gpiow_scene.blend" -o //frame -F PNG -x 1 -y -P %~dp0!pyScript! -- !compDiv! >> "blender_render_gpiow.log" 2>&1

exit /b
