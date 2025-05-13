@echo off
setlocal ENABLEEXTENSIONS
set KEY_NAME=HKLM\SOFTWARE\Wow6432Node\Keil\Products\MDK
set KEY_NAME_XP=HKLM\SOFTWARE\Keil\Products\MDK
set VALUE_NAME=Path

:find Keil install parh
FOR /F "tokens=2*" %%A IN ('REG.exe query "%KEY_NAME%" /v "%VALUE_NAME%"') DO (set pInstallDir=%%B)

IF "%pInstallDir%" == "" (
	FOR /F "tokens=2*" %%A IN ('REG.exe query "%KEY_NAME_XP%" /v "%VALUE_NAME%"') DO (set pInstallDir=%%B)
)

IF "%pInstallDir%" == "" GOTO Error

:start strip
echo Current Path %~dp0
echo Keil MDK Path %pInstallDir%
set KEIL_STRIP=%pInstallDir%\ARMCC\bin\fromelf.exe

if exist %KEIL_STRIP%  (
    echo Keil Compiler 5
) else (
    echo Keil Compiler 6
	set KEIL_STRIP=%pInstallDir%\ARMCLANG\bin\fromelf.exe
)

echo Start strip libraries in %KEIL_STRIP% -----

Set SearchFolder=%~dp0
dir "%SearchFolder%" /s /b /a:-D | find ".lib"  | find /V ".lib\" > LIB.list
FOR /F "tokens=*" %%i IN (LIB.list) DO (
@echo STRIP %%i 
copy "%%i" "%%i.bak"
%KEIL_STRIP% --strip=debug,symbols --elf "%%i" --output "%%i"
)
DEL /q LIB.list

exit

:Error
msg %SESSIONNAME% Failed to strip library 
	
