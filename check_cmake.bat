@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%~dp0

set cmake_minversion_minmaj="   3.   21"

set "cmake_version=    .    "

where /q cmake
if %ERRORLEVEL% EQU 0 (
	for /F "tokens=3" %%a in ('cmake --version ^| find "cmake version"') do set cmake_version=%%a
	if "%cmake_version%" == "" (
	  echo Unable to get version of cmake. >&2
	  exit /b 2
	)
) else (
  echo cmake was not found in path.
  goto :download_install
)

set cmake_ver_major=
set cmake_ver_minor=
for /F "tokens=1,2 delims=." %%a in ("%cmake_version%") do (
  set "cmake_ver_major=    %%a"
  set "cmake_ver_minor=    %%b"
)
set cmake_ver_minmaj="%cmake_ver_major:~-4%.%cmake_ver_minor:~-4%"
if %cmake_ver_minmaj% LSS %cmake_minversion_minmaj% (
  echo(
  echo Newer AirSim requires cmake verion %cmake_minversion_minmaj% but you have %cmake_ver_minmaj% which is older.  >&2
  goto :download_install
)

echo Found cmake version: %cmake_version%
exit /b 0

:download_install
set /p choice="Press any key to download and install cmake (make sure to add it in path in install options)"

IF NOT EXIST %temp%\cmake-3.29.0-windows-x86_64.msi (
	@echo on
	powershell -command "& { iwr https://github.com/Kitware/CMake/releases/download/v3.29.0/cmake-3.29.0-windows-x86_64.msi -OutFile %temp%\cmake-3.29.0-windows-x86_64.msi }"
	@echo off
)

msiexec.exe /i "%temp%\cmake-3.29.0-windows-x86_64.msi"

exit /b 1