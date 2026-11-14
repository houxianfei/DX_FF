@echo off
setlocal
set "SDK_ROOT=%~dp0.."
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 exit /b 1
cl /nologo /EHsc /MD /std:c++17 /utf-8 /I"%SDK_ROOT%\include" "%~dp0main.cpp" /link /LIBPATH:"%SDK_ROOT%\lib\x64\Release" ff_application.lib /OUT:"%~dp0example.exe"
if errorlevel 1 exit /b 1
copy /Y "%SDK_ROOT%\bin\x64\Release\ff_application.dll" "%~dp0" >nul
copy /Y "%SDK_ROOT%\bin\x64\Release\CppDllMightyZap_Ver2010.dll" "%~dp0" >nul
