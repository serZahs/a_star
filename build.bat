@echo off

set CompilerFlags=-nologo -MT -FC -Z7 -EHsc -W4 -D_CRT_SECURE_NO_WARNINGS /I"..\lib\raylib-5.0_win64_msvc16\include" /I"..\include"
set LinkerFlags=raylib.lib user32.lib gdi32.lib shell32.lib winmm.lib msvcrt.lib /NODEFAULTLIB:libcmt /LIBPATH:"..\lib\raylib-5.0_win64_msvc16\lib" /LIBPATH:"..\lib\glfw-3.3.5.bin.WIN64\lib-vc2022"
IF NOT EXIST build mkdir build
pushd build
cl %CompilerFlags% ..\main.cpp /link %LinkerFlags%
popd
