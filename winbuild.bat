setlocal enableextensions

rem ensure python 3.5 and cmake is on the system path, otherwise set them here
rem Python 3.5 also needs the "numpy" package, install via the command "pip install numpy --user"
rem ensure that the MORSE_BLENDER environment var is set to the blender.exe path+file
rem Also note that Blender and Python must both be 32bit or both 64bit installs. They can't be different.
rem you may also need a c compiler: "Microsoft Build Tools for Visual Studio 2017"

rem get python paths
for /f "delims=" %%i in ('python -c "from sysconfig import get_paths; print(get_paths()['include'])"') do set PYINC=%%i
for /f "delims=" %%A in ('where python') do set PYTHONPATH=%%~dpA

rem 32 or 64bit Python?
for /f %%i in ('python -c "import struct; bit=8*struct.calcsize('P'); print('-DCMAKE_GENERATOR_PLATFORM=x64' if bit==64 else '')"') do set ISBIT=%%i

rem only make the build folder if it doesn't exist
if not exist "build" mkdir build

rem and build and install into C:\morse
rem if you want a different folder for more to be installed into, change the -DCMAKE_INSTALL_PREFIX="" below

cd build
cmake .. -DPYTHON_INCLUDE_DIR="%PYINC%" %ISBIT% -DPYTHON_LIBRARY="%PYTHONPATH%libs\python35.lib" -DCMAKE_INSTALL_PREFIX="C:\morse"
cmake --build . --config Release --target install
pause
