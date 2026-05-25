@ECHO OFF 
:: This is a batch file to run Docker container and setup necessary variables.
TITLE Run Cascar container

ECHO Please wait... Setting up environment
:: Section 1: Setup
ECHO ==========================
ECHO NETWORK INFO
ECHO ============================
ipconfig | findstr IPv4

SET ip_address_string="IPv4 Address"
for /f "usebackq tokens=2 delims=:" %%f in (`ipconfig ^| findstr /c:%ip_address_string%`) do (
    ECHO Using IP Address: %%f to forward Docker GUI
    SET DispIP=%%f:0.0
    GOTO :end_loop
)

:end_loop
    set DispIP=%DispIP:~1% 


:: Section 2: Docker
ECHO ==========================
ECHO RUN DOCKER CONTAINER
ECHO ============================
ECHO docker run -ti --rm -e DISPLAY=%DispIP% --net=host --name ros-env -v %cd%/src:/cascar_ws/src cascar
:: Command line version
docker run -ti --rm -e DISPLAY=%DispIP% --net=host --name ros-env -v %cd%/src:/cascar_ws/src cascar


:: Powershell Version
:: docker run -ti --rm -e DISPLAY=%DispIP% --net=host --name ros-env -v ${PWD}/src:/cascar_ws/src cascar