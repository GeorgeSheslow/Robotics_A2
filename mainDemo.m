%% Check arduino serial port

serialportlist("available")'

%% Main Demo
close all
clear
clc

gui = GUI();
HardwareSafety("/dev/tty.usbserial-143140",gui);

%% Visual Servoing Demo
close all
clear
clc

vs = VSGUI();