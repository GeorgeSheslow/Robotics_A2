%% Check arduino serial port

serialportlist("available")'
%%
close all
clear
clc

gui = GUI();
HardwareSafety("/dev/tty.usbserial-143140",gui);

