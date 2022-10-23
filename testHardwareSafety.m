%% Check arduino serial port

serialportlist("available")'
%%
close all
clear
clc

gui = GUI();
HardwareSafety("/dev/tty.usbserial-144140",gui);

