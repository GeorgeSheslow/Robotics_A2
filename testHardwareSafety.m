close all
clear
clc

%     serialportlist("available")'
gui = GUI();
HardwareSafety("/dev/cu.usbmodem1431401",gui);
