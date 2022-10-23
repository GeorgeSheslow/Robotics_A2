close all
clear
clc

rob1 = DobotMagician(transl(1,0,0)*trotz(pi));

rob2 = IRB120(transl(0,0,0));
q0 = [0;pi/4;-pi/5;0;pi/2;pi];
rob2.model.animate(q0');
% rob1.model.animate([0,pi/4,pi/4,pi/2]);

servo = visualServo(rob1, rob2);
servo.init()
%%
servo.vs();
