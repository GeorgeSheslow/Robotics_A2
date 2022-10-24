close all
clear
clc

rob1 = DobotMagician();
rob1.model.base = transl(1,0,0)*trotz(pi);
rob2 = IRB120();
q0 = [0;-pi/2;0;0;pi;0];
rob1.model.animate([0,pi/4,pi/4,pi/2]);

visualServo(rob1, rob2)

%%

close all
clear
clc

vs = VSGUI();