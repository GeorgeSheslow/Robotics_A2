close all
clear
clc

robot = DobotMagician(transl(0,0,0)); % load robot model

%% Get Word trajectory points and use RMRC for drawing
clc
write = TextToTraj("Dobot","text");
drawPoints = write.GetTraj();
[x, qMatrix] = robot.trajGen.getQForTraj(drawPoints);

%% Plot calculated trajectory points from TextToTraj function
hold on;
for i = 1:size(x,2)
    plot3(drawPoints(1,i),drawPoints(2,i),drawPoints(3,i),'k.');
    hold on;
end

%% Plot dobot drawing using RMRC
dobot.trajGen.plotQAndTraj(qMatrix, x);

