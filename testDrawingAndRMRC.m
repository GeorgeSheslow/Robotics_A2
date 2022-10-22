%% Dobot RMRC Trajectory
clear all
close all
clc

dobot = DobotMagician(transl(0,0,0));
axis equal;

write = TextToTraj("DOBOT");
x_Write = write.GetTraj();
[x, qMatrix] = dobot.trajGen.getQForLineTraj(transl(x_Write(:,1)));
dobot.trajGen.animateQ(qMatrix)
[x, qMatrix] = dobot.trajGen.getQForTraj(x_Write);
dobot.trajGen.plotQAndTraj(qMatrix, x)
[x, qMatrix] = dobot.trajGen.getQForLineTraj(transl(0.3,0,0.25));
dobot.trajGen.animateQ(qMatrix)

%% Dobot Drawing
clear all
close all
clc

dobot = DobotMagician(transl(0,0,1));
axis equal;
hold on;
write = TextToTraj("DOBOT"); % create instance of TextToTraj class
write.addBaseOffsets(dobot.model.base(1:3,4));
x_Write = write.GetTraj()
write.PlotTraj();
%%
x_Write = write.GetTraj(); % Get points for text
[x, qMatrix] = dobot.trajGen.getQForLineTraj(transl(x_Write(:,1))); % Use RMRC line traj to get to paper level
dobot.trajGen.animateQ(qMatrix) % Animate
[x, qMatrix] = dobot.trajGen.getQForTraj(x_Write); % Use RMRC to write text
drawText(dobot,write.getDrawingHeight(),x, qMatrix,1); % animate
[x, qMatrix] = dobot.trajGen.getQForLineTraj(transl(0.3,0,0.25)); % Move EE to neutal pose
dobot.trajGen.animateQ(qMatrix)
%% IRB RMRC
clear all
close all
clc
IRB = IRB120(transl(0,0,0));
waypoint1 = [0 0.4 0];
middle = [0.4 0 0];
waypoint2 = [0 -0.4 0];
waitpoint = [0.3,0,0.6];
IRB.trajGen.steps = 20;
[x, traj] = IRB.trajGen.getQForLineTraj(transl(waitpoint));
IRB.trajGen.plotQAndTraj(traj, x)
[x, traj] = IRB.trajGen.getQForLineTraj(transl(waypoint1));
IRB.trajGen.plotQAndTraj(traj, x)
[x, traj] = IRB.trajGen.getQForZArcTraj(transl(middle));
IRB.trajGen.plotQAndTraj(traj, x)
[x, traj] = IRB.trajGen.getQForZArcTraj(transl(waypoint2));
IRB.trajGen.plotQAndTraj(traj, x)
[x, traj] = IRB.trajGen.getQForLineTraj(transl(waitpoint));
IRB.trajGen.plotQAndTraj(traj, x)

%% Functions

function drawText(robot,paperHeight,x, qMatrix, desiredTrajOn)
    for j = 1:size(qMatrix,1)
        newQ = qMatrix(j,:);
        robot.model.animate(newQ);
        drawnow();
        hold on
        pos = robot.model.fkine(robot.model.getpos());
        if pos(3,4) <= paperHeight + 0.005
            plot3(pos(1,4)+robot.toolOffset(1),pos(2,4)+robot.toolOffset(2),pos(3,4)+robot.toolOffset(3),'r.');
            if desiredTrajOn
                plot3(x(1,4)+robot.toolOffset(1),x(2,4)+robot.toolOffset(2),x(3,4)+robot.toolOffset(3),'k.');
            end
        end
        pause(0.1);
    end 
end