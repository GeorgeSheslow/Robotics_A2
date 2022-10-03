%% Code for Workspace Calculations
clear
clc
close all

pointCloud = FrontViewWorkspaceViz();
% pointCloud = FullViewWorkspaceViz();

% Create a 3D model showing where the end effector can be over all these samples
plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');

% Radius Calculations
xR = (max(pointCloud(:,1))-min(pointCloud(:,1)))/2;
yR = (max(pointCloud(:,2))-min(pointCloud(:,2)))/2;
zR = (max(pointCloud(:,3)))-min(pointCloud(:,3))/2;

display(['Robot radius: x = ',num2str(xR),' y = ',num2str(yR),' z = ',num2str(zR)]);

%% Functions for Calculations

function [pointCloud] = FullWorkspaceViz()
    robot = DobotMagician();
    hold on;
    qlim = robot.model.qlim;
    angle_resolution = 5;
    stepRads = deg2rad(angle_resolution);
    pointCloudeSize = prod(floor((qlim(1:4,2)-qlim(1:4,1))/deg2rad(angle_resolution) + 1));
    counter = 1;
    tic
    for q1 = qlim(1,1):stepRads:qlim(1,2)
        for q2 = qlim(2,1):stepRads:qlim(2,2)
            for q3 = qlim(3,1):stepRads:qlim(3,2)
                for q4 = qlim(4,1):stepRads:qlim(4,2)
                            q = [q1,q2,q3,q4];
                            tr = robot.model.fkine(q);                        
                            pointCloud(counter,:) = tr(1:3,4)';
                            counter = counter + 1; 
                            if mod(counter/pointCloudeSize * 100,1) == 0
                                display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                    end
                end
            end
        end
    end
end

function [pointCloud] = FrontViewWorkspaceViz()
    robot = DobotMagician();
    hold on;
    qlim = robot.model.qlim;
    angle_resolution = 5;
    stepRads = deg2rad(angle_resolution);
    pointCloudeSize = prod(floor((qlim(1:4,2)-qlim(1:4,1))/deg2rad(angle_resolution) + 1));
    counter = 1;
    q1 = 0;
    tic
%     for q1 = qlim(1,1):stepRads:qlim(1,2)
        for q2 = qlim(2,1):stepRads:qlim(2,2)
            for q3 = qlim(3,1):stepRads:qlim(3,2)
                for q4 = qlim(4,1):stepRads:qlim(4,2)
                            q = [q1,q2,q3,q4];
                            tr = robot.model.fkine(q);                        
                            pointCloud(counter,:) = tr(1:3,4)';
                            counter = counter + 1; 
                            if mod(counter/pointCloudeSize * 100,1) == 0
                                display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                    end
                end
            end
        end
%     end
end