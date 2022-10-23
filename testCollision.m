close all
clear
clc
set(figure,'units','normalized','outerpos',[0 0 1 1.2]);

%% IRB Collision checking with cube

robot = IRB120(transl(0,0,0));

q0 = robot.model.getpos();
jointTransforms = {};
modelPoints = {robot.model.base};
for i=1:6
    jointTransforms{i} = robot.model.A(i, q0);
    modelPoints{i+1} = cell2mat(modelPoints(i))*cell2mat(jointTransforms(i));
end

% centerPoints = [(transl(cell2mat(modelPoints(1))*(transl(0,0.0935,0))))';
%                 (transl(cell2mat(modelPoints(2))*(transl(0,0,0.0515))))';
%                 (transl(cell2mat(modelPoints(3))*(transl(0,0,0))))';
%                 (transl(cell2mat(modelPoints(4))*(transl(0,0,0))))';
%                 (transl(cell2mat(modelPoints(5))*(transl(0,0,0))))';
%                 (transl(cell2mat(modelPoints(6))*(transl(0,0,0))))';
%                 (transl(cell2mat(modelPoints(7))*(transl(0,0,0))))'];
centerPoints = [0.0, 0.0, 0.0935;
                0.0, 0.0515, 0.0;
                -0.186, 0.0, 0.0;
                0.0, 0.0, 0.0225;
                0.0, 0.095, 0.0;
                0.0, 0.0, 0.0;
                -0.1115, 0.0, 0.0;];
radii = [0.09, 0.09, 0.0935; %[X,Y,Z]
         0.09, 0.0515, 0.09; %[X,Z,Y]
         0.186, 0.09, 0.107; %[Z,X,Y]
         0.09, 0.07, 0.1115; %[Z,Y,X]
         0.06, 0.084, 0.07; %[Z,X,Y]
         0.06, 0.06 0.07; %[X,Y,Z]
         0.1115, 0.441, 0.041;]; %[Z,X,Y]
     
 cubePosition = [0,0,0];
 
 collision = CollisionChecker(robot, centerPoints, radii);
 collision.plotEllipsoids();
 hold on;
 cube = Cube(0.1,20,cubePosition);
 
 collision.checkCollision(cube.getPoints)
 
 %% Dobot Collision checking with cube
 
 robot = DobotMagician(transl(0,0,0));

centerPoints = [0.0, 0.0, 0.0;
                0.0, 0.0, 0.0;
                0.0, 0.0, 0.0;
                0.0, 0.0, 0.0;
                0.0, 0.0, 0.0;];
            
radii = [0.1,0.1,0.1;
         0.1,0.1,0.1;
         0.1,0.1,0.1;
         0.1,0.1,0.1;
         0.1,0.1,0.1;];
     
 cubePosition = [0,0,0];
 
 collision = CollisionChecker(robot, centerPoints, radii);
 collision.plotEllipsoids();
 hold on;
 cube = Cube(0.1,20,cubePosition);
 
 collision.checkCollision(cube.getPoints)