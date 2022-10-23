close all
clear
clc
set(figure,'units','normalized','outerpos',[0 0 1 1.2]);

%% IRB Collision checking with cube

robot = IRB120(transl(0,0,0));

centerPoints = [0.0, 0.0, 0.0;
                0.0, 0.0, 0.0;
                0.0, 0.0, 0.0;
                0.0, 0.0, 0.0;
                0.0, 0.0, 0.0;
                0.0, 0.0, 0.0;
                0.0, 0.0, 0.0;];
            
radii = [0.2,0.2,0.2;
         0.2,0.2,0.2;
         0.2,0.2,0.2;
         0.2,0.2,0.2;
         0.2,0.2,0.2;
         0.2,0.2,0.2;
         0.2,0.2,0.2;];
     
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