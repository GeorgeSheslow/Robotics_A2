close all
clear
clc
set(figure,'units','normalized','outerpos',[0 0 1 1.2]);

%% IRB Collision checking with cube
close all
clear
clc
set(figure,'units','normalized','outerpos',[0 0 1 1.2]);

robot = IRB120(transl(0,0,0));

centerPoints = [0.0, 0.0, 0.05;
                0.0, 0.1, 0.0;
                -0.1, 0.0, 0.0;
                0.0, 0.0, 0.12;
                0.0, 0.0, 0.0;
                0.0, 0.0, -0.135;
                0.0, 0.0, 0.0;];
radii = [0.1, 0.1, 0.1; %[X,Y,Z]
         0.1, 0.15, 0.11; %[X,Z,Y]
         0.25, 0.1, 0.13; %[Z,X,Y]
         0.1, 0.09, 0.19; %[Z,Y,X]
         0.08, 0.09, 0.1; %[Z,X,Y]
         0.05, 0.05 0.1; %[X,Y,Z]
         0.1, 0.1, 0.1;]; %[Z,X,Y]
     
 cubePosition = [0.35,0,0.4];
 
 collision = CollisionChecker(robot, centerPoints, radii);
 collision.plotEllipsoids();
 hold on;
 cube = Cube(0.1,20,cubePosition);
cube.updatePlot();
 collision.checkCollision(cube.getPoints())
 
 %% Dobot Collision checking with cube
 close all
clear
clc
set(figure,'units','normalized','outerpos',[0 0 1 1.2]);

 robot = DobotMagician(transl(0,0,0));

centerPoints = [0.0, 0.0, 0.03;
                0.0, 0.02, 0.0;
                -0.07, -0.02, 0.0;
                -0.065, -0.02, 0.0;
                0.0, 0.0, 0.0;];
            
radii = [0.1,0.1,0.05;
         0.09,0.09,0.07;
         0.095,0.05,0.03;
         0.15,0.055,0.0155;
         0.0175,0.0175,0.0705;];
     
 cubePosition = [0.2,0,0.1];
 
 collision = CollisionChecker(robot, centerPoints, radii);
 collision.plotEllipsoids();
 hold on;
 cube = Cube(0.1,20,cubePosition);
 cube.updatePlot();
 collision.checkCollision(cube.getPoints)