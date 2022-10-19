%% LUKE HOWARDS WORK!!!
classdef IRB120Gripper < handle
    %IRB120 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
       % workspace; 
        
        model; 
        
        base;
        
        plyDataStorage;
        
        workspace;
        
        currentQ;
        
        collisionEllipsoids;
             
    end
    
    methods
        function self = IRB120Gripper(spawnPose)
            %IRB120 Construct an instance of this class
            %   Detailed explanation goes here
            self.base = spawnPose;           
            self.workspace = self.generateWorkspace()
            self.GenerateIRB120Gripper();
            self.currentQ = [0 0 0 0 0 0];
            
            self.Display()
            axis([-0.2 0.2 -0.2 0.2 -0.3 0.3])
            disp("finished")
            self.model.teach()
            
        end
        %% Generate IRB120 Robot
        %Create and return an IRB120 robot model
        function GenerateIRB120Gripper(self)
            pause(0.001);
            name = ['IRBGripper_',datestr(now,'yyyymmddTHHMMSSFFF')];
            
            %Prismatic Joint of the LinearUR5 model taken from the robotics
            %toolbox
            L1 = Link([pi     0       0       pi/2    1]);
            L1.qlim = [-0.02 0];
            
            

            %Creating the model using DH params
            self.model = SerialLink([L1],'name',name);
            self.model.base = self.base;
        end

        %% Display the robot
        %Plots and colours the robot model, displaying a realistic
        %representation of the robot 
        function Display(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                disp(["loading link: ", num2str(linkIndex)])
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['IRB120Gripper_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
                self.plyDataStorage{linkIndex + 1} = plyData{linkIndex + 1};
                
            end
            
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            %Attempt to colour the robot if possible (colours must be
            %avaliable in the ply file
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end 
            end          
        end 
        
        %% Fucntion to move end effector
        function updateQ(self, qVals) 
            self.model.animate(qVals);
            drawnow()
            pause(0.03)
        end
        %% Fucntion to get q Vals
        function [qVals] = getQVals(self)
            qVals = self.model.getpos();
        end
        
        %% Method to Retract Gripper
        function retract(self) 
            for i = 1:20 
                self.model.animate([(i/20)*-0.02])
                pause(0.1)
            end
        end
        
        %% Method to Extend Gripper
        function extend(self) 
            for i = 1:20 
                num = -0.02*(((20-i))/20)
                self.model.animate(num)
                pause(0.1)
            end

        end
        
        
        
        %% Function to animate movement
        function animateMovement(self, qMatrix) 
            for i = 1:size(qMatrix, 1)
                self.model.animate(qMatrix(i,:));
                drawnow()
                pause(0.03)
            end
        end
 
        %% Fucntion to Demo Movement
        %!!!Will result in collision at upper limits
        function limitDemo(self)
            qMatrix2 = jtraj([deg2rad(-165) deg2rad(-110) deg2rad(-110) deg2rad(-160) deg2rad(-120) 0],[deg2rad(165) deg2rad(110) deg2rad(75) deg2rad(160) deg2rad(120) 0], 100);
            qMatrix1 = jtraj([0 0 0 0 0 0], [deg2rad(-165) deg2rad(-110) deg2rad(-110) deg2rad(-160) deg2rad(-120) 0], 100);
            qMatrix3 = jtraj([deg2rad(165) deg2rad(110) deg2rad(75) deg2rad(160) deg2rad(120) 0],[0 0 0 0 0 0], 100)
            self.animateMovement(qMatrix1)
            pause(1)
            self.animateMovement(qMatrix2)
            pause(1) 
            self.animateMovement(qMatrix3)
        end
        
        %% Method to calculate Collision Ellipsoids 
        function showCollisionEllipsoids(self)

        end
        
        %% Method to generate collision ellipsoids 
        function generateCollisionEllipsoids(self) 
            
            self.currentQ = self.model.getpos()
            jointTransforms = {};
            modelPoints = {self.base};
            %Generate current joint transforms
            for i = 1:6
                jointTransforms{i} = self.model.A(i, self.currentQ);
                modelPoints{i+1} = cell2mat(modelPoints(i))*cell2mat(jointTransforms(i));
            end
            
            
            for i = 1:7 
                hold on
                trplot(cell2mat(modelPoints(i)), 'length', 0.3)
                hold off
            end
            
            
%           Setting up the first  
            centre1 = transl(cell2mat(modelPoints(2))*(transl(-0.05,0.15,0)))
            rotationMatrix1 = cell2mat(modelPoints(2));
            rad1 = [0.2,0.15,0.15];
            [x1,y1,z1] = ellipsoid(centre1(1),centre1(2),centre1(3),rad1(1),rad1(2),rad1(3));
            hold on
            s = surf(x1,y1,z1)
            rotations = tr2rpy(rotationMatrix1,'deg')
            %rotate(s,[1 0 0],rotations(1),centre)
            %rotate(s,[0 1 0],rotations(1),centre)
            %rotate(s,[0 0 1],rotations(1),centre)
            
            hold off
            linkCentrePoints = [[0,0,0],
                                [0,0,0],
                                [0,0,0],
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]];
            linkRadii = [[3,2,1],
                         [3,2,1],
                         [3,2,1],
                         [3,2,1],
                         [3,2,1],
                         [3,2,1]];
             self.collisionEllipsoids = [linkCentrePoints, linkRadii];
             

            %[X,Y,Z] = ellipsoid(centerPoint (1),centerPoint (2),centerPoint (3),radii(1),radii(2),radii(3));
        end
        
        %% Method to update ellipsoid locations 
        function updateEllipsoids(self) 
            
        end
        
        %% function
        function [workspace] = generateWorkspace(self)
            xyz = transl(self.base)
            workspace = [xyz(1)-1
                         xyz(1)+1
                         xyz(2)-1
                         xyz(2)+1
                         xyz(3)
                         xyz(3)+1]
        end
    end
end

