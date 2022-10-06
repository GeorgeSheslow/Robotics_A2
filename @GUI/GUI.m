classdef GUI < matlab.apps.AppBase & handle
    
    properties(Access = private)
        fig;
        simPlot_h;
        
        dobotRobot;
        IRBRobot;
        environment;
        
        % GUI Buttons
        qPButtonsDobot
        qMButtonsDobot
        
        qPButtonsIRB
        qMButtonsIRB
        
        cartButtonsDobot
        cartButtonsIRB
        
        % GUI Button Properties
        
        % Joint Buttons for Dobot
        qBPosXDobot = 1335;
        qBPosYDobot = 260;
        
        % Joint Buttons for IRB
        qBPosXIRB = 1035;
        qBPosYIRB = 260;
        
        % QJog Variables
        qBPosXDelta = 70;
        qBPosYDelta = -30;
        qBSizeX = 50;
        qBSizeY = 30;

        % Cart Buttons for Dobot
        cartBPosXDobot = 1300;
        cartBPosYDobot = 300;

        % Cart Buttons for IRB
        cartBPosXIRB = 1000;
        cartBPosYIRB = 300;

        % CartJog Variables
        cartButtonPos = [60 30; -60 30; 0 60; 0 30; 60 60; -60 60];
        cartButtonNames = ['x+';'x-';'z+';'z-';'y+';'y-'];
        cartBSizeX = 50;
        cartBSizeY = 30;
        
        cartJoggingDelta = 0.01;
        
        % Software Estop
        estopButton
        estopBPosX = 1400;
        estopBPosY = 500;
        estopBSizeX = 60;
        estopBSizeY = 40;
        estopOn = false;
        
    end
    methods 
        function self = GUI()
            self.fig = figure('units','normalized','outerposition',[0 0 1 1]);
            self.simPlot_h = subplot(1,2,1);
            hold(self.simPlot_h, 'on');
            self.setupSim();
            self.setupCommandButtons();
            self.setupJogButtons();
        end
        function setupSim(self)
            % Load Sim Environment
%             self.environment = Environment();
            
            % Load the 2 Robot
            self.dobotRobot = DobotMagician();
            self.dobotRobot.model.base = transl(-0.7,0,0); %0.72
            self.dobotRobot.model.animate(zeros(1,4));
            
            self.IRBRobot = IRB120();
            self.IRBRobot.model.base = transl(0.2,0,0)*rpy2tr(0,0,180,'deg');
            self.IRBRobot.model.animate(zeros(1,6));
        end
        function setupCommandButtons(self)
            % GUI Title
            uicontrol('Style','text','String','Robotics A2 Simulation','FontSize',20,'position',[900 500 500 100]);

            % GUI Software Estop
            self.estopButton = uicontrol('String','ESTOP','FontSize',14,'position',[self.estopBPosX self.estopBPosY self.estopBSizeX self.estopBSizeY],'BackgroundColor','red');
            self.estopButton.Callback = @self.onEstopButton;
        end
        function onEstopButton(self, event, app)
            disp("ESTOP PRESSED!")
            estopOn = true;
        end
        function setupJogButtons(self)
            
            % Setup Joint Jogging Buttons for Dobot
            uicontrol('Style','text','String','Dobot Jog Joints','FontSize',16,'position',[self.qBPosXDobot-80 self.qBPosYDobot+5 150 50]);
            for i = 1:4
                self.qPButtonsDobot{i} = uicontrol('String', ['q',num2str(i),'+'], 'position', [ self.qBPosXDobot                     (self.qBPosYDobot + (i-1)*self.qBPosYDelta) self.qBSizeX self.qBSizeY]);
                self.qMButtonsDobot{i} = uicontrol('String', ['q',num2str(i),'-'], 'position', [(self.qBPosXDobot - self.qBPosXDelta) (self.qBPosYDobot + (i-1)*self.qBPosYDelta) self.qBSizeX self.qBSizeY]);
                
                self.qPButtonsDobot{i}.Callback = @self.onQButtonDobot;
                self.qMButtonsDobot{i}.Callback = @self.onQButtonDobot;
            end
            
            % Setup Joint Jogging Buttons for IRB120
            uicontrol('Style','text','String','IRB120 Jog Joints','FontSize',16,'position',[self.qBPosXIRB-80 self.qBPosYIRB+5 150 50]);
            for i = 1:6
                self.qPButtonsIRB{i} = uicontrol('String', ['q',num2str(i),'+'], 'position', [ self.qBPosXIRB                     (self.qBPosYIRB + (i-1)*self.qBPosYDelta) self.qBSizeX self.qBSizeY]);
                self.qMButtonsIRB{i} = uicontrol('String', ['q',num2str(i),'-'], 'position', [(self.qBPosXIRB - self.qBPosXDelta) (self.qBPosYIRB + (i-1)*self.qBPosYDelta) self.qBSizeX self.qBSizeY]);
            
                self.qPButtonsIRB{i}.Callback = @self.onQButtonIRB;
                self.qMButtonsIRB{i}.Callback = @self.onQButtonIRB;
            end

            % Setup EE Jogging Buttons for Dobot
            uicontrol('Style','text','String','Dobot End Effector Jogging','FontSize',16,'position',[self.cartBPosXDobot-50 self.cartBPosYDobot+80 150 50]);
            for i = 1:size(self.cartButtonPos,1)
               self.cartButtonsDobot{i} = uicontrol('String', self.cartButtonNames(i,:), 'position',[(self.cartBPosXDobot + self.cartButtonPos(i,1)) (self.cartBPosYDobot + self.cartButtonPos(i,2)) self.cartBSizeX self.cartBSizeY]);
               self.cartButtonsDobot{i}.Callback = @self.onCartButtonDobot;
            end
            
            % Setup EE Jogging Buttons for IRB120
            uicontrol('Style','text','String','IRB120 End Effector Jogging','FontSize',16,'position',[self.cartBPosXIRB-50 self.cartBPosYIRB+80 150 50]);
            for i = 1:size(self.cartButtonPos,1)
                self.cartButtonsIRB{i} = uicontrol('String', self.cartButtonNames(i,:), 'position', [(self.cartBPosXIRB + self.cartButtonPos(i,1)) (self.cartBPosYIRB + self.cartButtonPos(i,2)) self.cartBSizeX self.cartBSizeY]);
                self.cartButtonsIRB{i}.Callback = @self.onCartButtonIRB;
            end
        end
        
        function onQButtonDobot(self, event, app)
            disp(event.String);
            self.qJogRobot(self.dobotRobot, event.String);
        end
        function onQButtonIRB(self, event, app)
            disp(event.String);
            self.qJogRobot(self.IRBRobot, event.String);
        end
        function onCartButtonDobot(self, event, app)
            self.cartJogRobot(self.dobotRobot, event.String);
        end
        function onCartButtonIRB(self, event, app)
            self.cartJogRobot(self.IRBRobot, event.String);
        end
        function qJogRobot(self,robot,jointID)
            dir = jointID(3);
            id = regexp(jointID, '\d+', 'match');
            id = [id{:}];
            jointID = str2double(id);
            
            % get joint angles
            joints = robot.model.getpos();
            % update joints
            if dir == '+'
                joints(jointID) = joints(jointID) + deg2rad(5);
            elseif dir == '-'
                joints(jointID) = joints(jointID) - deg2rad(5);
            end
            % check joint limits
            qlim = robot.model.qlim();
            if (joints(jointID) > qlim(jointID,1)) && (joints(jointID) < qlim(jointID,2))
                robot.model.animate(joints);
            else
                disp('jog out of joint limits');
                disp(jointID);
            end
        end
        function cartJogRobot(self,robot, dir)
            disp(dir);
            currentPose = robot.model.fkine(robot.model.getpos());
            currentPosition = currentPose(1:3,4);
            desiredPosition = currentPosition;

            switch(dir)
                case 'x+'
                    desiredPosition(1) = desiredPosition(1) + self.cartJoggingDelta;
                case 'x-'
                    desiredPosition(1) = desiredPosition(1) - self.cartJoggingDelta;
                case 'y+'
                    desiredPosition(2) = desiredPosition(2) + self.cartJoggingDelta;
                case 'y-'
                    desiredPosition(2) = desiredPosition(2) - self.cartJoggingDelta;
                case 'z+'
                    desiredPosition(3) = desiredPosition(3) + self.cartJoggingDelta;
                case 'z-'
                    desiredPosition(3) = desiredPosition(3) - self.cartJoggingDelta;
            end
            
            q = robot.model.ikcon(transl(desiredPosition)); %TODO change to RMRC
            robot.model.animate(q);
        end
    end
end