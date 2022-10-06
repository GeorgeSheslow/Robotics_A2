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
        qBPosDobot = [1335 260]; % x,y
        
        % Joint Buttons for IRB
        qBPosIRB = [1035 260]; % x,y
        
        % QJog Variables
        qBPosDelta = [70 -30];
        qBSize = [50 30];

        % Cart Buttons for Dobot
        cartBPosDobot = [1300 300];

        % Cart Buttons for IRB
        cartBPosIRB = [1000 300];

        % CartJog Variables
        cartButtonPos = [60 30; -60 30; 0 60; 0 30; 60 60; -60 60];
        cartButtonNames = ['x+';'x-';'z+';'z-';'y+';'y-'];
        cartBSize = [50 30];
        
        cartJoggingDelta = 0.01;
        
        % Software Estop
        estopButton
        estopBPos = [1350 505];
        estopBSize = [60 40];
        estopOn = false;
        estopLock = false;
        
        % Command Buttons
        simStartButton
        simStopButton
        inputTextEdit
        textPreviewButton
        simStatus
        
        statusPos = [950 505];
        startButtonPos = [1150 510];
        intputTextPos = [950 445];
        
        
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
            
            % Sim Status
            uicontrol('Style','text','String','Simulation Status:','position',[self.statusPos(1) self.statusPos(2)-5 90 30]);
            self.simStatus = uicontrol('Style','text','String','Paused','FontSize',13,'position',[self.statusPos(1)+90 self.statusPos(2) 100 30]);
            
            % Sim Start/Stop Buttons
            self.simStartButton = uicontrol('String','Start Sim','position',[self.startButtonPos(1) self.startButtonPos(2) 70 30]);
            self.simStartButton.Callback = @self.startSim;
            self.simStopButton = uicontrol('String','Stop Sim','position',[self.startButtonPos(1)+90 self.startButtonPos(2) 70 30]);
            self.simStopButton.Callback = @self.stopSim;
          
            % User Input Text for Dobot
            uicontrol('Style','text','String','Dobot Text: ','position',[self.intputTextPos(1) self.intputTextPos(2) 100 30]);
            self.inputTextEdit = uicontrol('Style','edit','position',[self.intputTextPos(1) + 80 self.intputTextPos(2)+5  100 30]);
            self.inputTextEdit.Callback = @self.setInputText;
            
            % User text trajectory preview
            self.textPreviewButton = uicontrol('String','Preview','position',[self.intputTextPos(1) + 200 self.intputTextPos(2)+5  70 30]);
            self.textPreviewButton.Callback = @self.previewText;
            
            % GUI Software Estop
            self.estopButton = uicontrol('String','ESTOP','FontSize',13,'position',[self.estopBPos(1) self.estopBPos(2) self.estopBSize(1) self.estopBSize(2)]);
            self.estopButton.Callback = @self.onEstopButton;
        end
        function previewText(self, event, app)
            disp('Text trajectory preview');
        end
        function setInputText(self, event, app)
            disp('User entered: ');
            disp(event.String);
            % TODO generate text trajectory for dobot 
        end
        function startSim(self, event, app)
            % TODO: Add checker, that word has been entered by user
            disp('Starting Simulation');
            % TODO make button disbaled, during simulation, or change to a
            % continue button
        end
        function stopSim(self, event, app)
            disp('Pausing Simulation');
        end
        function onEstopButton(self, event, app)
            self.estopOn = true;
            self.estopLock = true;
            self.estopButton.BackgroundColor = 'red';
        end
        function setupJogButtons(self)
            
            % Setup Joint Jogging Buttons for Dobot
            uicontrol('Style','text','String','Dobot Jog Joints','FontSize',16,'position',[self.qBPosDobot(1)-80 self.qBPosDobot(2)+5 150 50]);
            for i = 1:4
                self.qPButtonsDobot{i} = uicontrol('String', ['q',num2str(i),'+'], 'position', [ self.qBPosDobot(1)                       (self.qBPosDobot(2) + (i-1)*self.qBPosDelta(2)) self.qBSize(1) self.qBSize(2)]);
                self.qMButtonsDobot{i} = uicontrol('String', ['q',num2str(i),'-'], 'position', [(self.qBPosDobot(1) - self.qBPosDelta(1)) (self.qBPosDobot(2) + (i-1)*self.qBPosDelta(2)) self.qBSize(1) self.qBSize(2)]);
                
                self.qPButtonsDobot{i}.Callback = @self.onQButtonDobot;
                self.qMButtonsDobot{i}.Callback = @self.onQButtonDobot;
            end
            
            % Setup Joint Jogging Buttons for IRB120
            uicontrol('Style','text','String','IRB120 Jog Joints','FontSize',16,'position',[self.qBPosIRB(1)-80 self.qBPosIRB(2)+5 150 50]);
            for i = 1:6
                self.qPButtonsIRB{i} = uicontrol('String', ['q',num2str(i),'+'], 'position', [ self.qBPosIRB(1)                     (self.qBPosIRB(2) + (i-1)*self.qBPosDelta(2)) self.qBSize(1) self.qBSize(2)]);
                self.qMButtonsIRB{i} = uicontrol('String', ['q',num2str(i),'-'], 'position', [(self.qBPosIRB(1) - self.qBPosDelta(1)) (self.qBPosIRB(2) + (i-1)*self.qBPosDelta(2)) self.qBSize(1) self.qBSize(2)]);
            
                self.qPButtonsIRB{i}.Callback = @self.onQButtonIRB;
                self.qMButtonsIRB{i}.Callback = @self.onQButtonIRB;
            end

            % Setup EE Jogging Buttons for Dobot
            uicontrol('Style','text','String','Dobot End Effector Jogging','FontSize',16,'position',[self.cartBPosDobot(1)-50 self.cartBPosDobot(2)+80 150 50]);
            for i = 1:size(self.cartButtonPos,1)
               self.cartButtonsDobot{i} = uicontrol('String', self.cartButtonNames(i,:), 'position',[(self.cartBPosDobot(1) + self.cartButtonPos(i,1)) (self.cartBPosDobot(2) + self.cartButtonPos(i,2)) self.cartBSize(1) self.cartBSize(2)]);
               self.cartButtonsDobot{i}.Callback = @self.onCartButtonDobot;
            end
            
            % Setup EE Jogging Buttons for IRB120
            uicontrol('Style','text','String','IRB120 End Effector Jogging','FontSize',16,'position',[self.cartBPosIRB(1)-50 self.cartBPosIRB(2)+80 150 50]);
            for i = 1:size(self.cartButtonPos,1)
                self.cartButtonsIRB{i} = uicontrol('String', self.cartButtonNames(i,:), 'position', [(self.cartBPosIRB(1) + self.cartButtonPos(i,1)) (self.cartBPosIRB(2) + self.cartButtonPos(i,2)) self.cartBSize(1) self.cartBSize(2)]);
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
                % estop check
                if self.estopLock == true
                    self.estopLock = false;
                    self.estopOn = false;
                    self.estopButton.BackgroundColor = 'white';
                end
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