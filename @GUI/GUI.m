classdef GUI < matlab.apps.AppBase & handle
    
    properties(Access = public)
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
        cartSelectButtonsDobot
        cartDirSelectDobot
        
        cartButtonsIRB
        cartSelectButtonsIRB
        cartDirSelectIRB
        % GUI Button Properties
        
        % Joint Buttons for Dobot
        qBPosDobot = [1255 360]; % x,y
        
        % Joint Buttons for IRB
        qBPosIRB = [1035 360]; % x,y
        
        % QJog Variables
        qBPosDelta = [70 -30];
        qBSize = [50 30];

        % Cart Buttons for Dobot
        cartBPosDobot = [1220 430];

        % Cart Buttons for IRB
        cartBPosIRB = [1000 430];

        % CartJog Variables
        cartButtonPos = [60 30; -60 30; 0 60; 0 30; 60 60; -60 60];
        cartButtonNames = ['x+';'x-';'z+';'z-';'y+';'y-'];
        cartBSize = [50 30];
        
        cartJoggingDelta = 0.03;
        
        % Software Estop
        estopButton
        estopBPos = [1350 705];
        estopBSize = [60 40];
        estopOn = false;
        estopLock = false;
        
        % Command Buttons
        simStartButton
        simStopButton
        inputTextEdit
        textPreviewButton
        simStatus
        
        statusPos = [950 705];
        startButtonPos = [1150 710];
        intputTextPos = [950 645];
        titlePos = [900 700];
        
        safety = struct("emergencyStopState",0,"safetyStopState",0, "guiEstop",0,"hardwareEstop",0,"hardwareIR",0,"guiPause",0);
        safetyLEDS;

        dobotText = "DOBOT"; % default text
        paper;
        safetyJog = 0;
        stateMachineVar =0;
        simOn = 0;
        
        collisionDemoButton;
        collisionOn;
        collisionDir;
        collisionSelectButtons;
        collisionDirButtons;
        
        visualServoDemoButton;
        
        cube;
    end
    methods 
        function self = GUI()
            self.fig = figure('units','normalized','outerposition',[0 0 1 1]);
            self.simPlot_h = subplot(1,2,1);
            hold(self.simPlot_h, 'on');
            self.setupSim();
            self.setupCommandButtons();
            self.setupJogButtons();
            self.setupSafetyLEDS();
            self.setupDemoButtons();
            
            self.cube = Cube(0.1,20,self.environment.trayThreePos(1:3,4)');
        end
        function IRBPickAndPlace(self,ver)
            waitpoint = [-0.3,0,0.6];
            paperoffset = transl(0,0,0.19);
            if ver == 1
                self.paper.MoveObj(self.environment.trayOnePos * transl(0,0,0.03)*rpy2tr(0,0,pi/2));
                [x, traj] = self.IRBRobot.trajGen.getQForLineTraj(self.environment.trayOnePos * paperoffset);
                self.IRBRobot.trajGen.animateQWGUI(traj,self)
                [x, traj] = self.IRBRobot.trajGen.getQForZArcTraj(self.environment.trayThreePos * paperoffset);
                self.IRBRobot.trajGen.animateQWObjWGUI(traj,self.paper, self)
                self.paper.MoveObj(self.environment.trayThreePos * transl(0,0,0.03)*rpy2tr(0,0,pi/2));
                [x, traj] = self.IRBRobot.trajGen.getQForLineTraj(transl(waitpoint) * self.IRBRobot.model.base);
                self.IRBRobot.trajGen.animateQWGUI(traj,self)
            elseif ver == 2
                [x, traj] = self.IRBRobot.trajGen.getQForLineTraj(self.environment.trayThreePos * paperoffset);
                self.IRBRobot.trajGen.animateQWGUI(traj,self)
                [x, traj] = self.IRBRobot.trajGen.getQForZArcTraj(self.environment.trayTwoPos * paperoffset);
                self.IRBRobot.trajGen.animateQWObjWGUI(traj,self.paper, self)
                self.paper.MoveObj(self.environment.trayTwoPos * transl(0,0,0.03)*rpy2tr(0,0,pi/2));
                [x, traj] = self.IRBRobot.trajGen.getQForLineTraj(transl(waitpoint) * self.IRBRobot.model.base);
                self.IRBRobot.trajGen.animateQWGUI(traj,self)
            end
        end
        function updateSafetyVars(self, estop, ir_safety, ir_data)
            self.safety.hardwareEstop = estop;
            self.safety.hardwareIR = ir_safety;
            if self.safety.hardwareEstop == 1
                self.safetyLEDS{4}.BackgroundColor = 'Red';
                self.updateEmergencyState(1);
            else
                self.safetyLEDS{4}.BackgroundColor = 'Green';
                self.updateEmergencyState(0);
            end
            if self.safety.hardwareIR == 1
                self.safetyLEDS{5}.BackgroundColor = 'Red';
                self.updateSafetyState(1);
            else
                self.safetyLEDS{5}.BackgroundColor = 'Green';
                self.updateSafetyState(0);
            end
%             disp(estop);
%             disp(ir_safety);
%             disp(ir_data);
        end
        function updateEmergencyState(self,state)
            if (state == 1  && self.safety.guiEstop == 0) || (state == 1  &&  self.safety.hardwareEstop == 0)
                self.safety.emergencyStopState = 1;
                self.safetyLEDS{1}.BackgroundColor = 'Red';
                self.safetyJog = 0;
            elseif state == 0 && self.safetyJog == 1 && self.safety.guiEstop == 0 && self.safety.hardwareEstop == 0
                self.safety.emergencyStopState = 0;
                self.safetyLEDS{1}.BackgroundColor = 'Green';
            end
        end
        function updateSafetyState(self,state)
            if state == 1
                self.safety.safetyStopState = 1;
                self.safetyLEDS{2}.BackgroundColor = 'Red';
            elseif self.safety.hardwareIR == 0 && self.safety.guiPause == 0
                self.safety.safetyStopState = 0;
                self.safetyLEDS{2}.BackgroundColor = 'Green';
            end
        end
        function setupSim(self)
            % Load Sim Environment
            self.environment = Environment("Full");
            
            % Load the 2 Robot
            self.dobotRobot = DobotMagician(transl(-0.6,0,0.72)); %table height: 0.72        
            self.IRBRobot = IRB120(transl(0.2,0,0.72));
            
            % Add Paper model
            self.paper = Paper(self.environment.trayOnePos * transl(0,0,0.03));
        end
        function setupCommandButtons(self)
            % GUI Title
            uicontrol('Style','text','String','Robotics A2 Simulation: Card Signing Robots','FontSize',20,'position',[self.titlePos(1) self.titlePos(2) 500 100]);
            
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
            figure(2)
            if strlength(self.dobotText) > 0
                write = TextToTraj(self.dobotText);
            else
                write = TextToTraj("Dobot");
            end
            drawPoints = write.GetTraj();
            for i = 1:size(drawPoints,2)
                plot(drawPoints(2,i),drawPoints(1,i),'k.');
                hold on
            end
            set(gca,'ydir','reverse')
            axis padded;
             self.simStatus.String = "Preview Text";
        end
        function setInputText(self, event, app)
            self.dobotText = event.String;
            self.simStatus.String = "Text Added";
        end
        function runSim(self)
            if(self.simOn)
                switch(self.stateMachineVar)
                    case 0
                        disp('Starting Simulation');
                        self.paper.clearText();
                        self.simStatus.String = "IRB Pick/Place";
                        self.IRBPickAndPlace(1);
                    case 1
                        self.simStatus.String = "Dobot RMRC Calcs";
                        write = TextToTraj(self.dobotText);
                        write.addBaseOffsets(self.dobotRobot.model.base(1:3,4));
                        xWrite = write.GetTraj();
                        [x, qMatrix] = self.dobotRobot.trajGen.getQForLineTraj(transl(xWrite(:,1))); % Use RMRC line traj to get to paper level
                        self.simStatus.String = "Dobot Drawing";
                        self.dobotRobot.trajGen.animateQWGUI(qMatrix,self) % Animate
                        [x, qMatrix] = self.dobotRobot.trajGen.getQForTraj(xWrite); % Use RMRC to write text
                        self.drawText(self.dobotRobot,write.getDrawingHeight(),x, qMatrix,0); % animate
                        [x, qMatrix] = self.dobotRobot.trajGen.getQForLineTraj(transl(0.17,0,0.157) * self.dobotRobot.model.base); % Move EE to neutral pose
                        self.dobotRobot.trajGen.animateQWGUI(qMatrix,self)
                    case 2
                        self.simStatus.String = "IRB Pick/Place";
                        self.IRBPickAndPlace(2);
                        self.simStatus.String = "Sim Finished";
                end
            end
            self.stateMachineVar = self.stateMachineVar +1;
            if self.stateMachineVar > 2
                self.stateMachineVar = 0;
            end
            self.startSim();
        end
        function startSim(self, event, app)
            self.simOn = 1;
            event.String = 'Sim Running';
            if self.safety.emergencyStopState == 0
                self.runSim();
            end
        end
        function drawText(self,robot,paperHeight,x, qMatrix, desiredTrajOn)
            for j = 1:size(qMatrix,1)
                newQ = qMatrix(j,:);
                if self.collisionOn
                    if robot.trajGen.collisionChecker.checkCollision(self.cube.cubePoints)
                        self.updateEmergencyState(1)
                    end
                end
                if(self.safety.emergencyStopState)
                    self.simOn = 0;
                    break;
                end
                while(self.safety.safetyStopState)
                    pause(0.5);
                end
                robot.model.animate(newQ);
                drawnow();
                hold on
                pos = robot.model.fkine(robot.model.getpos());
                if pos(3,4) <= paperHeight + 0.005
%                     plot3(pos(1,4)+robot.toolOffset(1),pos(2,4)+robot.toolOffset(2),pos(3,4)+robot.toolOffset(3),'r.');
                    point = [pos(1,4)+robot.toolOffset(1),pos(2,4)+robot.toolOffset(2),pos(3,4)+robot.toolOffset(3)];
                    self.paper.addText(point);
                    if desiredTrajOn
                        plot3(x(1,4)+robot.toolOffset(1),x(2,4)+robot.toolOffset(2),x(3,4)+robot.toolOffset(3),'k.');
                    end
                end
                pause(0.1);
            end 
        end
        function stopSim(self, event, app)
            disp('Pausing Simulation');
            if event.String == "Stop Sim" % TODO:  && state machine variable > 0
                self.safety.guiPause = 1;
                event.BackgroundColor = 'Red';
                event.String = "Stopped";
                self.updateSafetyState(1);
            else
                event.String = "Stop Sim";
                self.safety.guiPause = 0;
                event.BackgroundColor = 'White';
                self.updateSafetyState(0);
            end
        end
        function onEstopButton(self, event, app)
            self.safety.guiEstop = xor(self.safety.guiEstop,1);
            if self.safety.guiEstop == 1
                self.safetyLEDS{3}.BackgroundColor = 'Red';
                event.BackgroundColor = 'Red';
                self.updateEmergencyState(1);
            else
                self.safetyLEDS{3}.BackgroundColor = 'Green';
                event.BackgroundColor = 'White';
            end
        end
        function setupSafetyLEDS(self)
            self.safetyLEDS{1} = uicontrol('Style','text','String',"emergencyStopState",'FontSize',14,'position',[1470 760 140 25],'BackgroundColor','Green');
            self.safetyLEDS{2} = uicontrol('Style','text','String',"safetyStopState",'FontSize',14,'position',[1480 720 120 25],'BackgroundColor','Green');
            self.safetyLEDS{3} = uicontrol('Style','text','String',"GUI ESTOP",'FontSize',14,'position',[1480 670 120 25],'BackgroundColor','Green');
            self.safetyLEDS{4} = uicontrol('Style','text','String',"HW ESTOP",'FontSize',14,'position',[1480 640 120 25],'BackgroundColor','Green');
            self.safetyLEDS{5} = uicontrol('Style','text','String',"HW IR",'FontSize',14,'position',[1480 610 120 25],'BackgroundColor','Green');
        end
        function updateSafetyLEDs(self)
            if self.safety.emergencyStopState == 1
                self.safetyLEDS{1}.BackgroundColor = 'Red';
            else
                self.safetyLEDS{1}.BackgroundColor = 'Green';
            end
            if self.safety.safetyStopState == 1
                self.safetyLEDS{2}.BackgroundColor = 'Red';
            else
                self.safetyLEDS{2}.BackgroundColor = 'Green';
            end
            if self.safety.guiEstop == 1
                self.safetyLEDS{3}.BackgroundColor = 'Red';
            else
                self.safetyLEDS{3}.BackgroundColor = 'Green';
            end
            if self.safety.hardwareEstop == 1
                self.safetyLEDS{4}.BackgroundColor = 'Red';
            else
                self.safetyLEDS{4}.BackgroundColor = 'Green';
            end
            if self.safety.hardwareIR == 1
                self.safetyLEDS{5}.BackgroundColor = 'Red';
            else
                self.safetyLEDS{5}.BackgroundColor = 'Green';
            end
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
            uicontrol('Style','text','String','Dobot End Effector Jogging','FontSize',16,'position',[self.cartBPosDobot(1)-50 self.cartBPosDobot(2)+105 150 50]);
                % +/- buttons
            self.cartButtonsDobot{1} = uicontrol('String','+','position',[(self.cartBPosDobot(1) + self.cartButtonPos(1,1)) (self.cartBPosDobot(2) + self.cartButtonPos(1,2)) self.cartBSize(1) self.cartBSize(2)]);
            self.cartButtonsDobot{1}.Callback = @self.onCartButtonsDobot;
            self.cartButtonsDobot{2} = uicontrol('String','-','position',[(self.cartBPosDobot(1) + self.cartButtonPos(2,1)) (self.cartBPosDobot(2) + self.cartButtonPos(2,2)) self.cartBSize(1) self.cartBSize(2)]);
            self.cartButtonsDobot{2}.Callback = @self.onCartButtonsDobot;
                % x,y,z select
            self.cartSelectButtonsDobot{1} = uicontrol('Style','radiobutton','String','x','position',[1170 500 self.cartBSize(1) self.cartBSize(2)]);
            self.cartSelectButtonsDobot{1}.Callback = @self.OnCartSelectButtonsDobot;
            self.cartSelectButtonsDobot{2} = uicontrol('Style','radiobutton','String','y','position',[1220 500 self.cartBSize(1) self.cartBSize(2)]);
            self.cartSelectButtonsDobot{2}.Callback = @self.OnCartSelectButtonsDobot;
            self.cartSelectButtonsDobot{3} = uicontrol('Style','radiobutton','String','z','position',[1270 500 self.cartBSize(1) self.cartBSize(2)]);
            self.cartSelectButtonsDobot{3}.Callback = @self.OnCartSelectButtonsDobot;
           
            % Setup EE Jogging Buttons for IRB120
            uicontrol('Style','text','String','IRB120 End Effector Jogging','FontSize',16,'position',[self.cartBPosIRB(1)-50 self.cartBPosIRB(2)+105 150 50]);
                % +/- buttons
            self.cartButtonsIRB{1} = uicontrol('String','+','position',[(self.cartBPosIRB(1) + self.cartButtonPos(1,1)) (self.cartBPosIRB(2) + self.cartButtonPos(1,2)) self.cartBSize(1) self.cartBSize(2)]);
            self.cartButtonsIRB{1}.Callback = @self.onCartButtonsIRB;
            self.cartButtonsIRB{2} = uicontrol('String','-','position',[(self.cartBPosIRB(1) + self.cartButtonPos(2,1)) (self.cartBPosIRB(2) + self.cartButtonPos(2,2)) self.cartBSize(1) self.cartBSize(2)]);
            self.cartButtonsIRB{2}.Callback = @self.onCartButtonsIRB;
                % x,y,z select
            self.cartSelectButtonsIRB{1} = uicontrol('Style','radiobutton','String','x','position',[950 500 self.cartBSize(1) self.cartBSize(2)]);
            self.cartSelectButtonsIRB{1}.Callback = @self.OnCartSelectButtonsIRB;
            self.cartSelectButtonsIRB{2} = uicontrol('Style','radiobutton','String','y','position',[1000 500 self.cartBSize(1) self.cartBSize(2)]);
            self.cartSelectButtonsIRB{2}.Callback = @self.OnCartSelectButtonsIRB;
            self.cartSelectButtonsIRB{3} = uicontrol('Style','radiobutton','String','z','position',[1050 500 self.cartBSize(1) self.cartBSize(2)]);
            self.cartSelectButtonsIRB{3}.Callback = @self.OnCartSelectButtonsIRB;
           
        end
        function onCartButtonsDobot(self, event, app)
            jogType = [self.cartDirSelectDobot,event.String];
            self.cartJogRobot(self.dobotRobot,jogType);
        end
        function onCartButtonsIRB(self, event, app)
            jogType = [self.cartDirSelectIRB,event.String];
            self.cartJogRobot(self.IRBRobot,jogType);
        end        
        function OnCartSelectButtonsDobot(self, event, app)
            buttonName = event.String;
            disp(size(self.cartSelectButtonsDobot));
            self.cartDirSelectDobot = buttonName;
            for i = 1:size(self.cartSelectButtonsDobot,2)
                if self.cartSelectButtonsDobot{i}.String ~= buttonName
                    self.cartSelectButtonsDobot{i}.Value = 0;
                end
            end
        end
        function OnCartSelectButtonsIRB(self, event, app)
            buttonName = event.String;
            disp(size(self.cartSelectButtonsIRB));
            self.cartDirSelectIRB = buttonName;
            for i = 1:size(self.cartSelectButtonsIRB,2)
                if self.cartSelectButtonsIRB{i}.String ~= buttonName
                    self.cartSelectButtonsIRB{i}.Value = 0;
                end
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
                if self.safety.safetyStopState == 0 && self.safety.guiEstop == 0 && self.safety.hardwareEstop == 0
                    robot.model.animate(joints);
                    if self.safety.emergencyStopState == 1
                        self.safetyJog = 1;
                        self.updateEmergencyState(0);
                        if self.stateMachineVar > 0
                            self.stateMachineVar = self.stateMachineVar - 1;
                        end
                    end
                end
            else
                disp('jog out of joint limits');
                disp(jointID);
            end
        end
        function cartJogRobot(self,robot, dir)
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
            
            if self.safety.hardwareEstop == 0 && self.safety.guiEstop == 0 && self.safety.safetyStopState == 0 %% need to be able to jog in emergency state
%                 [x, qMatrix] = robot.trajGen.getQForLineTrajWSteps(transl(desiredPosition),10);
%                 robot.trajGen.animateQ(qMatrix)
                if robot.model.n == 6
                    desiredPose = transl(desiredPosition) * trotx(pi);
                elseif robot.model.n == 4
                    desiredPose = transl(desiredPosition);
                end
                q = robot.model.ikcon(desiredPose, robot.model.getpos());
                robot.model.animate(q);
                if self.safety.emergencyStopState == 1
                    self.safetyJog = 1;
                    self.updateEmergencyState(0);
                    if self.stateMachineVar > 0
                        self.stateMachineVar = self.stateMachineVar - 1;
                    end
                end
            end
        end
        function setupDemoButtons(self)
            % Collision GUI buttons
            uicontrol("Style","text","String", "Collision Checking with Cube","FontSize",16, "position",[1400 530 140 50]);
            self.collisionDemoButton = uicontrol("String","Start",'position',[1550 530 50 30]);
            self.collisionDemoButton.Callback = @self.onCollisionDemoButton;
            
                % Cube jogging buttons
            self.collisionSelectButtons{1} = uicontrol("String","+","position",[1500 460 50 30]);
            self.collisionSelectButtons{2} = uicontrol("String","-","position",[1400 460 50 30]);
            self.collisionSelectButtons{1}.Callback = @self.onCollisionSelectButton;
            self.collisionSelectButtons{2}.Callback = @self.onCollisionSelectButton;
            
            self.collisionDirButtons{1} = uicontrol('Style','radiobutton','String','x','position',[1400 500 50 30]);
            self.collisionDirButtons{2} = uicontrol('Style','radiobutton','String','y','position',[1450 500 50 30]);
            self.collisionDirButtons{3} = uicontrol('Style','radiobutton','String','z','position',[1500 500 50 30]);
            self.collisionDirButtons{1}.Callback = @self.onCollisionDirButton;
            self.collisionDirButtons{2}.Callback = @self.onCollisionDirButton;
            self.collisionDirButtons{3}.Callback = @self.onCollisionDirButton;
            
            
            % Visual Servoing GUI buttons
            self.visualServoDemoButton = uicontrol("String","Visual Servoing Demo",'position',[1400 380 120 30]);
            self.visualServoDemoButton.Callback = @onVisualServoDemoButton;
        end
        function onCollisionDemoButton(self, event, app)
            disp("Collision Demo");
            if event.String == "Start"
                event.String = "Stop";
                self.collisionOn = 1;
                self.cube.updatePlot();
            else
                event.String = "Start";
                self.collisionOn = 0;
                self.cube.removePlots();
            end
        end
        function onCollisionSelectButton(self, event, app)
            data = [self.collisionDir, event.String];
            self.cartJogCube(data);
        end
        function onCollisionDirButton(self, event, app)
            buttonName = event.String;
            for i = 1:size(self.collisionDirButtons,2)
                if self.collisionDirButtons{i}.String ~= buttonName
                    self.collisionDirButtons{i}.Value = 0;
                end
            end
            self.collisionDir = event.String;
        end
        function cartJogCube(self,dir)
            % same as for robot but with cube
            currentPosition = self.cube.position;
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
            if self.collisionOn
                self.cube.move(desiredPosition);
            end
        end
        function onVisualServoDemoButton(self, event, app)
            disp("Visual Servoing Demo");
        end
    end
end