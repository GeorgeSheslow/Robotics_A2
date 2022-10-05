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

    end
    methods 
        function self = GUI()
            self.fig = figure('units','normalized','outerposition',[0 0 1 1]);
            self.simPlot_h = subplot(1,2,1);
            hold(self.simPlot_h, 'on');
            self.setupSim();
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
        function setupJogButtons(self)
            % Joint Buttons for Dobot
            qBPosXDobot = 1300;
            qBPosYDobot = 300;
            % Joint Buttons for Dobot
            qBPosXIRB = 1000;
            qBPosYIRB = 300;
            % Position Variables
            qBPosXDelta = 70;
            qBPosYDelta = -30;
            qBSizeX = 50;
            qBSizeY = 30;

            for i = 1:4
                self.qPButtonsDobot{i} = uicontrol('String', ['q',num2str(i),'+'], 'position', [ qBPosXDobot                 (qBPosYDobot + (i-1)*qBPosYDelta) qBSizeX qBSizeY]);
                self.qMButtonsDobot{i} = uicontrol('String', ['q',num2str(i),'-'], 'position', [(qBPosXDobot - qBPosXDelta) (qBPosYDobot + (i-1)*qBPosYDelta) qBSizeX qBSizeY]);
                
                self.qPButtonsDobot{i}.Callback = @self.onPButtonDobot;
                self.qMButtonsDobot{i}.Callback = @self.onMButtonDobot;
            end
            for i = 1:6
                self.qPButtonsIRB{i} = uicontrol('String', ['q',num2str(i),'+'], 'position', [ qBPosXIRB                (qBPosYIRB + (i-1)*qBPosYDelta) qBSizeX qBSizeY]);
                self.qMButtonsIRB{i} = uicontrol('String', ['q',num2str(i),'-'], 'position', [(qBPosXIRB - qBPosXDelta) (qBPosYIRB + (i-1)*qBPosYDelta) qBSizeX qBSizeY]);
            
                self.qPButtonsIRB{i}.Callback = @self.onPButtonIRB;
                self.qMButtonsIRB{i}.Callback = @self.onMButtonIRB;
            end            
        end
        function onPButtonDobot(self, event, app)
            disp(event.String);
            self.jogRobot(self.dobotRobot, '+', event.String);
        end
        function onMButtonDobot(self, event, app)
            disp(event.String);
            self.jogRobot(self.dobotRobot, '-', event.String);
        end
        function onPButtonIRB(self, event, app)
            self.jogRobot(self.IRBRobot, '+', event.String);
        end
        function onMButtonIRB(self, event, app)
            self.jogRobot(self.IRBRobot, '-', event.String);
        end
        function jogRobot(self,robot,dir,jointID)
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
    end
end