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
%             self.dobotRobot = DobotMagician();
%             self.dobotRobot.model.base = transl(-0.7,0,0); %0.72
%             self.dobotRobot.model.animate(zeros(1,4));
            
%             self.IRBRobot = IRB120();
%             self.IRBRobot.model.base = transl(0.2,0,0)*rpy2tr(0,0,180,'deg');
%             self.IRBRobot.model.animate(zeros(1,6));
        end
        function setupJogButtons(self)
            
            % Setup Joint Jogging Buttons for Dobot
            uicontrol('Style','text','String','Dobot Jog Joints','FontSize',16,'position',[self.qBPosXDobot-80 self.qBPosYDobot+5 150 50]);
            for i = 1:4
                self.qPButtonsDobot{i} = uicontrol('String', ['q',num2str(i),'+'], 'position', [ self.qBPosXDobot                     (self.qBPosYDobot + (i-1)*self.qBPosYDelta) self.qBSizeX self.qBSizeY]);
                self.qMButtonsDobot{i} = uicontrol('String', ['q',num2str(i),'-'], 'position', [(self.qBPosXDobot - self.qBPosXDelta) (self.qBPosYDobot + (i-1)*self.qBPosYDelta) self.qBSizeX self.qBSizeY]);
                
                self.qPButtonsDobot{i}.Callback = @self.onPButtonDobot;
                self.qMButtonsDobot{i}.Callback = @self.onMButtonDobot;
            end
            
            % Setup Joint Jogging Buttons for IRB120
            uicontrol('Style','text','String','IRB120 Jog Joints','FontSize',16,'position',[self.qBPosXIRB-80 self.qBPosYIRB+5 150 50]);
            for i = 1:6
                self.qPButtonsIRB{i} = uicontrol('String', ['q',num2str(i),'+'], 'position', [ self.qBPosXIRB                     (self.qBPosYIRB + (i-1)*self.qBPosYDelta) self.qBSizeX self.qBSizeY]);
                self.qMButtonsIRB{i} = uicontrol('String', ['q',num2str(i),'-'], 'position', [(self.qBPosXIRB - self.qBPosXDelta) (self.qBPosYIRB + (i-1)*self.qBPosYDelta) self.qBSizeX self.qBSizeY]);
            
                self.qPButtonsIRB{i}.Callback = @self.onPButtonIRB;
                self.qMButtonsIRB{i}.Callback = @self.onMButtonIRB;
            end

            % Setup EE Jogging Buttons for Dobot
            uicontrol('Style','text','String','Dobot End Effector Jogging','FontSize',16,'position',[self.cartBPosXDobot-50 self.cartBPosYDobot+80 150 50]);
            for i = 1:size(self.cartButtonPos,1)
               self.cartButtonsDobot{i} = uicontrol('String', self.cartButtonNames(i,:), 'position',[(self.cartBPosXDobot + self.cartButtonPos(i,1)) (self.cartBPosYDobot + self.cartButtonPos(i,2)) self.cartBSizeX self.cartBSizeY]);
            end
            
            % Setup EE Jogging Buttons for IRB120
            uicontrol('Style','text','String','IRB120 End Effector Jogging','FontSize',16,'position',[self.cartBPosXIRB-50 self.cartBPosYIRB+80 150 50]);
            for i = 1:size(self.cartButtonPos,1)
                self.cartButtonsIRB{i} = uicontrol('String', self.cartButtonNames(i,:), 'position', [(self.cartBPosXIRB + self.cartButtonPos(i,1)) (self.cartBPosYIRB + self.cartButtonPos(i,2)) self.cartBSizeX self.cartBSizeY]);
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