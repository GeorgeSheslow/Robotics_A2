classdef VSGUI < matlab.apps.AppBase & handle
    properties
        fig
        simPlot_h
        
        dobot;
        IRB;
        
        servoing;
        
        cartButtonPos = [60 30; -60 30; 0 60; 0 30; 60 60; -60 60];
        cartButtonNames = ['x+';'x-';'z+';'z-';'y+';'y-'];
        cartBSize = [50 30];
        cartButtonsDobot;
        cartSelectButtonsDobot;
        cartJoggingDelta = 0.03;
        
        % Cart Buttons for Dobot
        cartBPosDobot = [1220 430];
    end
    methods
        function self = VSGUI()
            self.fig = figure('units','normalized','outerposition',[0 0 1 1]);
            self.simPlot_h = subplot(1,2,1);
            hold(self.simPlot_h, 'on');
            self.dobot = DobotMagician(transl(1,0,0)*trotz(pi));
            self.IRB = IRB120(transl(0,0,0));
            hold on
            self.servoing = visualServo(self.dobot, self.IRB);
            
            % Add buttons
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
            
            % Do the thing
            self.servoing.vs();
        end
        function onCartButtonsDobot(self, event, app)
            jogType = [self.cartDirSelectDobot,event.String];
            self.cartJogRobot(self.dobotRobot,jogType);
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
    end
end