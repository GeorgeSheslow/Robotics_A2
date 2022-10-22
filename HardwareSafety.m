classdef HardwareSafety
    properties(Access = public)
    safetyHardware;
    gui;
    end
    
    methods
        function self = HardwareSafety(port,gui)
            self.safetyHardware = serialport(port,9600);
            self.gui = gui;
            configureTerminator(self.safetyHardware,"CR/LF");
            flush(self.safetyHardware);
            self.safetyHardware.UserData = struct("IRData",[],"IR", 1, "Counter", 1, "Estop", 1);
            configureCallback(self.safetyHardware,"terminator",@self.hardwareSafetyCallback);
        end

        function hardwareSafetyCallback(self, src, ~)

            % Read the ASCII data from the serialport object.
            data = split(readline(src));

            if data(1,:) == "IR"
                src.UserData.IRData(src.UserData.Counter) = str2double(data(2,:));
                src.UserData.Counter = src.UserData.Counter +1;
                src.UserData.IR = mean(src.UserData.IRData);% average data and save
                if src.UserData.Counter > 10
                    src.UserData.Counter = 1;
                end
            end

            if data(1,:) == "EStop"

                src.UserData.Estop = str2double(data(2,:));
            end
            %src.UserData
            
            % Update GUI with this information
            self.gui.updateSafetyVars(src.UserData.Estop, src.UserData.IR);
        end
    end
end
%%
% safety = struct;
% safety.guiEStop = 0; % GUI button state
% safety.hardwareEStop = 0; % Arduino Estop button
% safety.hardwareIR = 0; % Arduino IR Sensor
% safety.StopState = 0; % Emergency Stop State
% safety.SafetyState = 0; % User and second motion for Estop State, stop and be able to resume simulation


%  a = timer('ExecutionMode','fixedRate','Period',1,'TimerFcn',@my_callback_fcn{hardware});
%  start(a);
% 
%%
% clear safetyHardware