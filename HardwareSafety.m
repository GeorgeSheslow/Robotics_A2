classdef HardwareSafety
    properties(Access = public)
    safetyHardware;
    gui;
    IR_threshold = 30;
    IR_averaging = 5;
    end
    
    methods
        function self = HardwareSafety(port,gui)
            self.safetyHardware = serialport(port,9600);
            self.gui = gui;
            configureTerminator(self.safetyHardware,"CR/LF");
            flush(self.safetyHardware);
            self.safetyHardware.UserData = struct("IRData",[],"IR", 1, "Counter", 1, "Estop", 1, "Safety", 1);
            configureCallback(self.safetyHardware,"terminator",@self.hardwareSafetyCallback);
        end

        function hardwareSafetyCallback(self, src, ~)

            % Read the ASCII data from the serialport object.
            data = split(readline(src));

            if data(1,:) == "IR"
                src.UserData.IRData(src.UserData.Counter) = str2double(data(2,:));
                src.UserData.Counter = src.UserData.Counter +1;
                if src.UserData.Counter > self.IR_averaging
                    src.UserData.Counter = 1;
                end
                src.UserData.IR = self.IRCalculation(src.UserData.IRData);
                src.UserData.Safety = self.IRStop(src.UserData.IR);
            end

            if data(1,:) == "EStop"

                src.UserData.Estop = str2double(data(2,:));
            end
%             src.UserData
            
            % Update GUI with this information
            self.gui.updateSafetyVars(src.UserData.Estop, src.UserData.Safety, src.UserData.IR);
        end
        
        function [IR_dis] = IRCalculation(self,data)
            IR_avg = mean(data);
            % IR distance calculations
            if(IR_avg < 215)
                IR_dis = exp(log(IR_avg /7289)/-0.87);
            else
                IR_dis = log(IR_avg / 789.51) / -0.023;
            end
        end
        function [stop] = IRStop(self,data)
            % IR threshold compare
            if(data > self.IR_threshold)
                stop = 0;
                % IR out of range condition option
                if (data > 150)
                    stop = 0;
                end
            else
                stop = 1;
            end
        end
    end
end
