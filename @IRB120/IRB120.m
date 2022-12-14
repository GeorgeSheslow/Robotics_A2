classdef IRB120 < handle
    properties
        model;
        workspace = [-1 1 -1 1 -0.2 1.1];
        trajGen;
        toolOffset = [0 0 0.16];
        
        % For collision checking
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
     
    end
    methods
        function self = IRB120(base)
            self.GetIRB120();
            self.model.base = base;
            self.PlotAndColourRobot();
            self.trajGen = RMRCTrajGen(self.model);
            self.trajGen.toolOffset = self.toolOffset;
            self.trajGen.steps = 25;
            self.trajGen.epsilon = 0.035;
            self.trajGen.setupCollisionChecker(self, self.centerPoints, self.radii);
        end
        %% GetIRB120
        % Create and return an IRB120 robot model
        function GetIRB120(self)
            L(1) = Link('d',0.29,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            L(2) = Link('d',0,'a',0.27,'alpha',0,'qlim', deg2rad([-110 110]), 'offset',-pi/2);
            L(3) = Link('d',0,'a',0.07,'alpha',-pi/2,'qlim', deg2rad([-220 70]), 'offset', 0);
            L(4) = Link('d',0.302,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            L(5) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-150,150]), 'offset',-pi/2);
            L(6) = Link('d',-0.072,'a',0,'alpha',pi,'qlim',deg2rad([-360,360]), 'offset', 0);

            self.model = SerialLink(L,'name','IRB120');
        end
        %% PlotAndColourRobot
        % Given a robot index, add the vertices and faces and colour them
        function PlotAndColourRobot(self)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
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
    end
end
