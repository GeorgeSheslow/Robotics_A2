classdef DobotMagician < handle
%% Variables - Public
    properties(Access =public) 
        name = 'DobotMagician';
        model;
        toolOffset = [0 0 -0.05];
        defaultQ  = [0,deg2rad(8),deg2rad(133),deg2rad(40)];
        trajGen;
        
        centerPoints = [0.0, 0.0, 0.03;
                        0.0, 0.02, 0.0;
                        -0.07, -0.02, 0.0;
                        -0.065, -0.02, 0.0;
                        0.0, 0.0, 0.0;];

        radii = [0.1,0.1,0.05;
                 0.09,0.09,0.07;
                 0.095,0.05,0.03;
                 0.15,0.055,0.0155;
                 0.0175,0.0175,0.0705;];
    end
%% Variables - Private
    properties(Access =private)
        workspace = [-0.5 0.5 -0.5 0.5 -0.1 0.7];
    end
 %% Public Methods
    methods (Access = public)
%% Constructor
        function self = DobotMagician(T)
            self.CreateModel();
            self.model.base = T;
            self.PlotAndColourRobot(self.name);
            self.model.animate(self.defaultQ);
            self.trajGen = RMRCTrajGen(self.model);
            self.trajGen.toolOffset = self.toolOffset;
            self.trajGen.epsilon = 0.0001;
            self.trajGen.setupCollisionChecker(self, self.centerPoints, self.radii);
        end
    end
    methods (Access =private)
%% Create Dobot with DH parameters
        function CreateModel(self)       
            L(1) = Link('d',0.103+0.0362,    'a',0,      'alpha',-pi/2,  'offset',0);
            L(2) = Link('d',0,        'a',0.135,  'alpha',0,      'offset',-pi/2);
            L(3) = Link('d',0,        'a',0.147,  'alpha',0,      'offset',0);
            L(4) = Link('d',0,        'a',0.06,      'alpha',pi/2,  'offset',-pi/2);

            L(1).qlim = [-135 135]*pi/180;
            L(2).qlim = [0 85]*pi/180;
            L(3).qlim = [0 135]*pi/180;
            L(4).qlim = [0 90]*pi/180;
            self.model = SerialLink(L,'name',self.name);
        end   
%% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and colour them in if data is available 
        function PlotAndColourRobot(self,fileName)
            for linkIndex = 0:self.model.n
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([fileName,'_J',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
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
                vertexColours = [0.5,0.5,0.5]; % Default if no colours in plyData
                try 
                     vertexColours = [plyData{linkIndex+1}.vertex.red ...
                                     , plyData{linkIndex+1}.vertex.green ...
                                     , plyData{linkIndex+1}.vertex.blue]/255;

                catch ME_1
                    disp(ME_1);
                    display('No vertex colours in plyData');
                    try 
                         vertexColours = [plyData{linkIndex+1}.face.red ...
                                     , plyData{linkIndex+1}.face.green ...
                                     , plyData{linkIndex+1}.face.blue]/255;
                    catch ME_1
                        disp(ME_1);
                        display('Also, no face colours in plyData, so using a default colour');
                    end
                end
                h.link(linkIndex+1).Children.FaceVertexCData = vertexColours;
                h.link(linkIndex+1).Children.FaceColor = 'interp';
            end
        end 
    end
end