classdef Environment < handle
    properties(Access = private)
        table
        tray
        estop
        extinguisher
        fence
        light
        %any other objects
    end
    properties(Access = public)
        trayOnePos = transl(0.05,-0.3,0.71)*trotz(90,'deg');
        trayTwoPos = transl(0.05,0.3,0.71)*trotz(90,'deg');
        trayThreePos = transl(-0.38,0,0.71)*trotz(90,'deg'); % Middle Dobot Tray
    end
    methods (Access =public)
        function self = Environment(type,pose)
            surf([-2,-2;2,2],[-2,2;-2,2],[-0.001,-0.001;-0.001,-0.001],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            axis equal
            hold on
            self.AddObj('table',pose * transl(0,0,0));

            self.AddObj('tray',pose * self.trayOnePos);
            self.AddObj('tray',pose * self.trayTwoPos);
            self.AddObj('tray',pose * self.trayThreePos);

            if type == "Full"
                self.AddObj('estop',pose * transl(0.6,0.4,0.71));
                self.AddObj('extinguisher',pose * transl(-0.8,-0.7,0));

                self.AddObj('fence',pose * transl(1,0,0));
                self.AddObj('fence',pose * transl(-1,0,0));
                self.AddObj('fence',pose * transl(0,-1,0)*trotz(90,'deg'));
                self.AddObj('fence',pose * transl(0,1,0)*trotz(90,'deg'));

                self.AddObj('light',pose * transl(0.6,1,0.8));
                self.AddObj('light',pose * transl(-0.6,1,0.8));
                self.AddObj('light',pose * transl(0.6,-1,0.8));
                self.AddObj('light',pose * transl(-0.6,-1,0.8));
            end
        end
    end
    methods (Static)
        % Function to add .ply models to simulation
        function AddObj(name,pose)
            mesh_h = PlaceObject([name, '.ply']);
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * pose';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
            drawnow();
        end      
    end
end

