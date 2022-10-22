classdef Environment

    properties
        table
        tray
        estop
        extinguisher
        fence
        light
        %any other objects
    end
    
    methods (Access =public)
        function self = Environment(type)
            surf([-2,-2;2,2],[-2,2;-2,2],[-0.001,-0.001;-0.001,-0.001],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            axis equal
            hold on
            self.AddObj('table',transl(0,0,0));

            self.AddObj('tray',transl(0,-0.3,0.71));
            self.AddObj('tray',transl(0,0.3,0.71));
            self.AddObj('tray',transl(-0.4,0,0.71)*trotz(90,'deg'));
            self.AddObj('paper',transl(0,0.3,0.74));

            if type == "Full"
                self.AddObj('estop',transl(0.6,0.4,0.71));
                self.AddObj('extinguisher',transl(-0.8,-0.7,0));

                self.AddObj('fence',transl(1,0,0));
                self.AddObj('fence',transl(-1,0,0));
                self.AddObj('fence',transl(0,-1,0)*trotz(90,'deg'));
                self.AddObj('fence',transl(0,1,0)*trotz(90,'deg'));

                self.AddObj('light',transl(0.6,1,0.8));
                self.AddObj('light',transl(-0.6,1,0.8));
                self.AddObj('light',transl(0.6,-1,0.8));
                self.AddObj('light',transl(-0.6,-1,0.8));
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

