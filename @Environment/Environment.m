classdef Environment

    properties
        table
        trays
        %any other objects
    end
    
    methods (Access =public)
        function self = Environment()
            surf([-1.8,-1.8;1.8,1.8],[-2,3;-2,3],[0.001,0.001;0.001,0.001],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            hold on
            self.AddObj('table',transl(0,0,0));
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

