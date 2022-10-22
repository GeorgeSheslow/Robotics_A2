classdef Paper < handle
    properties
        vertices
        transformedVertices
        mesh_h
        currentPose
    end
    
    methods
        function self = Paper(pose)

            self.mesh_h = PlaceObject('paper.ply');
            self.vertices = get(self.mesh_h,'Vertices');
            self.MoveObj(pose*rpy2tr(0,0,pi/2));
        end
        function MoveObj(self,Pose)
            Pose(1:3,1:3) = Pose(1:3,1:3) * rpy2r(0,0,pi/2);
            self.transformedVertices = [self.vertices,ones(size(self.vertices,1),1)] * Pose';
            set(self.mesh_h,'Vertices',self.transformedVertices(:,1:3));
            drawnow();
            self.currentPose = Pose;
        end
    end
end

