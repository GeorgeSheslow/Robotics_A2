classdef Paper < handle
    properties
        vertices
        transformedVertices
        mesh_h
        currentPose
        currentPoseText
        text_h;
        text_data;
    end
    
    methods
        function self = Paper(pose)

            self.mesh_h = PlaceObject('paper.ply');
            self.vertices = get(self.mesh_h,'Vertices');
            self.MoveObj(pose*rpy2tr(0,0,pi/2));
        end
        function MoveObj(self,Pose)
            if size(self.text_data,1) > 1
                % move text as well, for loop
                for i = 2:size(self.text_data,1)
                    % update point position in global space with relative pose
                    % passed in
                    temp = transl(self.text_data(i,1:3));
                    temp = (Pose * inv(self.currentPoseText)) * temp;
                    self.text_data(i,1:3) = temp(1:3,4)';
                    disp(self.text_data(2,1:3))
                end
                % replot
                self.text_h.reset();
                self.text_h = plot3(self.text_data(:,1),  self.text_data(:,2),  self.text_data(:,3),'r.');
                refreshdata
                drawnow
            end
            self.currentPoseText = Pose;
            
            Pose(1:3,1:3) = Pose(1:3,1:3) * rpy2r(0,0,pi/2);
            self.transformedVertices = [self.vertices,ones(size(self.vertices,1),1)] * Pose';
            set(self.mesh_h,'Vertices',self.transformedVertices(:,1:3));
            
            self.currentPose = Pose;
        end
        function addText(self,point)
            self.text_data(size(self.text_data,1)+1,1:3) = point;
            if size(self.text_data,1) > 2
                self.text_h.reset();
            end
            self.text_h = plot3( self.text_data(:,1),  self.text_data(:,2),  self.text_data(:,3),'r.');
        end
        function clearText(self)
            if size(self.text_data,1) > 2
                self.text_h.reset();
                refreshdata
                drawnow
            end
            self.text_data = 0;
        end
    end
end

