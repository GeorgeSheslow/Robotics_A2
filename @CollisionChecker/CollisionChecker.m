classdef CollisionChecker < handle
    properties (Access = public)
        centerPoints;
        radii;
        robot;
    end
    methods 
        function self = CollisionChecker(robot, centerPoints, radii)
            self.robot = robot;
            self.centerPoints = centerPoints;
            self.radii = radii;
        end
        function plotEllipsoids(self)
            for i = 1:self.robot.model.n
                [X, Y, Z] = ellipsoid(self.centerPoints(i,1), self.centerPoints(i,2), self.centerPoints(i,3), self.radii(i,1), self.radii(i,2), self.radii(i,3));
                self.robot.model.points{i} = [X(:),Y(:),Z(:)];
                warning off
                self.robot.model.faces{i} = delaunay(self.robot.model.points{i});    
                warning on;
            end
            self.robot.model.plot3d(self.robot.model.getpos());
        end
        function collision = checkCollision(self, cubePoints)
            collision = 0;
            q = self.robot.model.getpos();
            tr = zeros(4,4,self.robot.model.n+1);
            tr(:,:,1) = self.robot.model.base;
            L = self.robot.model.links;
            for i = 1:self.robot.model.n
                 tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end

            % Go through each ellipsoid
            for i = 1: size(tr,3)
                cubePointsAndOnes = [inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]']';
                updatedCubePoints = cubePointsAndOnes(:,1:3);
                algebraicDist = self.GetAlgebraicDist(updatedCubePoints, self.centerPoints(i,:), self.radii(i,:));
                pointsInside = find(algebraicDist < 1);
                collisionPoints = size(pointsInside,1);
                if collisionPoints > 0
                    collision = 1;
                    break
                end
%                 display(['There are ', num2str(collisionPoints),' points inside the ',num2str(i),'th ellipsoid']);
            end
        end
    end
    methods (Static)
        function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

            algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                          + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                          + ((points(:,3)-centerPoint(3))/radii(3)).^2;
            end
    end
end