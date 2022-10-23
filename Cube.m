classdef Cube < handle
    properties (Access = public)
        position;
        cubeLength;
        cubeDensity;
        sideLength;
        cubePoints;
        plot_h;
    end
    methods 
        function self = Cube(cubeLength, cubeDensity, position)
            self.updateParameters(cubeLength, cubeDensity, position)
        end
        function updateParameters(self, cubeLength, cubeDensity, position)
            self.cubeLength = cubeLength;
            self.cubeDensity = cubeDensity;
            self.position = position;
        end
        function removePlots(self)
            self.plot_h.reset();
            refreshdata
            drawnow            
        end
        function updatePlot(self)
            self.sideLength = -(self.cubeLength/2):(self.cubeLength/self.cubeDensity):(self.cubeLength/2);
            [Y,Z] = meshgrid(self.sideLength,self.sideLength);
            sizeMat = size(Y);
            X = repmat(self.cubeLength/2,sizeMat(1),sizeMat(2));
            % Combine one surface as a point cloud
            self.cubePoints = [X(:),Y(:),Z(:)];
            self.cubePoints = [ self.cubePoints ...
                         ; self.cubePoints * rotz(pi/2)...
                         ; self.cubePoints * rotz(pi) ...
                         ; self.cubePoints * rotz(3*pi/2) ...
                         ; self.cubePoints * roty(pi/2) ...
                         ; self.cubePoints * roty(-pi/2)];         
            self.cubePoints = self.cubePoints + repmat(self.position,size(self.cubePoints,1),1);
            try
                self.plot_h.reset();
            end
            self.plot_h = plot3(self.cubePoints(:,1),self.cubePoints(:,2),self.cubePoints(:,3),'r.');
            refreshdata
            drawnow
        end
        function move(self,position)
            self.position = position;
            self.updatePlot()
        end
        function points = getPoints(self)
            points = self.cubePoints;
        end
    end
end