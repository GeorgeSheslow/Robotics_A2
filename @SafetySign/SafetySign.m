classdef SafetySign < handle
    properties
        currentPose;
        color = 'b';
        alpha = 1;
        mesh = 'none';
        h;
        p;
        paper;
    end
    methods
        function self = SafetySign(pose)
            self.paper = Paper(pose*rpy2tr(0,0,-pi/2));
%             text = TextToTraj("STOP");
%             points = text.GetTraj();
            self.updatePlot(pose);
        end
        function updatePlot(self, pose)
            % update paper location
            self.paper.MoveObj(pose * rpy2tr(0,pi/2,0));
            % get new points, based off paper pose
            p = pose(1:3,4)';
            P=[p(1),p(1),p(1),p(1);
                p(2)-0.1,p(2)+0.1,p(2)+0.1,p(2)-0.1;
                p(3)+0.1,p(3)+0.1,p(3)-0.1,p(3)-0.1];
            self.addSpheres(P,0.05);
        end
        function addSpheres(self,c,r)
            daspect([1 1 1])
            hold on
            [xs,ys,zs] = sphere(40);

            if isvec(c,3)
                c = c(:);
            end

            r = r * ones(numcols(c),1);
            for i = 1:numcols(self.p)
                try
                    delete(self.h{i})
                    refershdata
                    drawnow
                end
            end
            for i=1:numcols(c)
                x = r(i)*xs + c(1,i);
                y = r(i)*ys + c(2,i);
                z = r(i)*zs + c(3,i);
                self.h{i} = surf(x,y,z, 'FaceColor', self.color, 'EdgeColor', self.mesh, 'FaceAlpha', self.alpha);
                refreshdata
                drawnow
            end
            self.p = c;
        end
    end
end