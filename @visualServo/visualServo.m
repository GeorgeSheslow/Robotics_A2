classdef visualServo < handle
    properties (Access = public)
        r1
        r2
        cam
        Tc0
        q0
        sphere_h;
        paper;
        servoingOn;
    end
    methods
        function self = visualServo(r1, r2)
            self.r1 = r1;
            self.r2 = r2;
            self.cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
                'resolution', [1024 1024], 'centre', [512 512],'name', 'IRBCamera');

            self.q0 = (self.r2.model.getpos())';
            self.Tc0 = self.r2.model.fkine(self.q0);
            
            self.cam.T = self.Tc0; % set camera to initial pose
            
            self.cam.plot_camera('Tcam',self.Tc0, 'label','scale',0.05);
            
            lighting gouraud
            light
        end

        function vs(self)
           fps = 25;
            lambda = 0.8;
            depth = 1;
            pStar = bsxfun(@plus, 200*[-1 -1 1 1; -1 1 1 -1], self.cam.pp');

            qr1 = self.r1.model.fkine(self.r1.model.getpos());
            qr1 = qr1(1:3,4);
            P=[qr1(1),qr1(1),qr1(1),qr1(1);
                qr1(2)-0.1,qr1(2)+0.1,qr1(2)+0.1,qr1(2)-0.1;
                qr1(3)+0.35,qr1(3)+0.35,qr1(3)+0.15,qr1(3)+0.15];
%             try
%                 self.sphere_h.reset();
%                 refreshdata
%                 drawnow
% 
%             end
            self.sphere_h = plot_sphere(P, 0.05, 'b');
            hold on
            pose = self.r1.model.fkine(self.r1.model.getpos())*transl(0.02,0,0.2);
            try
                self.paper.MoveObj(pose*troty(pi/2));
            catch
                self.paper = Paper(pose*troty(pi/2));
            end
            drawnow
            

           

            %% plotting graph
            p = self.cam.plot(P, 'Tcam', self.Tc0);
            % show ref location, wanted view when Tc = Tct_star
            self.cam.clf()
            self.cam.plot(pStar, '*'); % create the camera view
            self.cam.hold(true);
            self.cam.plot(P, 'Tcam', self.Tc0, 'o'); % create the camera view
            pause(2)
            self.cam.hold(true);

            self.cam.plot(P); % show initial view
            %%
            ksteps = 0;
            self.servoingOn = 1;
            while (true && self.servoingOn == 1)
                ksteps = ksteps + 1;
                Zest = [];

                % comute the view
                uv = self.cam.plot(P);

                % compute image plane error as a column
                e = uv - pStar; % feature error
                e = e(:);

                % compute Jacobian
                if isempty(depth)
                    % exact depth from simulation
                    pt = homtrans(inv(Tcam), P);
                    J = self.cam.visjac_p(uv, pt(3,:));
                elseif ~isempty(Zest)
                    J = self.cam.visjac_p(uv, Zest);
                else
                    J = self.cam.visjac_p(uv, depth);
                end

                % compute velocity of camera in camera frame
                try
                    v = -lambda * pinv(J) * e;
                catch
                    status = -1;
                    return
                end
                fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

                % ROBOT MOVEMENT
                J2 = self.r2.model.jacobn(self.q0); % jacobian for robot in pose q0
                Jinv = pinv(J2);
                qp = Jinv*v; % joint velocities
                % V = dx, dy, dz, dRx, dRy, dRz

                ind=find(qp>deg2rad(320));
                if ~isempty(ind)
                    qp(ind)=deg2rad(320);
                end
                ind=find(qp<-deg2rad(320));
                if ~isempty(ind)
                    qp(ind)=-deg2rad(320);
                end
                q = self.q0 + (1/fps)*qp;

                self.r2.model.animate(q');

                % compute new camera pose
                Tcam = self.r2.model.fkine(q);
                % update camera pose
                self.cam.T = Tcam;

                drawnow
                pause (1/fps)
                if ~isempty(200) && (ksteps > 50)
                    break;
                end
                self.q0 = q; % update current joint position

            end
        end

        function dobotMove(self,position)
            p2 = transl(position);
            [x, qMatrix] = self.r1.trajGen.getQForLineTraj(p2);
            self.r1.trajGen.animateQ(qMatrix);
            self.servoingOn = 0;

        end
    end
end