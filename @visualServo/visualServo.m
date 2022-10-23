classdef visualServo < handle
    properties (Access = public)
        r1
        r2
        cam
    end
    methods
        function self = visualServo(r1, r2)
            self.r1 = r1;
            self.r2 = r2;
            self.cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
                'resolution', [1024 1024], 'centre', [512 512],'name', 'IRBCamera');
        end
        function vs(self)
%             q0 = [0;0;0;0;0;0];
           
            qr1 = self.r1.model.fkine(self.r1.model.getpos());
            qr1 = qr1(1:3,4);
            P=[qr1(1),qr1(1),qr1(1),qr1(1);
                qr1(2)-0.1,qr1(2)+0.1,qr1(2)+0.1,qr1(2)-0.1;
                qr1(3)+0.35,qr1(3)+0.35,qr1(3)+0.15,qr1(3)+0.15];


            fps = 25;
            lambda = 0.8;
            depth = 1;

            pStar = bsxfun(@plus, 200*[-1 -1 1 1; -1 1 1 -1], self.cam.pp');
            
            q0 = (self.r2.model.getpos())';
            Tc0 = self.r2.model.fkine(q0);
            hold on
%             self.r2.model.animate(q0');
            pose = self.r1.model.fkine(self.r1.model.getpos())*transl(0.02,0,0.2);
            paper = Paper(pose*troty(pi/2));
            drawnow
            

            self.cam.T = Tc0; % set camera to initial pose
            plot_sphere(P, 0.05, 'b')
            self.cam.plot_camera('Tcam',Tc0, 'label','scale',0.05);
            
            lighting gouraud
            light

            %%
            p = self.cam.plot(P, 'Tcam', Tc0);
            % show ref location, wanted view when Tc = Tct_star
            self.cam.clf()
            self.cam.plot(pStar, '*'); % create the camera view
            self.cam.hold(true);
            self.cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
            pause(2)
            self.cam.hold(true);

            self.cam.plot(P); % show initial view

            %initialise display arrays
            vel_p = [];
            uv_p = [];
            history = [];
            %%
            ksteps = 0;
            while true
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
                J2 = self.r2.model.jacobn(q0); % jacobian for robot in pose q0
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
                q = q0 + (1/fps)*qp;

                self.r2.model.animate(q');

                % compute new camera pose
                Tcam = self.r2.model.fkine(q);
                % update camera pose
                self.cam.T = Tcam;

                drawnow

                hist.uv = uv(:);
                vel = v;
                hist.vel = vel;
                hist.e = e;
                hist.en = norm(e);
                hist.jcond = cond(J);
                hist.Tcam = Tcam;
                hist.vel_p = vel;
                hist.uv_p = uv;
                hist.q = q;

                history = [history hist];

                pause (1/fps)
                if ~isempty(200) && (ksteps > 200)
                    break;
                end
                q0 = q; % update current joint position

            end
        end
    end
end