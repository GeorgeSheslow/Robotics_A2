
% rob1 = DobotMagician();
% rob1.model.base = transl(1,0,0)*trotz(pi);
% rob2 = IRB120();
% q0 = [0;-pi/2;0;0;pi;0];
% visualServo(rob1, rob2, q0)
%%
classdef visualServo < handle
    properties

    end
    methods
        function vs(r1, r2)
            q0 = [0;0;0;0;pi;0];
            %r1.model.animate([0,pi/4,pi/4,pi/2]);
            qr1 = r1.model.fkine(r1.model.getpos());
            qr1 = qr1(1:3,4);
            P=[qr1(1),qr1(1),qr1(1),qr1(1);
                qr1(2)-0.1,qr1(2)+0.1,qr1(2)+0.1,qr1(2)-0.1;
                qr1(3)+0.25,qr1(3)+0.25,qr1(3)+0.05,qr1(3)+0.05];

            cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
                'resolution', [1024 1024], 'centre', [512 512],'name', 'IRBCamera');

            fps = 25;
            lambda = 0.8;
            depth = 1;

            pStar = bsxfun(@plus, 200*[-0.5 -0.5 0.5 0.5; -0.5 0.5 0.5 -0.5], cam.pp');
            %%
            q0 = q0+r2.model.getpos();
            Tc0 = r2.model.fkine(q0);
            r2.model.animate(q0');
            drawnow

            cam.T = Tc0; % set camera to initial pose
            cam.plot_camera(P, 'label','scale',0.05);
            plot_sphere(P, 0.05, 'b')
            lighting gouraud
            light
            %%
            p = cam.plot(P, 'Tcam', Tc0);
            %%
            % show ref location, wanted view when Tc = Tct_star
            cam.clf()
            cam.plot(pStar, '*'); % create the camera view
            cam.hold(true);
            cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
            pause(2)
            cam.hold(true);

            cam.plot(P); % show initial view

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
                uv = cam.plot(P);

                % compute image plane error as a column
                e = uv - pStar; % feature error
                e = e(:);

                % compute Jacobian
                if isempty(depth)
                    % exact depth from simulation
                    pt = homtrans(inv(Tcam), P);
                    J = cam.visjac_p(uv, pt(3,:));
                elseif ~isempty(Zest)
                    J = cam.visjac_p(uv, Zest);
                else
                    J = cam.visjac_p(uv, depth);
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
                J2 = r2.model.jacobn(q0); % jacobian for robot in pose q0
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

                r2.model.animate(q');

                % compute new camera pose
                Tcam = r2.model.fkine(q);
                % update camera pose
                cam.T = Tcam;

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
                if ~isempty(100) && (ksteps > 100)
                    break;
                end
                q0 = q; % update current joint position

            end
        end
    end
end