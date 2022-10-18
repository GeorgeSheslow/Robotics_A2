
function visualServoingTest(robot1, robot2)
qr1 = robot1.model.fkine(robot1.model.getpos());
qr1 = qr1(1:3,4); %dobot end effector location

P=[qr1(1),qr1(1),qr1(1),qr1(1);
qr1(2)-0.025,qr1(2)+0.025,qr1(2)+0.025,qr1(2)-0.025;
qr1(3)+0.05,qr1(3)+0.05,qr1(3),qr1(3)];

% Image Target (target points in 2D image plane)
pStar = [662 362 362 662; 362 362 662 662];

% Add camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'IRB120camera');

% Frame rate
fps = 25;

% Controller gain
lambda = 0.3;

% IBVS depth
depth = 1.8;

q0 = robot2.model.getpos(); %irb end effector location

Tc0 = robot2.model.fkine(q0);%*transl(0.05,0,0)*troty(180,"deg");
% robot2.model.animate(q0');
drawnow

cam.T = Tc0;

cam.plot_camera('Tcam',Tc0, 'label','scale',0.05);
plot_sphere(P,0.005,'b')
lighting gouraud
light

p = cam.plot(P, 'Tcam', Tc0);

cam.clf()
cam.plot(pStar, '*');
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o');
pause(2)
cam.hold(true);
cam.plot(P); %displays initial view

%Initialise display arrays
vel_p = [];
uv_p = [];
history = [];
ksteps = 0;
 while true
        ksteps = ksteps + 1;
        
        % compute the view of the camera
        uv = cam.plot(P);
        
        % compute image plane error as a column
        e = pStar-uv;   % feature error
        e = e(:);
        Zest = [];
        
        % compute the Jacobian
        if isempty(depth)
            % exact depth from simulation (not possible in practice)
            pt = homtrans(inv(Tcam), P);
            J = cam.visjac_p(uv, pt(3,:) );
        elseif ~isempty(Zest)
            J = cam.visjac_p(uv, Zest);
        else
            J = cam.visjac_p(uv, depth );
        end

        % compute the velocity of camera in camera frame
        try
            v = lambda * pinv(J) * e;
        catch
            status = -1;
            return
        end
        fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

        %compute robot's Jacobian and inverse
        J2 = robot2.model.jacobn(q0);
        Jinv = pinv(J2);
        % get joint velocities
        qp = Jinv*v;

         
         %Maximum angular velocity cannot exceed 180 degrees/s
         ind=find(qp>pi);
         if ~isempty(ind)
             qp(ind)=pi;
         end
         ind=find(qp<-pi);
         if ~isempty(ind)
             qp(ind)=-pi;
         end

        %Update joints 
        q = q0 + ((1/fps)*qp)'
        robot2.model.animate(q);

        %Get camera location
        Tc = robot2.model.fkine(q);%*transl(0.05,0,0)*troty(180,"deg")
        cam.T = Tc;

        drawnow
        
        % update the history variables
        hist.uv = uv(:);
        vel = v;
        hist.vel = vel;
        hist.e = e;
        hist.en = norm(e);
        hist.jcond = cond(J);
        hist.Tcam = Tc;
        hist.vel_p = vel;
        hist.uv_p = uv;
        hist.qp = qp;
        hist.q = q;

        history = [history hist];

         pause(1/fps)

        if ~isempty(200) && (ksteps > 200)
            break;
        end
        
        %update current joint position
        q0 = q;
 end %loop finishes


end
