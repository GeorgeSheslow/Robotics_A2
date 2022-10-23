

% 3D points (1,-0.05, 0.25), (1,0.05,0.25), (1,0.05,0.15), (1,-0.05,0.15)
P = [1, 1, 1, 1;
    -0.05, 0.05, 0.05, -0.05;
    0.25, 0.25, 0.15, 0.15];

q0 = [0; -pi/2; 0; 0; 0; 0];

r = IRB120();

cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'IRBCamera');

fps = 25;
lambda = 0.8;
depth = 1;

pStar = bsxfun(@plus, 200*[-1 -1 1 1; -1 1 1 -1], cam.pp');

% initial camera position
Tc0 = r.model.fkine(q0)*trotz(pi/2)*troty(-pi);
Tcam = Tc0 % initial camera/robot pose

cam.T = Tc0; % set camera to initial pose


% show ref location, wanted view when Tc = Tct_star
cam.clf()
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
pause(2)
cam.hold(false);
cam.clf();

cam.plot(P); % show initial view

% external view of points and camera
r.model.animate(q0');
drawnow
cam.plot_camera(P, 'label');
plot_sphere(P, 0.05, 'b')
lighting gouraud
light

%initialise display arrays
vel_p = [];
uv_p = [];
history = [];


ksteps = 0;
while true
    ksteps = ksteps + 1;
    Zest = [];

    % comute the view
    uv = cam.plot(P);
    
    % Optional Depth Estimation
%     if depthest
%         [Zest, Ztrue] = depth_estimator(uv);
%         depth = Zest;
%         hist.Ztrue = Ztrue(:);
%         hist.Zest = Zest(:);
%     end

    % compute image plane error as a column
    e = uv - pStar % feature error
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
    J2 = r.model.jacobn(q0); % jacobian for robot in pose q0
    Jinv = pinv(J2);
    % V = dx, dy, dz, dRx, dRy, dRz
    waypoint = [v(1) v(2) v(3)];
    [x, traj] = r.trajGen.getQForLineTraj(transl(waypoint))
%     qp = Jinv*v; 
%     ind=find(qp>deg2rad(420));
%          if ~isempty(ind)
%              qp(ind)=deg2rad(420);
%          end
%          ind=find(qp<-deg2rad(420));
%          if ~isempty(ind)
%              qp(ind)=-deg2rad(420);
%          end
%     q = q0 +(1/fps)*qp;

%     r.model.animate(q');
%     steps = 2;
%     currentPoint = r.model.fkine(r.model.getpos());
%     nextPoint = trnorm(Tcam*trnorm(delta2tr(v)));
%     x = zeros(3,steps);
%     theta = zeros(3,steps);
%     s = lspb(0,1,steps);
%     for i = 1:steps
%         x(1,i) = (1-s(i))*currentPoint(1,4)' + s(i)*nextPoint(1,4)'; % Points in x
%         x(2,i) = (1-s(i))*currentPoint(2,4)' + s(i)*nextPoint(1,4)'; % Points in y
%         x(3,i) = (1-s(i))*currentPoint(3,4)' + s(i)*nextPoint(1,4)'; % Points in z
%         theta(1,i) = 0;                 % Roll angle
%         theta(2,i) = 0;                 % Pitch angle
%         theta(3,i) = 0;                 % Yaw angle
%     end
%     qMatrix = nan(steps,6);
%     T = [rpy2r(0,0,0) x(:,1);zeros(1,3) 1];
%     q0 = r.model.getpos();
%     qMatrix(1,:) = r.model.ikcon(T,q0);
%     m = zeros(steps,1);
%     qdot = zeros(steps,6);
%     for i = 1:steps-1
%         T = r.model.fkine(qMatrix(i,:));
%         deltaX = x(:,i+1) - T(1:3,4);
%         Rd = rpy2r(0,theta(2,i+1),0);
%         Ra = T(1:3,1:3);
%         Rdot = (1/fps)*(Rd-Ra);
%         S = Rdot*Ra';
%         linear_velocity = (1/fps)*deltaX;
%         jacob = r.model.jacob0(qMatrix(i,:));
%         angular_velocity = [S(3,2);S(1,3);S(2,1)];
%         J = jacob;
%         xdot = diag([1 1 1 0.1 0.1 0.1])*[linear_velocity;angular_velocity];
%         m(i) = sqrt(det(J*J'));
%         invJ = inv(J'*J +lambda *eye(6))*J';
%         qdot(i,:) = (invJ*xdot)';
%         for j = 1:6
%             if qMatrix(i,j) + (1/fps)*qdot(i,j) < r.model.qlim(j,1)
%                 qdot(i,j) = 0; %stop the motor
%             elseif qMatrix(i,j) + (1/fps)*qdot(i,j) > r.model.qlim(j,2)
%                 qdot(i,j) = 0; % stop the motor
%             end
%         end
%         qMatrix(i+1,:) = qMatrix(i,:) + (1/fps)*qdot(i,:);
%     end
%     qMatrix = real(qMatrix);
%     q = qMatrix(2,:);
%     r.model.animate(q);

    % compute new camera pose
    Tcam = r.model.fkine(q)*trotz(pi/2)*troty(-pi);
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
    if ~isempty(200) && (ksteps > 200)
        break;
    end
    q0 = q; % update current joint position
end

