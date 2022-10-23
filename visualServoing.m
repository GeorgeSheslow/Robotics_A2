clear
clf
clc
%%

% 3D points (1,-0.05, 0.25), (1,0.05,0.25), (1,0.05,0.15), (1,-0.05,0.15)
P = [1, 1, 1, 1;
    -0.1, 0.1, 0.1, -0.1;
    0.35, 0.35, 0.15, 0.15];

% q0 = [pi/2;-pi/4;0;0;0;0];
q0 = [0;-pi/2;0;0;pi;0];
% r = UR3();
r = IRB120();

cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'IRBCamera');

fps = 25;
lambda = 0.8;
depth = 1;

pStar = bsxfun(@plus, 200*[-0.5 -0.5 0.5 0.5; -0.5 0.5 0.5 -0.5], cam.pp');
%%
% initial camera position
Tc0 = r.model.fkine(q0);
% Tcam = Tc0 % initial camera/robot pose

% external view of points and camera
r.model.animate(q0');
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
%%
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
    J2 = r.model.jacobn(q0); % jacobian for robot in pose q0
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
         test = (1/fps)*qp;
    q = q0 + test;

    r.model.animate(q');

    % compute new camera pose
    Tcam = r.model.fkine(q);
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

