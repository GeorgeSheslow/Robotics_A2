close all
clear
clc
pStar = [400 200 200 400; 200 200 400 400];

dobot = DobotMagician(transl(-1,0,0));
dobotQ = deg2rad([0 11 79 89]);
dobot.model.animate(dobotQ);

IRB = IRB120(transl(0,0,0));
IRBQ = deg2rad([180 -37 58 0 90 0]);
IRB.model.animate(IRBQ);
hold on
% attach paper to EE
pose = dobot.model.fkine(dobot.model.getpos())*transl(0.02,0,0);
paper = Paper(pose*troty(pi/2));

% 3D points for sign
P=[0,0,0,0;
-0.07,0.07,0.07,-0.07;
 0.07,0.07,-0.07,-0.07];
% mod to EE frame
for i = 1:size(P,2)
    temp = transl(P(:,i)');
    temp = pose * temp;
    P(:,i) = temp(1:3,4)';
end
hold on
plot_sphere(P, 0.03, 'b');
lighting gouraud
light
axis equal;
%%

cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'IRBcamera');

% frame rate
fps = 25;
%Define values
%gain of the controler
lambda = 0.2;
%depth of the IBVS
depth = mean (P(1,:));

q0 = IRBQ';
Tc0= IRB.model.fkine(q0');
% plot camera and points
cam.T = Tc0;
cam.plot_camera('Tcam',Tc0, 'label','scale',0.15);

%% 1.3 Initialise Simulation (Display in Image view)

%Project points to the image
p = cam.plot(P, 'Tcam', Tc0);

%camera view and plotting
cam.clf()
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
pause(2)
cam.hold(true);
cam.plot(P);    % show initial view

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
            disp("velocity computation failed");
            status = -1;
            return
        end
        fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

        %compute robot's Jacobian and inverse
        J2 = IRB.model.jacobn(q0);
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
        q = q0 + (1/fps)*qp;
        IRB.model.animate(q');

        %Get camera location
        Tc = IRB.model.fkine(q);
        cam.T = Tc;

        drawnow

         pause(1/fps)

        if ~isempty(200) && (ksteps > 200)
            break;
        end
        
        %update current joint position
        q0 = q;
 end %loop finishes


