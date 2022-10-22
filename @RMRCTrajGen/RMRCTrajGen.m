classdef RMRCTrajGen
    properties (Access = public)
        robot
        numJoints
        steps = 10; % default value
        
        epsilon = 0.0001;      % Threshold value for manipulability/Damped Least Squares
        deltaT = 0.002;      % Control frequency
        W;
        m;
        qdot;
        toolOffset = [0 0 0];
    end
    methods (Access = public)
        function self = RMRCTrajGen(robot)
            self.robot = robot;
            self.numJoints = self.robot.n;
            
            if self.numJoints == 4
                self.W = diag([1 1 1 0.1]);    % Weighting matrix for the velocity vector
            elseif self.numJoints == 6
                self.W = diag([1 1 1 0.1 0.1 0.1]);
            else
                disp('Weighting matrix not set');
            end
        end
        function [x, q] = getQForLineTraj(self, point)
            currentPoint = self.robot.fkine(self.robot.getpos());
            [x, theta] = self.lineTraj(currentPoint(1:3,4)',point(1:3,4)');
            q = self.getRMRC(x, theta, self.steps);
        end
        function [x, q] = getQForLineTrajWSteps(self, point, steps)
            currentPoint = self.robot.fkine(self.robot.getpos());
            [x, theta] = self.lineTraj(currentPoint(1:3,4)',point(1:3,4)');
            q = self.getRMRC(x, theta,steps);
        end
        function [x, q] = getQForTraj(self,traj)
            count = 1;
            for i = 1:size(traj,2)
                if i < size(traj,2) && (norm(traj(1:3,i) - traj(1:3,i+1)) > 0.01)
                    [x1, theta1] = self.lineTraj(traj(1:3,i),traj(1:3,i+1));
                    for j = 1:size(x1,2)
                        x(1:3,count) = x1(1:3,j);
                        theta(1,count) = 0;                 % Roll angle 
                        theta(2,count) = 0;                 % Pitch angle
                        theta(3,count) = 0;                 % Yaw angle
                        count = count +1;
                    end
                else
                    x(1:3,count) = traj(:,i)';
                    theta(1,count) = 0;                 % Roll angle 
                    theta(2,count) = 0;                 % Pitch angle
                    theta(3,count) = 0;                 % Yaw angle
                    count = count +1;
                end
            end
            q = self.getRMRC(x,theta,count-1);
        end
        function [x, q] = getQForZArcTraj(self, point)
            currentPoint = self.robot.fkine(self.robot.getpos());
            [x, theta] = self.zArcTraj(currentPoint(1:3,4)',point(1:3,4)');
            q = self.getRMRC(x,theta,self.steps);
        end
        function animateQ(self, qMatrix)
            for j = 1:size(qMatrix,1)
                newQ = qMatrix(j,:);
                self.robot.animate(newQ);
                drawnow();
                hold on
                pause(0.1);
            end 
        end
        function animateQWObj(self, qMatrix, object)
            for j = 1:size(qMatrix,1)
                newQ = qMatrix(j,:);
                self.robot.animate(newQ);
                % move EE with object attached to it 
                pose  = self.robot.fkine(self.robot.getpos());
                pose = pose * transl(self.toolOffset);
                object.MoveObj(pose);
                drawnow();
                hold on
                pause(0.1);
            end 
        end
        function plotQ(self, qMatrix)
            for j = 1:size(qMatrix,1)
                newQ = qMatrix(j,:);
                self.robot.animate(newQ);
                drawnow();
                hold on
                pos = self.robot.fkine(self.robot.getpos());
                plot3(pos(1,4)+self.toolOffset(1),pos(2,4)+self.toolOffset(2),pos(3,4)+self.toolOffset(3),'r.');
                pause(0.1);
            end 
        end
        function plotQAndTraj(self, qMatrix, x)
            for j = 1:size(qMatrix,1)
                newQ = qMatrix(j,:);
                self.robot.animate(newQ);
                drawnow();
                hold on
                pos = self.robot.fkine(self.robot.getpos());
                plot3(pos(1,4)+self.toolOffset(1),pos(2,4)+self.toolOffset(2),pos(3,4)+self.toolOffset(3),'r.');
                plot3(x(1,4)+self.toolOffset(1),x(2,4)+self.toolOffset(2),x(3,4)+self.toolOffset(3),'k.');
                pause(0.1);
            end 
        end
    end
    methods (Access = private)
        function [x, theta] = lineTraj(self,p1, p2)
            % TODO: preallocate x, theta array
            s = lspb(0,1,self.steps);                % Trapezoidal trajectory scalar
            for i=1:self.steps
                x(1,i) = (1-s(i))*p1(1) + s(i)*p2(1); % Points in x
                x(2,i) = (1-s(i))*p1(2) + s(i)*p2(2); % Points in y
                x(3,i) = (1-s(i))*p1(3) + s(i)*p2(3); % Points in z
                theta(1,i) = 0;                 % Roll angle 
                theta(2,i) = 0;                 % Pitch angle
                theta(3,i) = 0;                 % Yaw angle
            end
        end
        function [x, theta] = zArcTraj(self, p1, p2)
            dis = pi / self.steps;
            s = lspb(0,1,self.steps);                % Trapezoidal trajectory scalar
            for i=1:self.steps
                x(1,i) = (1-s(i))*p1(1) + s(i)*p2(1); % Points in x
                x(2,i) = (1-s(i))*p1(2) + s(i)*p2(2); % Points in y
                r = pdist([p1; p2]) / 2;
                x(3,i) = p1(3) + r * sin(i * dis); % Points in z
                theta(1,i) = 0;                 % Roll angle 
                theta(2,i) = 0;                 % Pitch angle
                theta(3,i) = 0;                 % Yaw angle
            end
        end
        function qMatrix = getRMRC(self, x, theta, steps)
            self.m = zeros(steps,1);             % Array for Measure of Manipulability
            self.qdot = zeros(steps,self.numJoints);          % Array for joint velocities
            T = [rpy2r(0,0,0) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            q0 = self.robot.getpos();                                                            % Initial guess for joint angles
            qMatrix = zeros(steps,self.numJoints);       % Array for joint anglesR
            qMatrix(1,:) = self.robot.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint

            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                T = self.robot.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(0,theta(2,i+1),0);                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/self.deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
%                 deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/self.deltaT)*deltaX;
                jacob = self.robot.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state

                if self.numJoints == 4
                    angular_velocity = S(1,3); %[S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                    J = jacob(1:3,:);
                    J(4,:) = jacob(5,:);
                elseif self.numJoints == 6
                    angular_velocity = [S(3,2);S(1,3);S(2,1)];
                    J = jacob;
                end
                xdot = self.W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                self.m(i) = sqrt(det(J*J'));
                if self.m(i) < self.epsilon  % If manipulability is less than given threshold
                    lambda = (1 - self.m(i)/self.epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(self.numJoints))*J';                                   % DLS Inverse
                self.qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:self.numJoints                                                            % Loop through joints 1 to 6
                    if qMatrix(i,j) + self.deltaT*self.qdot(i,j) < self.robot.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        self.qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + self.deltaT*self.qdot(i,j) > self.robot.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        self.qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + self.deltaT*self.qdot(i,:);                         	% Update next joint state based on joint velocities
            end
            qMatrix = real(qMatrix);
        end
    end
end