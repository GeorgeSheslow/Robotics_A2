function [qMatrix] = DobotRMRC(robot, desiredPos, steps)
 

    epsilon = 0.01;      % Threshold value for manipulability/Damped Least Squares
    W = diag([1 1 1 0.1]);    % Weighting matrix for the velocity vector
    numJoints = 4;
    % 1.2) Allocate array data
    m = zeros(steps,1);             % Array for Measure of Manipulability
    qMatrix = zeros(steps,numJoints);       % Array for joint anglesR
    qdot = zeros(steps,numJoints);          % Array for joint velocities
    deltaT = 0.002;      % Control frequency
    
    currentPose = robot.model.fkine(robot.model.getpos());

    s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
    for i=1:steps
        x(1,i) = (1-s(i))*currentPose(1,4) + s(i)*desiredPos(1); % Points in x
        x(2,i) = (1-s(i))*currentPose(2,4) + s(i)*desiredPos(2); % Points in y
        x(3,i) = (1-s(i))*currentPose(3,4) + s(i)*desiredPos(3); % Points in z
        theta(1,i) = 0;                 % Roll angle 
        theta(2,i) = 0;            % Pitch angle
        theta(3,i) = 0;                 % Yaw angle
    end
    
    % TODO:
    % RMRC options: p-p straight line trajectory gen, text trajectory,
    % parabola in Z between p-p on plane
    
    T = [rpy2r(0,0,0) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
    q0 = robot.model.getpos();                                                            % Initial guess for joint angles
    qMatrix(1,:) = robot.model.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint


    % 1.4) Track the trajectory with RMRC
    for i = 1:steps-1
        T = robot.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
        deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
        Rd = rpy2r(0,theta(2,i+1),0);                     % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
        Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
        S = Rdot*Ra';                                                           % Skew symmetric!
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(1,3);]; %[S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
%         deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
        xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
        JBig = robot.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
        J = JBig(1:3,:);
        J(4,:) = JBig(5,:);
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(numJoints))*J';                                   % DLS Inverse
        qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
        for j = 1:numJoints                                                            % Loop through joints 1 to 6
            if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
            elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0; % Stop the motor
            end
        end
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
    end
    qMatrix = real(qMatrix);
end