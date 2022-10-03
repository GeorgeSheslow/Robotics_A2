function qMatrix = RMRCTraj(robot, x, theta)

    epsilon = 0.0001;      % Threshold value for manipulability/Damped Least Squares
    W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
    steps = size(x,2);
    % 1.2) Allocate array data
    m = zeros(steps,1);             % Array for Measure of Manipulability
    qMatrix = zeros(steps,6);       % Array for joint anglesR
    qdot = zeros(steps,6);          % Array for joint velocities
%     positionError = zeros(3,steps); % For plotting trajectory error
%     angleError = zeros(3,steps);    % For plotting trajectory error
    deltaT = 0.002;      % Control frequency

    T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
    q0 = zeros(1,6);                                                            % Initial guess for joint angles
    qMatrix(1,:) = robot.model.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint


    % 1.4) Track the trajectory with RMRC
    for i = 1:steps-1
        T = robot.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
        deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
        Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
        S = Rdot*Ra';                                                           % Skew symmetric!
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
        deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
        xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
        J = robot.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
        qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
        for j = 1:6                                                             % Loop through joints 1 to 6
            if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
            elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0; % Stop the motor
            end
        end
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
%         positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
%         angleError(:,i) = deltaTheta;                                           % For plotting
    end
end