rosinit()

%% test joint states command topic
jointTarget = [0.1,0.4,0.3,0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

%% test end effector command topic

endEffectorPosition = [0.2,0,0.02];
endEffectorRotation = [0,0,0];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub,targetEndEffectorMsg);

%% Calculate Drawing point trajectories
write = TextToTraj("DOBOT","text");
x_Write = write.GetTraj();

%% RMRC Calculations
robot = DobotMagician(transl(0,0,0));
[x, qMatrix] = robot.trajGen.getQForTraj(x_Write);
robot.trajGen.plotQAndTraj(qMatrix, x)

%% Simulation
close all
hold on;
for i = 1:size(x_Write,2)
    plot3(x_Write(1,i),x_Write(2,i),x_Write(3,i),'k.');
    hold on;
end

%% Actuate real robot
robot = DobotMagician(transl(0,0,0));
[x, qMatrix] = robot.trajGen.getQForTraj(x_Write);
for i = 1:size(qMatrix,1)
    joints = qMatrix(i,:);
    sendJointCommands(joints);
    pause(0.2);
end
%% Actuate real robot
for i = 1:size(x_Write,2)
    position = x_Write(1:3,i)';
    sendEECommands(position);
    pause(0.6);
end

%%

endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses'); % Create a ROS Subscriber to the topic end_effector_poses
pause(2); %Allow some time for MATLAB to start the subscriber
currentEndEffectorPoseMsg = endEffectorPoseSubscriber.LatestMessage;
% Extract the position of the end effector from the received message
currentEndEffectorPosition = [currentEndEffectorPoseMsg.Pose.Position.X,
                              currentEndEffectorPoseMsg.Pose.Position.Y,
                              currentEndEffectorPoseMsg.Pose.Position.Z];
% Extract the orientation of the end effector
currentEndEffectorQuat = [currentEndEffectorPoseMsg.Pose.Orientation.W,
                          currentEndEffectorPoseMsg.Pose.Orientation.X,
                          currentEndEffectorPoseMsg.Pose.Orientation.Y,
                          currentEndEffectorPoseMsg.Pose.Orientation.Z];
% Convert from quaternion to euler
[roll,pitch,yaw] = quat2eul(currentEndEffectorQuat);

%%

function sendIt(position)
    endEffectorPosition = position;
    endEffectorRotation = [0,0,0];
    
    [targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
    
    targetEndEffectorMsg.Position.X = endEffectorPosition(1);
    targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
    targetEndEffectorMsg.Position.Z = endEffectorPosition(3);
    
    qua = eul2quat(endEffectorRotation);
    targetEndEffectorMsg.Orientation.W = qua(1);
    targetEndEffectorMsg.Orientation.X = qua(2);
    targetEndEffectorMsg.Orientation.Y = qua(3);
    targetEndEffectorMsg.Orientation.Z = qua(4);
    
    send(targetEndEffectorPub,targetEndEffectorMsg);
end

%%
function sendJoints(targetJointStates)
    
    jointTarget = targetJointStates; % Remember that the Dobot has 4 joints by default.
    
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    
    send(targetJointTrajPub,targetJointTrajMsg);

end

