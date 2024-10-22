%% Initialise ROS
runInit = 0;
if runInit == 1
    rosinit('192.168.27.1'); % If unsure, please ask a tutor
    input('press enter to continue');
end

function SetInitialiseDobot()
    [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
    safetyStateMsg.Data = 2;
    send(safetyStatePublisher,safetyStateMsg);
end

% Get the current joint state


% Get current end effector pose
function Transform = getCurrentEndEffectorPose(rossubscriber)
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
    Transform = transl(currentEndEffectorPosition) * trotx(roll) * troty(pitch) * trotz(yaw);
end




%% IR


[client, goal] = rosactionclient('/ur/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 5; % This is how many seconds the movement will take 
openService.call();
% pause(5);
% % closeService.call();

input('Ensure Robot is home, then press enter to continue');

Qmatrix = [ ...
    deg2rad(0), deg2rad(-90), deg2rad(50), deg2rad(-100), deg2rad(-90), deg2rad(0); ... % hover
    deg2rad(0), deg2rad(-90), deg2rad(100), deg2rad(-100), deg2rad(-90), deg2rad(0); ... % down
    deg2rad(0), deg2rad(-90), deg2rad(50), deg2rad(-100), deg2rad(-90), deg2rad(0); ... % up
    deg2rad(76), deg2rad(-137), deg2rad(134), deg2rad(-143), deg2rad(-73), deg2rad(0); ... % clean
    deg2rad(76), deg2rad(-137), deg2rad(134), deg2rad(-143), deg2rad(-73), deg2rad(360); ... % clean
    deg2rad(0), deg2rad(-90), deg2rad(50), deg2rad(-100), deg2rad(-90), deg2rad(0); ... % hover
    deg2rad(0), deg2rad(-90), deg2rad(100), deg2rad(-100), deg2rad(-90), deg2rad(0); ... % down
    deg2rad(0), deg2rad(-90), deg2rad(50), deg2rad(-100), deg2rad(-90), deg2rad(0); ... % hover
    deg2rad(88), deg2rad(-64), deg2rad(9), deg2rad(-55), deg2rad(-86), deg2rad(271); ... % hover above plate
    deg2rad(88), deg2rad(-60), deg2rad(24), deg2rad(-61), deg2rad(-87), deg2rad(271); ... % go down to plate
    deg2rad(88), deg2rad(-64), deg2rad(9), deg2rad(-55), deg2rad(-86), deg2rad(271); ... % hover above plate
    deg2rad(139), deg2rad(-91), deg2rad(60), deg2rad(-57), deg2rad(-88), deg2rad(319); ... % hover above drop
    deg2rad(140), deg2rad(-91), deg2rad(90), deg2rad(-87), deg2rad(-89), deg2rad(319); ... % drop
    deg2rad(139), deg2rad(-91), deg2rad(60), deg2rad(-57), deg2rad(-88), deg2rad(319); ... % hover above drop
    deg2rad(0), deg2rad(-90), deg2rad(50), deg2rad(-100), deg2rad(-90), deg2rad(0) ... % hover
];

openEndMatrix = [2, 8, 14];
closeEndMatrix = [3, 11];

% Loop through each row of Qmatrix and animate the robot
for i = 1:size(Qmatrix, 1)
    moveClaw = "don't move";  % Default value
    if ismember(i, openEndMatrix)
        moveClaw = "open after movement";
    elseif ismember(i, closeEndMatrix)
        moveClaw = "close after movement";
    end
    % Pass the moveClaw and services to moveRobot function
    moveRobot(Qmatrix(i, :), moveClaw, jointStateSubscriber, goal, client, bufferSeconds, durationSeconds, openService, closeService);
    input("Press Enter to continue...");
end

function moveRobot(Q, moveClaw, jointStateSubscriber, goal, client, bufferSeconds, durationSeconds, openService, closeService)
    % Get the current joint state and reorder joints
    currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)';
    currentJointState_123456 = [currentJointState_321456(3:-1:1), currentJointState_321456(4:6)];

    % Create start and end trajectory points
    startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    startJointSend.Positions = currentJointState_123456;
    startJointSend.TimeFromStart = rosduration(0);

    endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    endJointSend.Positions = Q;
    endJointSend.TimeFromStart = rosduration(durationSeconds);

    % Set trajectory points and header stamp
    goal.Trajectory.Points = [startJointSend; endJointSend];
    goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

    % Send the trajectory goal
    sendGoal(client, goal);

    % Handle claw movement based on the moveClaw value
    if moveClaw == "open after movement"
        openService.call();
    elseif moveClaw == "close after movement"
        closeService.call();
    end
end
