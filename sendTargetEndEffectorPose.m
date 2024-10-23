function sendTargetEndEffectorPose(position, rotation)
    % SENDTARGETENDEFFECTORPOSE - Sends the desired end-effector pose 
    % (position and rotation) to the /dobot_magician/target_end_effector_pose topic.
    %
    % Inputs:
    %   position - A 1x3 vector [X, Y, Z] specifying the target position.
    %   rotation - A 1x3 vector [roll, pitch, yaw] specifying the target rotation in Euler angles.

    % Create a ROS Publisher and message for the target end-effector pose
    [targetEndEffectorPub, targetEndEffectorMsg] = ...
        rospublisher('/dobot_magician/target_end_effector_pose');

    % Set the target position in the message
    targetEndEffectorMsg.Position.X = position(1);
    targetEndEffectorMsg.Position.Y = position(2);
    targetEndEffectorMsg.Position.Z = position(3);

    % Convert the Euler angles to a quaternion
    quaternion = eul2quat(rotation);

    % Set the target orientation (as quaternion) in the message
    targetEndEffectorMsg.Orientation.W = quaternion(1);
    targetEndEffectorMsg.Orientation.X = quaternion(2);
    targetEndEffectorMsg.Orientation.Y = quaternion(3);
    targetEndEffectorMsg.Orientation.Z = quaternion(4);

    % Send the message to the topic
    send(targetEndEffectorPub, targetEndEffectorMsg);
end
