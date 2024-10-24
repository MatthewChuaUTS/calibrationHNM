function currentJointState = GetCurrentJointState()
    jointStateSubscriber = rossubscriber('/dobot_magician/joint_states'); % Create a ROS Subscriber to the topic joint_states
    pause(2); % Allow some time for a message to appear
    currentJointState = jointStateSubscriber.LatestMessage.Position % Get the latest message
end

    % jointStateSubscriber = rossubscriber('/dobot_magician/joint_states'); % Create a ROS Subscriber to the topic joint_states
    % pause(2); % Allow some time for a message to appear
    % currentJointState = jointStateSubscriber.LatestMessage.Position % Get the latest message

