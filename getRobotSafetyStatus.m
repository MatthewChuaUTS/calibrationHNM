function currentSafetyStatus = getRobotSafetyStatus(rossubscriber)
    safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
    pause(2);
    currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data;
    disp(currentSafetyStatus);
end

safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status','std_msgs/UInt8');
receive(safetyStatusSubscriber, 10);
currentSafetyStatus = currentMessage.Data;
disp(currentSafetyStatus);

currentMessage = receive(safetyStatusSubscriber, 20);


%  try >> rostopic echo /dobot_magician/safety_status
% no output >> dobott not publishing to the topic
% try >> rosnode list