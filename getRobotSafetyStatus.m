function currentSafetyStatus = getRobotSafetyStatus(rossubscriber)
    safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
    pause(2);
    currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data;
end