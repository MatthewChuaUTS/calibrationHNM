function WAIT_NO_NO_NO_STOP_THE_ROBOT()
    [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
    safetyStateMsg.Data = 3;
    send(safetyStatePublisher,safetyStateMsg);
end