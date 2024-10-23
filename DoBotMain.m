% Initialise ROS
runInit = 0;
if runInit == 1
    rosinit('192.168.27.1'); % If unsure, please ask a tutor
    input('press enter to continue');
end

% SetInitialiseDobot
[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher,safetyStateMsg);

cam = webcam;  % Connect to the default webcam
CalibrateDobotMagician(); %idk what I'm not actually passing out so yeah lol.
basePosImg = GetBasePos();
VisualServoing();
