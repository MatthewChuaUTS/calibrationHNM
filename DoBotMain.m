InitialiseRos();
basePosImg = GetBasePos();
cam = webcam;  % Connect to the default webcam
CalibrateDobotMagician(10, cam); %idk what I'm not actually passing out so yeah lol.
input('ensure the command window has the safety stop function ready, then press enter');
% virtual estop function -> WAIT_NO_NO_NO_STOP_THE_ROBOT();
VisualServoing(cam, basePosImg);

