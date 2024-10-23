InitialiseRos();
basePosImg = GetBasePos();
cam = webcam;  % Connect to the default webcam
CalibrateDobotMagician(10, cam); %idk what I'm not actually passing out so yeah lol.
VisualServoing(cam, basePosImg);
