%% PROJECT CHECKLIST

% - [ ] test initialising ROS
% - [ ] test GetBasePos
% - [ ] add calibration code
% - [ ] manually record calibration endeffector positions
% - [ ] populate moveArray with those positions
% - [ ] test the calibration
% - [ ] test sendTargetEndEffectorPose() with a valid position
% - [ ] check what happens if you try to send an invalid position (plz don't break)
% - [ ] test the VisualServoing() function.

%% POSSIBLE THINGS THAT MAY GO WRONG

% >>> nothing (PLEASE I NEED THIS)
% >>> the movement functions can't access something that I should have made global
% >>> the calibration doesn't work
% >>> we don't actually end up using calibration like the tutors want
% >>> the visual servoing just doesn't work (very not good, worst case scenario)


%% ACTUAL CODE

InitialiseRos(1);
basePosImg = GetBasePos();
cam = webcam;  % Connect to the default webcam
CalibrateDobotMagician(10, cam); %idk what I'm not actually passing out so yeah lol.
input('ensure the command window has the safety stop function ready, then press enter');
% virtual estop function -> WAIT_NO_NO_NO_STOP_THE_ROBOT();
VisualServoing(cam, basePosImg);

