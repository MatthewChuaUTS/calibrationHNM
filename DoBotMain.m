%% PROJECT CHECKLIST

% - [X] test initialising ROS
% - [X] test GetBasePos
% - [X] add calibration code
% - [X] manually record calibration endeffector positions
% - [X] populate moveArray with those positions
% - [X] test the calibration
% - [X] test sendTargetEndEffectorPose() with a valid position
% - [X] check what happens if you try to send an invalid position (plz don't break)
% - [?] test the VisualServoing() function.

%% POSSIBLE THINGS THAT MAY GO WRONG

% >>> nothing (PLEASE I NEED THIS)
% >>> the movement functions can't access something that I should have made global
% >>> the calibration doesn't work
% >>> we don't actually end up using calibration like the tutors want
% >>> the visual servoing just doesn't work (very not good, worst case scenario)


%% ACTUAL CODE
% 
% InitialiseRos(0);
% InitialiseDobot();
% input("press enter once finished calibrating");
cam = webcam;  % Connect to the default webcam
[t_ce, cameraParams] = newPhotoFinalProject(cam); %idk what I'm not actually passing out so yeah lol.
disp("TCE");
disp(t_ce);
disp('reproj errors');
disp(cameraParams.ReprojectionErrors);
basePosImg = GetBasePos(cam);
% input('ensure the command window has the safety stop function ready, then press enter');
disp('done, will pause for a sec');
pause(1);
T_base_world = eye(4);

% mattsVisualServoing(cam, basePosImg, t_ce, cameraParams);
Controller_Part2(cam, t_ce, T_base_world, cameraParams);
% Controller_Part2(cam, T_camera_endeffector, T_base_world, cameraParams)

