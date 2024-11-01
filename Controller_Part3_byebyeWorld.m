% Load Calibration and Transformation Parameters
load('newPhotoFinalProject.mat');   % Camera calibration parameters
load('T_ce.mat');                   % Camera-to-end-effector transformation matrix

% Define the proportional gain for control
K_p = 0.1;  % Control gain for smooth control

% Initialize the Camera
cam = webcam;  % Replace with RealSense initialization if needed

% Define checkerboard parameters
squareSize = 0.013;  % Size of one square on the checkerboard in mm

while true
    % Capture an image and detect checkerboard position
    img = snapshot(cam);
    [imagePoints, boardSize] = detectCheckerboardPoints(img);
    
    % Check if checkerboard is detected
    if isempty(imagePoints)
        disp("Checkerboard not detected.");
        continue;  % Skip this iteration if no checkerboard is detected
    end
    
    % Generate 3D world coordinates of the checkerboard points
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    
    % Estimate the checkerboard's position in the camera frame
    [rotationMatrix, translationVector] = estimateWorldCameraPose(imagePoints, worldPoints, cameraParams);
    
    % Construct the camera-to-checkerboard transformation (T_cc)
    T_cc = [rotationMatrix, translationVector'; 0, 0, 0, 1];
    
    % Calculate the checkerboard position relative to the robot base
    T_cb = T_ce * T_cc;  % Checkerboard in the base frame
    
    % Extract the checkerboard position in the base frame
    checkerboardPosition_base = T_cb(1:3, 4);
    
    % Get the current end-effector position in the base frame
    currentEndEffectorPosition_base = getCurrentEndEffectorPosition();  % Define this based on Dobot's current setup
    
    % Calculate the position error between the end-effector and the checkerboard
    positionError = checkerboardPosition_base - currentEndEffectorPosition_base;
    
    % Proportional control to generate movement command to reduce the error
    movementCommand = K_p * positionError;
    
    % Send the movement command to the Dobot
    sendMovementCommandToDobot(movementCommand);  % Implement this function to interface with Dobot
    
    % Display feedback information
    fprintf("Current End-Effector Position: (%.2f, %.2f, %.2f)\n", currentEndEffectorPosition_base);
    fprintf("Checkerboard Position: (%.2f, %.2f, %.2f)\n", checkerboardPosition_base);
    fprintf("Position Error: (%.2f, %.2f, %.2f)\n", positionError);
    fprintf("Movement Command: (%.2f, %.2f, %.2f)\n\n", movementCommand);
    
    % Optional: Break condition to exit the loop if needed
end

% Clear the camera when done
clear cam;
