function CalibrateDobotMagician(calibrationAmount, cam)
    imageArray = zeros((length(calibrationAmount) - 1));

    % teach the robot to different positions, save [xyz] [rpy] 10-20x
    moveArray = [   [],[];...
                    [],[];...
                    [],[];...
                    [],[];...
                    [],[];...
                    [],[];...
                    [],[];...
                    [],[];...
                    [],[];...
                    [],[]...
                    ]; 
    
    
    % Joint states of the robot for calibration

    % Capture the images after moving the robot
    for i = 1:length(calibrationAmount)       
        position = moveArray(i, 1);  % Extract position
        rotation = moveArray(i, 2);  % Extract rotation
        sendTargetEndEffectorPose(position, rotation); % Pass position and rotation separately
        img = snapshot(cam);
        imageArray(i) = img;
    end

    % use the images in imageArray and intrinsically and extrinsically 
    % calibrate using moveArray 
    % Insert the script that the calibration app generates and it should be
    % good
end

