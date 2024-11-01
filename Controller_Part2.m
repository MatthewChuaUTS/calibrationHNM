function Controller_Part2(cam, T_camera_endeffector, T_base_world, cameraParams)
    % VISUALSERVOING Performs visual servoing using checkerboard detection and proportional control.
    %
    % Inputs:
    %   cam                  - Camera object used to capture images.
    %   T_camera_endeffector - Transformation from the camera frame to the end-effector frame.
    %   T_base_world         - Transformation from the robot base frame to the world frame.
    %   cameraParams         - Camera intrinsic parameters.

    % Define the proportional gain for control
    K_p = 0.1;  % Adjust this value for smooth control

    % Define checkerboard parameters
    squareSize = 0.035;  % Size of one square on the checkerboard in meters

    % Main Control Loop
    while true
        % Capture an image and detect checkerboard position
        img = snapshot(cam);
        [imagePoints, boardSize] = detectCheckerboardPoints(img);

        % Check if checkerboard was detected in the current image
        if isempty(imagePoints)
            disp('Checkerboard not detected.');
            continue;
        end

        % Generate 3D world coordinates of the checkerboard points
        worldPoints = generateCheckerboardPoints(boardSize, squareSize);
        worldPoints3D = [worldPoints, zeros(size(worldPoints, 1), 1)];

        % Estimate the position of the checkerboard in the camera frame
        [rotationMatrix, translationVector] = estimateWorldCameraPose(imagePoints, worldPoints3D, cameraParams);

        % Convert to homogeneous transformation matrix (pose of checkerboard in camera frame)
        T_cw = [rotationMatrix, translationVector'; 0, 0, 0, 1];

        % Transform checkerboard position to base frame
        T_cb = T_base_world * T_cw * T_camera_endeffector;  % Checkerboard position in base frame

        % Extract checkerboard position in base frame
        checkerboardPosition_base = T_cb(1:3, 4);

        % Get the current end-effector position in the base frame
        [currentEndEffectorPosition_base, rot] = getCurrentEndEffectorPose();  % Implement this function

        % Calculate the position error
        positionError = checkerboardPosition_base' - currentEndEffectorPosition_base;

        % Proportional control: generate movement command to reduce the error
        movementCommand = K_p * positionError;

        % Send the movement command to the Dobot
        sendTargetEndEffectorPose(movementCommand, rot);  % Implement this function

        % Display information
        fprintf("Current End-Effector Position: (%.2f, %.2f, %.2f)\n", currentEndEffectorPosition_base);
        fprintf("Checkerboard Position: (%.2f, %.2f, %.2f)\n", checkerboardPosition_base);
        fprintf("Position Error: (%.2f, %.2f, %.2f)\n", positionError);
        fprintf("Movement Command: (%.2f, %.2f, %.2f)\n\n", movementCommand);

        % Optional: Add a break condition to exit the loop if needed
        % For example, if the position error is below a certain threshold
        if norm(positionError) < 1e-3  % Adjust the threshold as needed
            disp('Target position reached within tolerance.');
            break;
        end

        % Wait for Next Iteration
        pause(0.1);
    end
end
