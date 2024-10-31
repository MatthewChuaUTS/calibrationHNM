function mattsVisualServoing(cam, basePosImg, T_camera_endeffector, cameraParams)
    % VISUALSERVOING Performs visual servoing using checkerboard detection.
    %
    % Inputs:
    %   cam                 - Camera object used to capture images.
    %   basePosImg          - Reference image of the checkerboard in the desired position.
    %   T_camera_endeffector - Transformation from the camera frame to the end-effector frame.
    
    % Define the control gain and time step
    % Lambda = 0.18;
    % dt = 0.1;         % Time step (seconds)
    Lambda = 0.05; % Reduced from 0.18
    dt = 0.05;     % Reduced from 0.1
    tolerance = 1e-6; % Error tolerance
    
    % Camera intrinsic parameters
    principalPoint = cameraParams.Intrinsics.PrincipalPoint;
    focalLengthPixels = cameraParams.Intrinsics.FocalLength;
    focalLength = cameraParams.Intrinsics.FocalLength
    
    % Define the size of each checkerboard square in millimeters
    squareSize = 13;  % Checkerboard square size in mm
    
    % Detect the reference (desired) checkerboard points
    basePosImg = rgb2gray(basePosImg);
    [desiredPoints, boardSize] = detectCheckerboardPoints(basePosImg);
    
    % Generate world coordinates of the checkerboard corners
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);

    % Main Control Loop
    while true
        % Capture the current image from the camera
        currentImg = rgb2gray(snapshot(cam));
        
        % Detect checkerboard points in the current image
        [currentPoints, ~] = detectCheckerboardPoints(currentImg);
        
        % Check if checkerboard was detected in the current image
        if isempty(currentPoints)
            disp('Checkerboard not detected in the current image.');
            continue;
        end
        
        % Estimate the pose of the checkerboard
        [rotationMatrix, translationVector] = extrinsics(currentPoints, worldPoints, cameraParams);
        Z = translationVector(3); % Update Z with the actual depth
        
        % Normalize the matched feature coordinates
        xy = (desiredPoints - principalPoint) ./ focalLength;
        Obsxy = (currentPoints - principalPoint) ./ focalLength;
        
        % Compute Interaction Matrix Lx
        n = size(desiredPoints, 1);
        Lx = zeros(2*n, 6);  % Preallocate Lx for efficiency
        
        for i = 1:n
            Lxi = FuncLx(Obsxy(i, 1), Obsxy(i, 2), Z, Lambda);
            Lx(2*i-1:2*i, :) = Lxi;
        end
        
        % Compute the error
        e = reshape((Obsxy - xy)', [], 1);
    
        % Compute error norm
        errorNorm = norm(e);
        disp(['Error Norm: ', num2str(errorNorm)]);
    
        % Termination condition
        if errorNorm < tolerance
            disp('Target position reached within tolerance.');
            break;
        end
    
        % Set a minimum error threshold
        minErrorThreshold = 0.005; % Adjust as needed
        if errorNorm < minErrorThreshold
            disp('Error below threshold; no movement commanded.');
            Vc = zeros(6, 1);
        else
            % Damping factor for damped least squares
            lambda_dls = 0.01; % Adjust as necessary
    
            % Compute damped least squares inverse of Lx
            Lx_pseudo_inv = Lx' * inv(Lx * Lx' + lambda_dls^2 * eye(size(Lx,1)));
    
            % Compute control law
            Vc = -Lambda * Lx_pseudo_inv * e;
        end
    
        % Display Vc
        disp('Vc:');
        disp(Vc);
    
        % Integrate Vc to Get Displacement
        delta_Xc = Vc * dt;
        disp('delta_Xc:');
        disp(delta_Xc);
    
        % Limit maximum displacement
        max_displacement = 0.005; % Maximum allowed displacement in meters
        delta_Xc(1:3) = max(min(delta_Xc(1:3), max_displacement), -max_displacement);
        delta_Xc(4:6) = max(min(delta_Xc(4:6), max_displacement), -max_displacement);
    
        % Transform delta_Xc from camera frame to end-effector frame
        delta_Xe = T_camera_endeffector(1:3,1:3) * delta_Xc(1:3);
        delta_Omega_e = T_camera_endeffector(1:3,1:3) * delta_Xc(4:6);
        delta_Xe_full = [delta_Xe; delta_Omega_e];
    
        % Display delta_Xe_full
        disp('delta_Xe_full:');
        disp(delta_Xe_full);
    
        % Update End-Effector Pose
        [currentPosition, currentEulerAngles] = getCurrentEndEffectorPose();
        currentRotationMatrix = eul2rotm(currentEulerAngles, 'XYZ');
        T_current = [currentRotationMatrix, currentPosition'; 0 0 0 1];
    
        % Compute incremental transformation
        delta_translation = delta_Xe_full(1:3);
        deltaEulerAngles = delta_Xe_full(4:6)';
        delta_rotation_matrix = eul2rotm(deltaEulerAngles, 'XYZ');
        T_delta = [delta_rotation_matrix, delta_translation; 0 0 0 1];
    
        % Compute new pose
        T_new = T_current * T_delta;
        newPosition = T_new(1:3, 4)';
        newRotationMatrix = T_new(1:3, 1:3);
        newEulerAngles = rotm2eul(newRotationMatrix, 'XYZ');
    
        % Send New Pose to Robot
        sendTargetEndEffectorPose(newPosition, newEulerAngles);
        disp('Current End-Effector Pose (T_current):');
        disp(T_current);
        disp('New End-Effector Pose (T_new):');
        disp(T_new);
        input("Press enter to continue");
    
        % Wait for Next Iteration
        pause(dt);
end
