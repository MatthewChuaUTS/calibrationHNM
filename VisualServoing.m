function VisualServoing(cam, basePosImg, T_camera_endeffector)
    % VISUALSERVOING Performs visual servoing using matched SURF features.
    %
    % Inputs:
    %   cam        - Camera object used to capture images.
    %   basePosImg - Reference image for the desired features.
    
    % Define the control gain and time step
    Lambda = 0.18;
    dt = 0.2;         % Time step (seconds)
    tolerance = 1e-3; % Error tolerance
    
    % Camera intrinsic parameters (adjust these to match your camera)
    imageWidth = 1280;
    imageHeight = 720; %#ok<NASGU>
    principalPoint = [960, 540]; 

    % Field of view in radians (using horizontal FOV)
    fovRadians = 69.4 * (pi / 180);
    focalLengthPixels = (imageWidth / 2) / tan(fovRadians / 2);
    focalLength = [focalLengthPixels, focalLengthPixels];
    
    % Estimated depth of features (Z-coordinate)
    Z = 0.18; % Meters
    
    % Extract Desired Features
    % Detect SURF features in the reference image
    basePosImg = rgb2gray(basePosImg);
    desiredFeatures = detectSURFFeatures(basePosImg);
    
    % Extract features (descriptors) from the base image
    [desiredFeaturesExtracted, desiredValidPoints] = extractFeatures(basePosImg, desiredFeatures);
    
    % Main Control Loop
    while true
        % Get Current Features and Compute Error
        % Capture current image from the camera
        currentImg = rgb2gray(snapshot(cam));
        
        % Detect SURF features in the current image
        currentFeatures = detectSURFFeatures(currentImg);
        
        % Extract features (descriptors) from the current image
        [currentFeaturesExtracted, currentValidPoints] = extractFeatures(currentImg, currentFeatures);
        
        % Match features between the base image and the current image
        indexPairs = matchFeatures(desiredFeaturesExtracted, currentFeaturesExtracted);
        
        % Retrieve the matched points
        matchedDesiredPoints = desiredValidPoints(indexPairs(:,1));
        matchedCurrentPoints = currentValidPoints(indexPairs(:,2));
        
        % Update 'n' to the number of matched features
        n = size(matchedDesiredPoints, 1);
        
        % Check if any features were matched
        if n == 0
            disp('No matched features found. Exiting control loop.');
            break;
        end
        
        % Normalize the matched feature coordinates
        xy = (matchedDesiredPoints.Location - principalPoint) ./ focalLength;
        Obsxy = (matchedCurrentPoints.Location - principalPoint) ./ focalLength;
        
        % Compute the error between current and desired features
        e2 = Obsxy - xy;
        e = reshape(e2', [], 1);
        
        % Check if the error is within tolerance
        if norm(e) < tolerance
            disp('Error is within tolerance. Exiting control loop.');
            break;
        end
        
        % Compute Interaction Matrix Lx
        Lx = zeros(2*n, 6); % Preallocate Lx for efficiency
        for i = 1:n
            Lxi = FuncLx(Obsxy(i, 1), Obsxy(i, 2), Z);
            Lx(2*i-1:2*i, :) = Lxi;
        end
        
        % Compute Camera Velocity Vc
        Vc = -Lambda * pinv(Lx) * e;
        
        %Integrate Vc to Get Displacement
        delta_Xc = Vc * dt;
        
        % Update End-Effector Pose
        % Get current pose
        [currentPosition, currentEulerAngles] = getCurrentEndEffectorPose();
        currentRotationMatrix = eul2rotm(currentEulerAngles, 'XYZ');
        T_current = [currentRotationMatrix, currentPosition'; 0 0 0 1];
        disp('TC')
        disp(T_current);
        
        % Compute incremental transformation
        delta_translation = delta_Xc(1:3);
        deltaEulerAngles = delta_Xc(4:6)';
        delta_rotation_matrix = eul2rotm(deltaEulerAngles, 'XYZ');
        T_delta = [delta_rotation_matrix, delta_translation; 0 0 0 1];
        disp('delta');
        disp(T_delta);
        
        % Compute new pose
        T_new = T_current * T_delta; % * T_camera_endeffector *0.01
        newPosition = T_new(1:3, 4)';
        newRotationMatrix = T_new(1:3, 1:3);
        newEulerAngles = rotm2eul(newRotationMatrix, 'XYZ');
        disp('tn');
        disp(T_new);
        
        % Send New Pose to Robot
        sendTargetEndEffectorPose(newPosition, newEulerAngles);
        
        % Wait for Next Iteration
        % pause(dt);
        input('press enter for next step');
    end
end