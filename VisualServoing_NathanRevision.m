function VisualServoing_NathanRevision(cam, basePosImg, T_camera_endeffector)
    % VISUALSERVOING Performs visual servoing using checkerboard detection.
    %
    % Inputs:
    %   cam                 - Camera object used to capture images.
    %   basePosImg          - Reference image of the checkerboard in the desired position.
    %   T_camera_endeffector - Transformation from the camera frame to the end-effector frame.
    
    % Define the control gain and time step
    Lambda = 0.18;
    dt = 0.2;         % Time step (seconds)
    tolerance = 1e-3; % Error tolerance
    
    % Camera intrinsic parameters
    imageWidth = 1280;
    imageHeight = 720; %#ok<NASGU>
    principalPoint = [960, 540];
    fovRadians = 69.4 * (pi / 180);
    focalLengthPixels = (imageWidth / 2) / tan(fovRadians / 2);
    focalLength = [focalLengthPixels, focalLengthPixels];
    
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
        
        % Normalize the matched feature coordinates
        xy = (desiredPoints - principalPoint) ./ focalLength;
        Obsxy = (currentPoints - principalPoint) ./ focalLength;
        
        % Compute the error between current and desired checkerboard points
        e2 = Obsxy - xy;
        e = reshape(e2', [], 1);
        
        % Check if the error is within tolerance
        if norm(e) < tolerance
            disp('Error is within tolerance. Exiting control loop.');
            break;
        end
        
        % Compute Interaction Matrix Lx
        n = size(desiredPoints, 1);
        Lx = zeros(2*n, 6);  % Preallocate Lx for efficiency
        Z = 1.05;  % Ok I need to adjust this but apparently Z is in meters

        for i = 1:n
            Lxi = FuncLx(Obsxy(i, 1), Obsxy(i, 2), Z);
            Lx(2*i-1:2*i, :) = Lxi;
        end
        
        % Compute Camera Velocity Vc
        Vc = -Lambda * pinv(Lx) * e;
        
        % Integrate Vc to Get Displacement
        delta_Xc = Vc * dt;
        
        % Update End-Effector Pose
        % Get current pose
        [currentPosition, currentEulerAngles] = getCurrentEndEffectorPose();
        currentRotationMatrix = eul2rotm(currentEulerAngles, 'XYZ');
        T_current = [currentRotationMatrix, currentPosition'; 0 0 0 1];
        
        % Compute incremental transformation
        delta_translation = delta_Xc(1:3);
        deltaEulerAngles = delta_Xc(4:6)';
        delta_rotation_matrix = eul2rotm(deltaEulerAngles, 'XYZ');
        T_delta = [delta_rotation_matrix, delta_translation; 0 0 0 1];
        
        % Compute new pose
        T_new = T_current * T_delta * T_camera_endeffector;
        newPosition = T_new(1:3, 4)';
        newRotationMatrix = T_new(1:3, 1:3);
        newEulerAngles = rotm2eul(newRotationMatrix, 'XYZ');
        
        % Send New Pose to Robot
        sendTargetEndEffectorPose(newPosition, newEulerAngles);
        
        % Wait for Next Iteration
        % Remove the pause line for continuous real-time response
        % pause(dt);
    end
end
