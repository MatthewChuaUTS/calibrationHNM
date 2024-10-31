function mattsVisualServoing(cam, basePosImg, T_camera_endeffector, cameraParams)
    % VISUALSERVOING Performs visual servoing using checkerboard detection.
    %
    % Inputs:
    %   cam                 - Camera object used to capture images.
    %   basePosImg          - Reference image of the checkerboard in the desired position.
    %   T_camera_endeffector - Transformation from the camera frame to the end-effector frame.
    
    % Define the control gain and time step
    Lambda = 0.18;
    dt = 1;         % Time step (seconds)
    tolerance = 1e-3; % Error tolerance
    
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
        % if isempty(currentPoints)
        %     disp('Checkerboard not detected in the current image.');
        %     continue;
        % end
        % 
        % Normalize the matched feature coordinates
        xy = (desiredPoints - principalPoint) ./ focalLength;
        Obsxy = (currentPoints - principalPoint) ./ focalLength;
        

        % Compute Interaction Matrix Lx
        n = size(desiredPoints, 1);
        Lx = zeros(2*n, 6);  % Preallocate Lx for efficiency
        Z = 0.18;  % Ok I need to adjust this but apparently Z is in meters

        for i = 1:n
            Lxi = FuncLx(Obsxy(i, 1), Obsxy(i, 2), Z, Lambda);
            Lx(2*i-1:2*i, :) = Lxi;
        end
        
        e2 = Obsxy-xy;  % We're seeing how far our square has moved
        e = reshape(e2',[],1);  % We're organizing our numbers in a special way
        de = -e*Lambda;  % We're figuring out how much we need to move our camera


        Lx2 = inv(Lx'*Lx)*Lx';  %#ok<MINV> % We're doing some more special math
        Vc = -Lambda*Lx2*e;   % This tells us how to move our camera to center the square


        % Integrate Vc to Get Displacement
        delta_Xc = Vc * dt;
        
        % Update End-Effector Pose
        % Get current pose
        [currentPosition, currentEulerAngles] = getCurrentEndEffectorPose();
        currentRotationMatrix = eul2rotm(currentEulerAngles, 'XYZ');
        T_current = [currentRotationMatrix, currentPosition'; 0 0 0 1];
        disp(T_current);
        
        % Compute incremental transformation
        delta_translation = delta_Xc(1:3);
        deltaEulerAngles = delta_Xc(4:6)';
        delta_rotation_matrix = eul2rotm(deltaEulerAngles, 'XYZ');
        T_delta = [delta_rotation_matrix, delta_translation; 0 0 0 1];
        disp(T_delta);
        
        % Compute new pose
        T_new = T_current * T_delta * T_camera_endeffector; %* T_camera_endeffector 
        newPosition = T_new(1:3, 4)'; 
        newRotationMatrix = T_new(1:3, 1:3);
        newEulerAngles = rotm2eul(newRotationMatrix, 'XYZ');
        disp(T_new);
        
        % Send New Pose to Robot
        sendTargetEndEffectorPose(newPosition, newEulerAngles);
        
        % Wait for Next Iteration
        % Remove the pause line for continuous real-time response
        % pause(dt);
        input('press enter for next step');
    end
end