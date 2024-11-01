function mattsVisualServoing(cam, basePosImg, T_camera_endeffector, cameraParams)
    % VISUALSERVOING Performs visual servoing using checkerboard detection.
    %
    % Inputs:
    %   cam                 - Camera object used to capture images.
    %   basePosImg          - Reference image of the checkerboard in the desired position.
    %   T_camera_endeffector - Transformation from the camera frame to the end-effector frame.
    %   cameraParams        - Camera intrinsic parameters.

    % Define the control gain and time step
    Lambda = 1; % Control gain
    dt = 0.05;     % Time step (seconds)
    tolerance = 1e-3; % Error tolerance

    % Camera intrinsic parameters
    principalPoint = cameraParams.Intrinsics.PrincipalPoint;
    focalLength = cameraParams.Intrinsics.FocalLength;

    % Define the size of each checkerboard square in millimeters
    squareSize = 0.013;  % Checkerboard square size in meters

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

        % Compute depths for each point
        numPoints = size(currentPoints, 1);
        Z = zeros(numPoints, 1);
        for i = 1:numPoints
            % Transform world points to camera frame using rotation and translation
            worldPoint = [worldPoints(i, :) 1]'; % Homogeneous coordinates
            cameraPoint = rotationMatrix * worldPoint(1:3) + translationVector';
            Z(i) = cameraPoint(3); % Depth of the i-th point
        end

        % Normalize the matched feature coordinates
        xy = (desiredPoints - principalPoint) ./ focalLength;
        Obsxy = (currentPoints - principalPoint) ./ focalLength;

        % Number of feature points
        n = size(desiredPoints, 1); % Use all detected points

        % Interaction matrix for translational DoF only
        Lx = zeros(2*n, 3);

        for i = 1:n
            Lxi = FuncLxTranslational(Obsxy(i, 1), Obsxy(i, 2), Z(i));
            Lx(2*i-1:2*i, :) = Lxi;
        end

        % Compute the error
        e = reshape((Obsxy - xy)', [], 1); % e is 2n x 1

        % Compute error norm
        errorNorm = norm(e);
        disp(['Error Norm: ', num2str(errorNorm)]);

        % Termination condition
        if errorNorm < tolerance
            disp('Target position reached within tolerance.');
            % break;
        end

        % Set a minimum error threshold
        minErrorThreshold = 0.005; % Adjust as needed
        if errorNorm < minErrorThreshold
            disp('Error below threshold; no movement commanded.');
            Vc_translational = zeros(3, 1);
        else
            % Damped least squares inverse of Lx
            lambda_dls = 0.01; % Damping factor
            Lx_pseudo_inv = Lx' * inv(Lx * Lx' + lambda_dls^2 * eye(2*n));

            % Compute control law (translational velocities only)
            Vc_translational = -Lambda * Lx_pseudo_inv * e; % Vc_translational is 3 x 1
        end

        % Integrate translational velocities to get displacement
        delta_Xc = Vc_translational * dt;
        delta_Xc = delta_Xc(:); % Ensure it's a column vector

        % Limit maximum displacement
        max_displacement = 0.005; % Maximum allowed displacement in meters
        delta_Xc = max(-max_displacement, min(max_displacement, delta_Xc));

        % Transform delta_Xc from camera frame to end-effector frame
        delta_Xe = T_camera_endeffector(1:3,1:3) * delta_Xc; % delta_Xe is 3 x 1

        % Since rotation is not possible, set rotational increments to zero
        delta_Omega_e = zeros(3,1);

        % Combine translational and rotational increments
        delta_Xe_full = [delta_Xe; delta_Omega_e]; % delta_Xe_full is 6 x 1

        % Update End-Effector Pose
        [currentPosition, currentAngle] = getCurrentEndEffectorPose();

        % New position is the current position plus the delta
        newPosition = currentPosition + delta_Xe';
        disp('delta');
        disp(delta_Xe);

        % Send new joint positions to the robot
        sendTargetEndEffectorPose(newPosition, currentAngle);

        % Wait for Next Iteration
        pause(dt);
    end
end

% Function to compute the interaction matrix for translational movement
function Lxi = FuncLxTranslational(x, y, Z)
    Lxi = [-1/Z, 0, x/Z;
           0, -1/Z, y/Z];
end
