function HarrshaVisualServoing(cam, basePosImg, T_camera_endeffector, cameraParams)
    % Define control parameters
    Lambda = 0.05;
    dt = 0.05;
    tolerance = 1e-6;

    % Extract camera intrinsics
    principalPoint = cameraParams.Intrinsics.PrincipalPoint;
    focalLength = cameraParams.Intrinsics.FocalLength;

    % Checkerboard parameters
    squareSize = 0.013;  % 13mm squares

    % Process reference image
    basePosImg = rgb2gray(basePosImg);
    [desiredPoints, boardSize] = detectCheckerboardPoints(basePosImg);
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);

    while true
        % Capture and process current image
        currentImg = rgb2gray(snapshot(cam));
        [currentPoints, ~] = detectCheckerboardPoints(currentImg);

        if isempty(currentPoints)
            disp('Checkerboard not detected.');
            continue;
        end

        % Estimate checkerboard pose
        [rotationMatrix, translationVector] = extrinsics(currentPoints, worldPoints, cameraParams);
        Z = translationVector(3);

        % Normalize feature coordinates
        xy = (desiredPoints - principalPoint) ./ focalLength;
        Obsxy = (currentPoints - principalPoint) ./ focalLength;

        % Compute Interaction Matrix
        n = size(desiredPoints, 1);
        Lx = zeros(2*n, 6);
        for i = 1:n
            Lxi = FuncLx(Obsxy(i, 1), Obsxy(i, 2), Z, Lambda);
            Lx(2*i-1:2*i, :) = Lxi;
        end

        % Compute error
        e = reshape((Obsxy - xy)', [], 1);
        errorNorm = norm(e);
        disp(['Error Norm: ', num2str(errorNorm)]);

        if errorNorm < tolerance
            disp('Target position reached.');
            break;
        end

        % Compute control law
        lambda_dls = 0.01;
        Lx_pseudo_inv = Lx' * inv(Lx * Lx' + lambda_dls^2 * eye(size(Lx,1)));
        Vc = -Lambda * Lx_pseudo_inv * e;

        % Integrate velocity to get displacement
        delta_Xc = Vc * dt;

        % Limit maximum displacement
        max_displacement = 0.005;
        delta_Xc(1:3) = max(min(delta_Xc(1:3), max_displacement), -max_displacement);
        delta_Xc(4:6) = max(min(delta_Xc(4:6), max_displacement), -max_displacement);

        % Transform displacement to end-effector frame
        delta_Xe = T_camera_endeffector(1:3,1:3) * delta_Xc(1:3);
        delta_Omega_e = T_camera_endeffector(1:3,1:3) * delta_Xc(4:6);
        delta_Xe_full = [delta_Xe; delta_Omega_e];

        % Update end-effector pose
        [currentPosition, currentEulerAngles] = getCurrentEndEffectorPose();
        currentRotationMatrix = eul2rotm(currentEulerAngles, 'XYZ');
        T_current = [currentRotationMatrix, currentPosition'; 0 0 0 1];

        deltaEulerAngles = delta_Xe_full(4:6)';
        delta_rotation_matrix = eul2rotm(deltaEulerAngles, 'XYZ');
        T_delta = [delta_rotation_matrix, delta_Xe_full(1:3); 0 0 0 1];

        T_new = T_current * T_delta;
        newPosition = T_new(1:3, 4)';
        newRotationMatrix = T_new(1:3, 1:3);
        newEulerAngles = rotm2eul(newRotationMatrix, 'XYZ');

        % Send new pose to robot
        sendTargetEndEffectorPose(newPosition, newEulerAngles);

        pause(dt);
    end
end

function Lx = FuncLx(x, y, Z, Lambda)
    Lx = [-Lambda/Z, 0, x/Z, x*y, -(Lambda+x^2), y;
           0, -Lambda/Z, y/Z, Lambda+y^2, -x*y, -x];
end