function VisualServoing(cam)
    % Define the control gain and time step
    Lambda = 0.18;
    dt = 0.2;  % Time step (seconds) (Decrease value to increase the fidelity of the servoing alg)
    tolerance = 1e-3;  % Error tolerance
    
    % We need to calculate these
    principal_point = [932, 542]; 
    focal_length = [985, 978];
    Z = 1.2;
    
    
    while true
        % Step 1: Get Current Features and Compute Error
        currentImg = snapshot(cam);
        current_features = detectSURFFeatures(currentImg); % in one of the 
        % feature algorithms, I think we can specify how many of the most 
        % important features we want. This may be important as different 
        % nubmers of features might be found

        Obsxy = (current_features - principal_point) ./ focal_length;
        e2 = Obsxy - xy;
        e = reshape(e2', [], 1);
        
        % Check if the error is within tolerance
        if norm(e) < tolerance
            disp('Error is within tolerance. Exiting control loop.');
            break;
        end
    
        % Step 2: Compute Interaction Matrix Lx
        Lx = [];
        for i = 1:n
            Lxi = FuncLx(Obsxy(i, 1), Obsxy(i, 2), Z);
            Lx = [Lx; Lxi];
        end
    
        % Step 3: Compute Camera Velocity Vc
        Vc = -Lambda * pinv(Lx) * e;
    
        % Step 4: Integrate Vc to Get Displacement
        delta_Xc = Vc * dt;
    
        % Step 5: Update End-Effector Pose
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
        T_new = T_current * T_delta;
        newPosition = T_new(1:3, 4)';
        newRotationMatrix = T_new(1:3, 1:3);
        newEulerAngles = rotm2eul(newRotationMatrix, 'XYZ');
    
        % Step 6: Send New Pose to Robot
        sendTargetEndEffectorPose(newPosition, newEulerAngles);
    
        % Step 7: Wait for Next Iteration
        pause(dt);
    end
end