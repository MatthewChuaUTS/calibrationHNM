function CalibrateDobotMagician()
    cam=webcam;
    % imageArray = zeros((length(calibrationAmount) - 1));

    % teach the robot to different positions, save [xyz] [rpy] 10-20x
    fixedRobotPoses = [
    [0.2689   -0.0034    0.1164],
    [0.2672   -0.0167    0.0616],
    [0.2664    0.0030    0.0855],
    [0.2805   -0.0246    0.0432],
    [0.2463   -0.0804    0.0831],
    [0.1870   -0.0002    0.1748],
    [0.2957   -0.0001    0.1182],
    [0.2437   -0.0092    0.0666],
    [0.2348   -0.0701    0.0702],
    [0.1851    0.0347    0.1661]
    ];
    
    calibrationRPY = [
        [0.0624         0         0],
        [0.0125         0         0],
        [0.0860         0         0],
        [-0.0124         0         0],
        [-0.2406         0         0],
        [0.0740         0         0],
        [0.0744         0         0],
        [0.0370         0         0],
        [-0.2152         0         0],
        [0.2602         0         0]
    ];
    
    for i=1:length(fixedRobotPoses)
        % sendTargetEndEffectorPose(calibrationXYZ(i,:), calibrationRPY(i,:));
        % pause(3);
        img = snapshot(cam);  % Capture the image
        imageFileName = sprintf('C:\\Users\\mattk\\OneDrive - UTS\\Uni\\Year 2\\SCMS\\calibrationHNM\\Image%d.png', i);  % Create a unique filename
        imwrite(img, imageFileName);  % Save the image to the specified path
        input('press enter to continue');
    end
    
        imageFileNames = {'C:\Users\mattk\OneDrive - UTS\Uni\Year 2\SCMS\calibrationHNM\Image1.png',...
            'C:\Users\mattk\OneDrive - UTS\Uni\Year 2\SCMS\calibrationHNM\Image2.png',...
            'C:\Users\mattk\OneDrive - UTS\Uni\Year 2\SCMS\calibrationHNM\Image3.png',...
            'C:\Users\mattk\OneDrive - UTS\Uni\Year 2\SCMS\calibrationHNM\Image4.png',...
            'C:\Users\mattk\OneDrive - UTS\Uni\Year 2\SCMS\calibrationHNM\Image5.png',...
            'C:\Users\mattk\OneDrive - UTS\Uni\Year 2\SCMS\calibrationHNM\Image6.png',...
            'C:\Users\mattk\OneDrive - UTS\Uni\Year 2\SCMS\calibrationHNM\Image7.png',...
            'C:\Users\mattk\OneDrive - UTS\Uni\Year 2\SCMS\calibrationHNM\Image8.png',...
            'C:\Users\mattk\OneDrive - UTS\Uni\Year 2\SCMS\calibrationHNM\Image9.png',...
            'C:\Users\mattk\OneDrive - UTS\Uni\Year 2\SCMS\calibrationHNM\Image10.png'...
            };
    
    % Detect calibration pattern in images
    detector = vision.calibration.monocular.CheckerboardDetector();
    [imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames, 'HighDistortion', true);
    imageFileNames = imageFileNames(imagesUsed);
    
    % Read the first image to obtain image size
    originalImage = imread(imageFileNames{1});
    [mrows, ncols, ~] = size(originalImage);
    
    % Generate world coordinates for the planar pattern keypoints
    squareSize = 35;  % in units of 'millimeters'
    worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);
    worldPoints3D = [worldPoints, zeros(size(worldPoints, 1), 1)];
    
    
    % Calibrate the camera
    [cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
        'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
        'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'millimeters', ...
        'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
        'ImageSize', [mrows, ncols]);
    
    % View reprojection errors
    h1=figure; showReprojectionErrors(cameraParams);
    
    % Visualize pattern locations
    h2=figure; showExtrinsics(cameraParams, 'CameraCentric');
    
    % Display parameter estimation errors
    displayErrors(estimationErrors, cameraParams);
    
    % For example, you can use the calibration data to remove effects of lens distortion.
    undistortedImage = undistortImage(originalImage, cameraParams);
    
    % See additional examples of how to use the calibration data.  At the prompt type:
    % showdemo('MeasuringPlanarObjectsExample')
    % showdemo('StructureFromMotionExample')
    
    numImages = 5; % Number of images taken by the moving camera
    movingCameraPoses = cell(1, numImages); % Preallocate a cell array
    
    for i = 1:numImages
        % Detect checkerboard in the i-th image
        [imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames{i});
    
        % Estimate pose using cameraParams from calibration
        [R, t] = estimateWorldCameraPose(imagePoints, worldPoints3D, cameraParams);
    
        % Store transformation matrix (4x4)
        movingCameraPoses{i} = [R, t'; 0 0 0 1];
    end
    
    function [R, t] = estimateTargetPoseFromImage(cameraImage, cameraParams, squareSize)
        % Detect checkerboard points
        [imagePoints, boardSize] = detectCheckerboardPoints(cameraImage);
    
        % Generate world coordinates of the checkerboard corners
        worldPoints = generateCheckerboardPoints(boardSize, squareSize);
        worldPoints3D = [worldPoints, zeros(size(worldPoints, 1), 1)];
        
    
        % Estimate camera pose
        [R, t] = estimateWorldCameraPose(imagePoints, worldPoints3D, cameraParams);
    end
    
    function result = helperEstimateHandEyeTransform(cameraPoses, robotPoses, calibrationType)
        % This is a simplified version. You might need a more complex implementation
        % depending on your specific requirements.
        
        numPoses = length(cameraPoses);
        A = zeros(3,3,numPoses-1);
        B = zeros(3,3,numPoses-1);
        
        for i = 1:numPoses-1
            A(:,:,i) = cameraPoses{i+1}(1:3,1:3)' * cameraPoses{i}(1:3,1:3);
            B(:,:,i) = robotPoses(i+1,:)' * robotPoses(i,:);
        end
        
        % Solve for rotation (simplified method)
        R = eye(3); % Placeholder for actual rotation solving
        
        % Solve for translation
        C = zeros(3*(numPoses-1), 3);
        d = zeros(3*(numPoses-1), 1);
        
        for i = 1:numPoses-1
            C(3*i-2:3*i,:) = eye(3) - A(:,:,i);
            d(3*i-2:3*i) = robotPoses(i+1,:)' - R * cameraPoses{i+1}(1:3,4) + ...
                           A(:,:,i) * R * cameraPoses{i}(1:3,4);
        end
        
        t = C \ d;
        
        % Create the transformation
        T = [R, t; 0 0 0 1];
        
        result.T = T;
    end
    
    handEyeCalibrationResult = helperEstimateHandEyeTransform(movingCameraPoses, fixedRobotPoses, 'eyeToHand');
    T_CE = handEyeCalibrationResult.T;  % Extract transformation matrix
    
    
    % 1. Get robot's end-effector pose relative to the base
    % [currentEndEffectorPosition, currentEulerAngles] = getCurrentEndEffectorPose();
    T_BE = [[1],[0],[0],[-0.0331];
    [0],[-0.1350],[0.9909],[-0.2433];
    [0],[-0.9909],[-0.1350],[0.0055];
    [0],[0],[0],[1]];
    
    % T_BE = transl(currentEndEffectorPosition)*trotx(currentEulerAngles(1)); % Assume this function returns a 4x4 transformation matrix
    disp('T_BE:');
    disp(T_BE);
    
    % 2. Use the hand-eye calibration result (T_CE) to find the camera's pose relative to the base
    T_BC = T_BE * T_CE; % Camera's pose relative to the base
    disp('T_BC:');
    disp(T_BC);
    
    % 3. Use the camera image to get the transformation from the camera to the target
    rotations = zeros(3, 3, numImages);
    translations = zeros(3, numImages);
    
    % Loop through each image to estimate T_CT
    for i = 1:numImages
        cameraImage = imread(imageFileNames{i});
        T_CT = estimateTargetPoseFromImage(cameraImage, cameraParams, squareSize);
    
        % Store rotation and translation separately
        rotations(:,:,i) = T_CT(1:3, 1:3);
        translations(:,i) = T_CT(1:3, 2);
    end
    
    % Average the translation vectors
    averageTranslation = mean(translations, 2);
    averageTranslation = averageTranslation(:);
    
    % Average the rotations using quaternions
    quaternions = zeros(4, numImages);
    for i = 1:numImages
        q = rotm2quat(rotations(:,:,i)); % Convert each rotation to a quaternion
        quaternions(:,i) = q';
    end
    averageQuaternion = mean(quaternions, 2);
    averageRotation = quat2rotm(averageQuaternion'); % Convert the average quaternion back to a rotation matrix
    
    % Construct the averaged T_CT
    T_CT_avg = [averageRotation, averageTranslation; 0, 0, 0, 1];
    disp('Averaged T_CT:');
    disp(T_CT_avg);
    
    % 4. Compute the transformation from the base to the target
    T_BT = T_BC * T_CT_avg; % T_BT now represents the pose of the target in the base coordinate frame
    disp('T_BT:');
    disp(T_BT); % Print the final transformation matrix
end