function T_CE = newPhotoFinalProject(cam)
    
fixedRobotPoses = [0.1958   -0.0144    0.1640
    0.1932   -0.0362    0.1645
    0.1859   -0.0674    0.1567
    0.1964   -0.0219    0.1568
    0.1956    0.0200    0.1627
    0.1981    0.0150    0.1491
    0.1991   -0.0071    0.1434
    0.1969   -0.0318    0.1358
    0.1914   -0.0560    0.1338
    0.1984   -0.0171    0.1470]
fixedRobotOrientatons = [-0.0735         0         0
   -0.1853         0         0
   -0.3479         0         0
   -0.1109         0         0
    0.1021         0         0
    0.0758         0         0
   -0.0356         0         0
   -0.1603         0         0
   -0.2846         0         0
   -0.0859         0         0]

% Initialise Webcam
% cam = webcam;

% Specify the path for saving images
save_path = 'C:\Users\harrs\OneDrive - UTS\Documents\GitHub\calibrationHNM\';

% Check if save_path exists
if ~isfolder(save_path)
    error("Save path does not exist. Please check the path.");
end

% Capture and save images with sequential filenames
numImages = 10;
for i = 1:numImages
    % Capture image
    sendTargetEndEffectorPose(fixedRobotPoses(i,:), fixedRobotOrientatons(i,:));
    img = snapshot(cam);
    
    pause(3);
    % Check if the image was captured successfully
    if isempty(img)
        error("Image capture failed. Please check the camera setup.");
    end
    
    % Define filename with path and sequence number
    filename = sprintf('%simage%d.png', save_path, i);
    
    % Save the image
    imwrite(img, filename);
    
    % Display a confirmation message
    fprintf("Image saved as %s\n", filename);
end

% Release the camera after capturing images
% clear cam;

% Define images to process
imageFileNames = cellstr(sprintfc('image%d.png', 1:numImages));

% Check if imageFileNames is populated correctly
if isempty(imageFileNames)
    error("No images found in the specified path. Check that images are saved correctly.");
else
    disp('Image files detected:');
    disp(imageFileNames);  % Display list of detected images for debugging
end

% Detect calibration pattern in images
detector = vision.calibration.monocular.CheckerboardDetector();
[imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames, 'HighDistortion', true);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread('C:\Users\harrs\OneDrive - UTS\Documents\GitHub\calibrationHNM/image1.png');
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates for the planar pattern keypoints
squareSize = 13;  % in units of 'millimeters'
worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize, 'boardSize', [5 5]);
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
h2=figure; showExtrinsics(cameraParams, 'PatternCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
undistortedImage = undistortImage(originalImage, cameraParams);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')

numImages = length(imageFileNames); % Number of images taken by the moving camera
movingCameraPoses = cell(1, numImages); % Preallocate a cell array

for i = 1:numImages
    % Detect checkerboard in the i-th image
    [imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames{i});
    if size(imagePoints, 1) > size(worldPoints, 1)
        imagePoints = imagePoints(1:size(worldPoints, 1), :);
    end

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