% Initialise camera object
cam = webcam;  % Create webcam object

% Initialise the array to store images and filenames
imageArray = cell(1, 10);
imageFileNames = cell(1, 10);  % Store file names for calibration later

% Loop to capture 10 images
for i = 1:10
    img = snapshot(cam);  % Capture the image
    imageFileName = sprintf('C:\\matlab\\Final Project\\Image%d.png', i);  % Create a unique filename
    imwrite(img, imageFileName);  % Save the image to the specified path
    
    imageArray{i} = img;  % Store the image in the array
    imageFileNames{i} = imageFileName;  % Store the file name for later use
    input('Press Enter to continue');  % Pause until user presses Enter
end

% Clear the camera object after use
clear cam;

% Detect calibration pattern in images
detector = vision.calibration.monocular.CheckerboardDetector();
[imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames, 'HighDistortion', true);
imageFileNames = imageFileNames(imagesUsed);  % Filter filenames to those successfully used

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates for the planar pattern keypoints
squareSize = 72;  % in units of 'millimeters'
worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

% View reprojection errors
h1 = figure; showReprojectionErrors(cameraParams);

% Visualise pattern locations
h2 = figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% Remove lens distortion from the first image
undistortedImage = undistortImage(originalImage, cameraParams);
imshow(undistortedImage);  % Display the undistorted image
