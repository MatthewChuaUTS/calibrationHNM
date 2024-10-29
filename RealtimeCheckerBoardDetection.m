% Load Camera Calibration Parameters
load('newPhotoFinalProject'); %Might have to manually add in the variables 
% we get 

% Initialise RealSense Camera
cam = webcam; 

% Define the size of each square on the checkerboard in millimeters
squareSize = 35;  % Adjust to whatever our square size is

% Set up a loop for real-time detection
figure; % Create a figure to display the results
while true
    % Capture an image
    img = snapshot(cam);
    
    % Detect checkerboard points
    [imagePoints, boardSize] = detectCheckerboardPoints(img);
    
    % If the checkerboard is not detected, skip the rest of the loop
    if isempty(imagePoints)
        disp("Checkerboard not detected.");
        continue;
    end
    
    % Draw detected points on the image for visualization
    imgWithPoints = insertMarker(img, imagePoints, 'o', 'Color', 'green', 'Size', 5);
    imshow(imgWithPoints);
    title("Checkerboard Detection - Real-Time");
    drawnow;  % Update the displayed image immediately

    % Generate world coordinates for the checkerboard corners
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);

    % Estimate camera pose (rotation and translation) relative to the checkerboard
    [rotationMatrix, translationVector] = estimateWorldCameraPose(imagePoints, worldPoints, newPhotoFinalProject);
    
    % Display the 3D position of the checkerboard (translation vector)
    fprintf("Checkerboard Position (X, Y, Z) in Camera Frame: (%.2f, %.2f, %.2f) mm\n", ...
            translationVector(1), translationVector(2), translationVector(3));
    
    % Optional: If you want to exit the loop after a certain condition, add a break condition here
    % For example, if a certain key is pressed, exit the loop
    %Ask Matt and Harrsha later if we add the function here then we
    %control our robot's on and off from here.
end

% Clear the camera when done
clear cam;
