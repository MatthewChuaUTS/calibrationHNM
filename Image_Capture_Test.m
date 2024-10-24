% Initialise camera object
cam = webcam;  % Create webcam object

% Initialise the array to store images
imageArray = cell(1, 10);  

% Loop to capture 10 images
for i = 1:10
    img = snapshot(cam);  % Capture the image
    imageFileName = sprintf('C:\\matlab\\Final Project\\Image%d.png', i);  % Create a unique filename
    imwrite(img, imageFileName);  % Save the image to the specified path
    
    imageArray{i} = img;  % Store the image in the array if needed later
    input('Press Enter to continue');  % Pause until user presses Enter
end

% Clear the camera object after use
clear cam;

