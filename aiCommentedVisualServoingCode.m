clc;        % Clear the command window
clear;      % Clear all variables from the workspace

%% Introduction to Visual Servoing
% Visual servoing is a control technique where visual information from a camera is used to control
% the motion of a robot or camera itself. The main goal is to move the robot so that the observed
% features in the camera image align with desired features.

%% Camera Parameters and Control Gain

% Define the image resolution (width x height) in pixels
image_resolution = [1920, 1080];

% Define the principal point (optical center of the camera) in pixels
principal_point = [932, 542]; 

% Define the focal length of the camera in pixels for x and y axes
focal_length = [985, 978];

% Estimate of the depth (Z-coordinate) of the features relative to the camera
Z = 1.2;   % Depth in meters

% Control gain (Lambda) for the visual servoing algorithm
Lambda = 0.18;

%% Feature Points in the Image

% Current observed positions of the features in the image (pixel coordinates)
% Each row corresponds to a feature point [x_pixel, y_pixel]
current_features = [
    25, 80;    % Feature 1
    965, 25;   % Feature 2
    80, 745;   % Feature 3
    965, 745   % Feature 4
];

% Desired positions of the features in the image (pixel coordinates)
desired_features = [
    20, 20;     % Desired position for Feature 1
    1550, 20;   % Desired position for Feature 2
    20, 750;    % Desired position for Feature 3
    1550, 800   % Desired position for Feature 4
];

%% Normalizing Image Coordinates

% Convert pixel coordinates to normalized image coordinates
% This accounts for the camera's intrinsic parameters (principal point and focal length)
xy = (desired_features - principal_point) ./ focal_length;
% 'xy' now contains the desired features in normalized coordinates

Obsxy = (current_features - principal_point) ./ focal_length;
% 'Obsxy' contains the current observed features in normalized coordinates

%% Constructing the Interaction Matrix (Image Jacobian)

% Number of feature points
n = length(desired_features(:, 1));

% Initialize the interaction matrix 'Lx'
Lx = [];

% Loop over each feature point to compute its interaction matrix
for i = 1:n
    % Extract the normalized coordinates of the current feature point
    x = Obsxy(i, 1);
    y = Obsxy(i, 2);
    
    % Compute the interaction matrix for the feature point
    % 'FuncLx' is a function that computes the interaction matrix for a point
    Lxi = FuncLx(x, y, Z);
    
    % Append the interaction matrix of the current point to 'Lx'
    Lx = [Lx; Lxi];
end

%% Computing the Error Vector

% Compute the error between the current and desired normalized feature positions
e2 = Obsxy - xy;
% 'e2' is a matrix where each row contains the error [delta_x, delta_y] for a feature point

% Reshape the error matrix into a single column vector
e = reshape(e2', [], 1);
% 'e' is a (2n x 1) vector containing the errors in x and y for all feature points

%% Computing the Camera Velocity

% Compute the pseudo-inverse of the interaction matrix 'Lx'
% This step solves for the camera velocities that will minimize the error 'e'
Lx_pinv = pinv(Lx);

% Compute the camera velocity vector 'Vc' using the control law
Vc = -Lambda * Lx_pinv * e;
% 'Vc' is a 6-element vector [Vx; Vy; Vz; Wx; Wy; Wz]
% Vx, Vy, Vz are linear velocities along the x, y, z axes
% Wx, Wy, Wz are angular velocities around the x, y, z axes

%% Displaying the Result

% Display the computed camera velocities
disp('Computed camera velocity (Vc):');
disp(Vc);

%% Function Definition

% This function computes the interaction matrix (Jacobian) for a feature point
% Input:
%   x, y - Normalized image coordinates of the feature point
%   Z    - Estimated depth of the feature point
% Output:
%   Lx   - The 2x6 interaction matrix for the feature point

function [Lx] = FuncLx(x, y, Z)
    % Initialize the interaction matrix with zeros
    Lx = zeros(2, 6);
    
    % Compute the elements of the interaction matrix
    % The interaction matrix relates the change in image features to the camera velocities
    
    % First row corresponds to the x-component of the image feature
    Lx(1, 1) = -1 / Z;        % Partial derivative with respect to Vx (linear velocity in x)
    Lx(1, 2) = 0;             % Partial derivative with respect to Vy (linear velocity in y)
    Lx(1, 3) = x / Z;         % Partial derivative with respect to Vz (linear velocity in z)
    Lx(1, 4) = x * y;         % Partial derivative with respect to Wx (angular velocity around x)
    Lx(1, 5) = -(1 + x^2);    % Partial derivative with respect to Wy (angular velocity around y)
    Lx(1, 6) = y;             % Partial derivative with respect to Wz (angular velocity around z)
    
    % Second row corresponds to the y-component of the image feature
    Lx(2, 1) = 0;             % Partial derivative with respect to Vx
    Lx(2, 2) = -1 / Z;        % Partial derivative with respect to Vy
    Lx(2, 3) = y / Z;         % Partial derivative with respect to Vz
    Lx(2, 4) = 1 + y^2;       % Partial derivative with respect to Wx
    Lx(2, 5) = -x * y;        % Partial derivative with respect to Wy
    Lx(2, 6) = -x;            % Partial derivative with respect to Wz
end
