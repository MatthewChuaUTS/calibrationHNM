clc;
clear;

%% Parameters
image_resolution = [1920, 1080];
principal_point = [932, 542];
focal_length = [985, 978];
Z = 1.2;
Lambda = 0.18;

current_features = [25, 80; 965, 25; 80, 745; 965, 745];
desired_features = [20, 20; 1550, 20; 20, 750; 1550, 800];

%% Normalise coordinates
xy = (desired_features - principal_point) ./ focal_length;
Obsxy = (current_features - principal_point) ./ focal_length;

%% Construct Interaction Matrix
n = length(desired_features(:, 1));
Lx = [];
for i = 1:n
    Lxi = FuncLx(Obsxy(i, 1), Obsxy(i, 2), Z);
    Lx = [Lx; Lxi];
end

%% Compute the reprojection error
e2 = Obsxy - xy;
e = reshape(e2', [], 1);

%% Compute the camera velocity
Lx2 = pinv(Lx);
Vc = -Lambda * Lx2 * e;

%% Display Results
disp('Computed camera velocity (Vc):');
disp(Vc);