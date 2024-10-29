% Initialise RealSense pipeline and get intrinsics
pipe = realsense.pipeline();
config = realsense.config();
config.enable_stream(realsense.stream.depth, 640, 480, realsense.format.z16, 30);
config.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30);
profile = pipe.start(config);
depth_sensor = profile.get_device().first_depth_sensor();
intrinsics = depth_sensor.get_intrinsics();

% Transformation matrices (placeholders for now)
T_ce = ...; % Hand-eye calibration matrix (Camera to End-Effector)
T_bw = ...; % Base to World matrix
K_p = 0.1;  % Proportional control gain

while true
    % Step 1: Capture RGB and Depth Data
    [rgbImage, depthData] = getRGBDData(pipe);
    
    % Step 2: Detect Checkerboard Pattern
    [imagePoints, ~] = detectCheckerboardPoints(rgbImage);
    if isempty(imagePoints)
        continue; % Skip if checkerboard not found
    end
    
    % Step 3: Calculate 3D Position in Camera Frame
    pattern3DPos_camera = get3DPositionFromDepth(imagePoints, depthData, intrinsics);
    
    % Step 4: Transform to World Frame
    pattern3DPos_world = T_bw * T_ce * pattern3DPos_camera;
    
    % Step 5: Calculate Error and Generate Control Signal
    currentPos_world = getEndEffectorPosition(); % Retrieve end-effector world position
    error = pattern3DPos_world - currentPos_world;
    movementCommand = K_p * error; % Proportional control
    
    % Step 6: Send Movement Command to Dobot
    sendMovementCommand(movementCommand); % Command Dobot movement
    
    pause(0.1); % Small delay for smoother tracking
end
