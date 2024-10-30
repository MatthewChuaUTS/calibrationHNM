% listOfPositions = [];
% listOfRotations = [];
% timesWeWantToCalibrate = 10;
% 
% for i=1:timesWeWantToCalibrate
%     disp('starting to get stuff');
%     [currentEndEffectorPosition, rotation] = getCurrentEndEffectorPose();
%     listOfPositions = [listOfPositions; currentEndEffectorPosition];
%     listOfRotations = [listOfRotations; rotation];
%     input('done, move the bot, then press enter');
% end
% disp('list of positions:')
% disp(listOfPositions);
% disp('list of rotations:')
% disp(listOfRotations);


pose = [
    [0.2161, -0.0052, 0.0745],
    [0.2002, -0.0178, 0.1260],
    [0.2336, 0.0482, 0.0859],
    [0.2649, 0.0173, 0.0825],
    [0.2302, 0.0722, 0.1327],
    [0.2429, 0.0062, 0.1268],
    [0.2536, 0.0261, 0.0510],
    [0.2594, 0.0136, 0.0609],
    [0.2367, 0.0615, 0.0877],
    [0.2422, -0.0185, 0.0823],
    [0.2422, -0.0185, 0.0823]
];

newOrientation = [
    [-0.0240, 0, 0],
    [-0.0886, 0, 0],
    [0.2034, 0, 0],
    [0.0652, 0, 0],
    [0.3040, 0, 0],
    [0.0254, 0, 0],
    [0.1026, 0, 0],
    [0.0522, 0, 0],
    [0.2541, 0, 0],
    [-0.0762, 0, 0],
    [-0.0762, 0, 0]
];



% Joint states of the robot for calibration

% Capture the images after moving the robot
% for i = 1:length(pose)       
%     sendTargetEndEffectorPose(pose(i,:), newOrientation(i,:));
%     pause(5); % pause until the robot has finished moving, we have to hardcode this 
% end
% 
% 
% 
% % % InitialiseRos(1);
% % IF IT DOESNT WORK WRITR: rosshutdown
% % % InitialiseDobot();
% % 
PositionMatrix = []; 
OrientationMatrix = [];
maxSteps = 10;

for i = 1:maxSteps
    input('Please move the robot to a new position, and then press enter');

    % Get the current end-effector pose (position and quaternion).
    [currentEndEffectorPosition, currentEndEffectorQuat] = getCurrentEndEffectorPose();

    % Append the position and orientation as rows to their respective matrices.
    PositionMatrix = [PositionMatrix; currentEndEffectorPosition];
    OrientationMatrix = [OrientationMatrix; currentEndEffectorQuat];
    disp('ok, I finished this one!');
end
disp(PositionMatrix);
disp(OrientationMatrix);
% % input('press enter to move again');
% % 
% % for i=1:maxSteps
% %     sendTargetEndEffectorPose(PositionMatrix(i,:), OrientationMatrix(i,:));
% %     input('press enter to move again');
% % end
% % 
% % % rosnode('ping', '/dobot_magician/dobot_magician_node')
% % % orientation = [
% % %     [-0.9999, 0, 0, 0.0120],
% % %     [-0.9990, 0, 0, 0.0443],
% % %     [0.9948, 0, 0, 0.1015],
% % %     [0.9995, 0, 0, 0.0326],
% % %     [0.9885, 0, 0, 0.1514],
% % %     [0.9999, 0, 0, 0.0127],
% % %     [0.9987, 0, 0, 0.0513],
% % %     [0.9997, 0, 0, 0.0261],
% % %     [0.9919, 0, 0, 0.1267],
% % %     [-0.9993, 0, 0, 0.0381],
% % %     [-0.9993, 0, 0, 0.0381]
% % % ];
% % % 
% % % pose = [
% % %     [0.2161, -0.0052, 0.0745],
% % %     [0.2002, -0.0178, 0.1260],
% % %     [0.2336, 0.0482, 0.0859],
% % %     [0.2649, 0.0173, 0.0825],
% % %     [0.2302, 0.0722, 0.1327],
% % %     [0.2429, 0.0062, 0.1268],
% % %     [0.2536, 0.0261, 0.0510],
% % %     [0.2594, 0.0136, 0.0609],
% % %     [0.2367, 0.0615, 0.0877],
% % %     [0.2422, -0.0185, 0.0823],
% % %     [0.2422, -0.0185, 0.0823]
% % % ];
% % % 
% % % newOrientation = [
% % %     [-0.0240, 0, 0],
% % %     [-0.0886, 0, 0],
% % %     [0.2034, 0, 0],
% % %     [0.0652, 0, 0],
% % %     [0.3040, 0, 0],
% % %     [0.0254, 0, 0],
% % %     [0.1026, 0, 0],
% % %     [0.0522, 0, 0],
% % %     [0.2541, 0, 0],
% % %     [-0.0762, 0, 0],
% % %     [-0.0762, 0, 0]
% % % ];
% % 
% % 
% % % newOrientation = [];
% % % for i=1:length(orientation)
% % %     [output] = quat2eul(orientation(i,:));
% % %     newOrientation = [newOrientation; output];
% % % end
% % 
% % % sendTargetEndEffectorPose(pose(1,:), newOrientation(1,:));
% % % 
% % % endEffectorPosition = [0.2422, -0.0185, 0.0823];
% % % endEffectorRotation = [-0.0762, 0, 0];
% % % sendTargetEndEffectorPose(endEffectorPosition, endEffectorRotation);
% % 
% % 
% % % for i=1:length(newOrientation)
% % %     sendTargetEndEffectorPose(pose(i,:), newOrientation(i,:));
% % %     input('press enter to move again');
% % % end
% % 