listOfPositions = [];
listOfRotations = [];
timesWeWantToCalibrate = 10;

for i=1:timesWeWantToCalibrate
    disp('starting to get stuff');
    [currentEndEffectorPosition, rotation] = getCurrentEndEffectorPose();
    listOfPositions = [listOfPositions; currentEndEffectorPosition];
    listOfRotations = [listOfRotations; rotation];
    input('done, move the bot, then press enter');
end

save("savedListOfPositions.mat", "listOfPositions");
save("savedlistOfRotations.mat", "listOfRotations");
disp('list of positions:')
disp(listOfPositions);
disp('list of rotations:')
disp(listOfRotations);

% for i = 1:length(listOfPositions)       
%     sendTargetEndEffectorPose(listOfPositions(i,:), listOfRotations(i,:));
%     pause(5); % pause until the robot has finished moving, we have to hardcode this 
% end