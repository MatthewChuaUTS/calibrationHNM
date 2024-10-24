robotPoses = [
    [0.1706, -0.0701, 0.1720],
    [0.2197, -0.0053, 0.0495],
    [0.1836, -0.1270, 0.0656],
    [0.2098, 0.0716, 0.1280],
    [0.2018, -0.0024, 0.0132],
    [0.2895, 0.0040, 0.0836],
    [0.1939, 0.1130, 0.1062],
    [0.2037, 0.0132, 0.1802],
    [0.3031, 0.0119, 0.0355],
    [0.2369, -0.1186, 0.0516],
    [0.1822, 0.0351, 0.1037]
];

eulerAngles = [
    [-0.9811, 0, 0, 0.1937],
    [-0.9999, 0, 0, 0.0120],
    [-0.9546, 0, 0, 0.2980],
    [0.9865, 0, 0, 0.1637],
    [-1.0000, 0, 0, 0.0060],
    [1.0000, 0, 0, 0.0069],
    [0.9654, 0, 0, 0.2608],
    [0.9995, 0, 0, 0.0323],
    [0.9998, 0, 0, 0.0196],
    [-0.9732, 0, 0, 0.2301],
    [0.9955, 0, 0, 0.0950]
];

for i=1:length(robotPoses)
    sendTargetEndEffectorPose(robotPoses(i,:), eulerAngles(i,:));
    input('once the robot moves, press enter to move the robot');
end