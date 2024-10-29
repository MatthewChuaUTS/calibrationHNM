calibrationXYZ = [
    [0.2689   -0.0034    0.1164],
    [0.2672   -0.0167    0.0616],
    [0.2664    0.0030    0.0855],
    [0.2805   -0.0246    0.0432],
    [0.2463   -0.0804    0.0831],
    [0.1870   -0.0002    0.1748],
    [0.2957   -0.0001    0.1182],
    [0.2437   -0.0092    0.0666],
    [0.2348   -0.0701    0.0702],
    [0.1851    0.0347    0.1661]
];

calibrationRPY = [
    [0.0624         0         0],
    [0.0125         0         0],
    [0.0860         0         0],
    [-0.0124         0         0],
    [-0.2406         0         0],
    [0.0740         0         0],
    [0.0744         0         0],
    [0.0370         0         0],
    [-0.2152         0         0],
    [0.2602         0         0]
];

for i=1:length(calibrationXYZ)
    sendTargetEndEffectorPose(calibrationXYZ(i,:), calibrationRPY(i,:));
    pause(2);
    % imageFileName = sprintf('C:\\Users\\mattk\\OneDrive - UTS\Uni\\Year 2\\SCMS\\calibrationHNM\\Image%d.png', i);  % Create a unique filename
    % imwrite(img, imageFileName);  % Save the image to the specified path
end

% for i = 1:length(calibrationXYZ)       
%     sendTargetEndEffectorPose(calibrationXYZ(i,:), calibrationRPY(i,:));
%     pause(3);
%     disp('taking an image');
%     pause(1); 
% end