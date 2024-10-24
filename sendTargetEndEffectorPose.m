function sendTargetEndEffectorPose(endEffectorPosition, endEffectorRotation)   
    [targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
    
    targetEndEffectorMsg.Position.X = endEffectorPosition(1);
    targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
    targetEndEffectorMsg.Position.Z = endEffectorPosition(3);
    
    qua = eul2quat(endEffectorRotation);
    targetEndEffectorMsg.Orientation.W = qua(1);
    targetEndEffectorMsg.Orientation.X = qua(2);
    targetEndEffectorMsg.Orientation.Y = qua(3);
    targetEndEffectorMsg.Orientation.Z = qua(4);
    
    send(targetEndEffectorPub,targetEndEffectorMsg);
end
