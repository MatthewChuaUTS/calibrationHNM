function basePosImg = GetBasePos(cam)
    moveRobot(basePosition);
    basePosImg = snapshot(cam);
end