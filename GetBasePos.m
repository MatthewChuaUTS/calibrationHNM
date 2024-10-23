function basePosImg = GetBasePos(cam)
    InitialiseDobot();
    basePosImg = snapshot(cam);
end