function InitialiseRos(runInit)
    if runInit == 1
        rosinit('192.168.27.1'); % If unsure, please ask a tutor
        input('ros initialised, press enter to continue');
    end
end