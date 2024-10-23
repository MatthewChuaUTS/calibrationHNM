function InitialiseDobot()
    % SetInitialiseDobot
    [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
    safetyStateMsg.Data = 2;
    send(safetyStatePublisher,safetyStateMsg);
    % Initialize a variable to store the length of the previous message
    prevMsgLength = 0;
    
    for timeUntilCompletion = 10:-1:0
        % Create the message to display
        msg = sprintf('Time until initialization is complete: %d seconds', timeUntilCompletion);
        
        if timeUntilCompletion ~= 10
            % Erase the previous message by printing backspaces
            fprintf(repmat('\b', 1, prevMsgLength)); % YOOO THIS IS SO COOOL
        end
    
        % Print the new message
        fprintf('%s', msg);
        
        % Update the length of the message for the next iteration
        prevMsgLength = length(msg);
        
        pause(1);
    end
    % Move to the next line after completion
    fprintf('\nInitialization complete.\n');
end