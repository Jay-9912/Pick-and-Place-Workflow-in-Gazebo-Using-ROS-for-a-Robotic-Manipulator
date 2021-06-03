function exampleCommandPickingLogicROSGazebo(coordinator)

% Determine which parts to pick next
%   This command instructs the robot which parts to pick next based on the
%   order of a preset list.
%
% Copyright 2020 The MathWorks, Inc.

       % Parts will be picked according to order in job.detectedParts list
        coordinator.NextPart = coordinator.NextPart + 1; 
        if coordinator.NextPart<=length(coordinator.Parts)               
            % Objects are placed on either belt1 or belt2 according to
            % their type
            if coordinator.DetectedParts{coordinator.NextPart}.type == 1
                coordinator.DetectedParts{coordinator.NextPart}.placingBelt = 1;                    
            else
                coordinator.DetectedParts{coordinator.NextPart}.placingBelt = 2;
            end
            disp(coordinator.DetectedParts{coordinator.NextPart}.type)
            % Trigger Stateflow chart Event
            coordinator.FlowChart.partsDetected;
            return;
        end

        % Trigger Stateflow chart Event
        coordinator.FlowChart.noPartsDetected;

end