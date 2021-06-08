function exampleCommandPickingLogicROSGazebo(coordinator)

% Determine which parts to pick next
%   This command instructs the robot which parts to pick next based on the
%   order of a preset list.
%
% Copyright 2020 The MathWorks, Inc.

       % Parts will be picked according to order in job.detectedParts list
        if coordinator.DetectedParts.type == 1
            coordinator.DetectedParts.placingBelt = 1;                    
        else
            coordinator.DetectedParts.placingBelt = 2;
        end
        disp(coordinator.DetectedParts.type)
        % Trigger Stateflow chart Event
        coordinator.FlowChart.partsDetected;

        % Trigger Stateflow chart Event

end