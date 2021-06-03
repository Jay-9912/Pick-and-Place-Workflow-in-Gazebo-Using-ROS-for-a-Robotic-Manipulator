function exampleCommandClassifyPartsROSGazebo(coordinator)
%
%CommandClassifyParts Classify the parts to determine where to place them
%   This command classifies the detected parts using a numeric type: type 1
%   or type 2.
%
% Copyright 2020 The MathWorks, Inc.

    % In this implementation, parts have already been classified in
    % function commandDetectParts

   % Trigger Stateflow chart Event
   coordinator.FlowChart.partsClassified;       
end