function exampleCommandDetectPartsROSGazebo(coordinator)

% Detect parts and identify their poses
% This function detects parts using a pre-trained deep learning model. Each
% part is a struct with 2 elements: centerPoint and type.

% Copyright 2020 The MathWorks, Inc.

     % Empty cell array of parts to detect new parts
     coordinator.Parts = {};

     % Camera properties
     hfov = 1.211269;
     imageWidth = 480;
     focalLength = (imageWidth/2)/tan(hfov/2);

     % Read image from simulated Gazebo camera
     rgbImg = readImage(coordinator.ROSinfo.rgbImgSub.LatestMessage);
     centerPixel = [round(size(rgbImg,1)/2), round(size(rgbImg,2)/2)];

     % Detect parts and show labels
     % figure;
     imshow(rgbImg);
     [bboxes,~,labels] = detect(coordinator.DetectorModel,rgbImg); % 检测出物体
     if ~isempty(labels) 
        labeledImg = insertObjectAnnotation(rgbImg,'Rectangle',bboxes,cellstr(labels));
        imshow(labeledImg); % 显示bbox           
        numObjects = size(bboxes,1); % 物体数量
        allLabels =table(labels);
        %需要修改
        for i=1:numObjects
            if allLabels.labels(i)=='can'
                % Height of objects is known according to type
                part.Z = 0.052;
                part.type = 2;
            else 
                part.Z = 0.17;
                part.type = 1;
            end
            cameraTransf = getTransform(coordinator.Robot, coordinator.CurrentRobotJConfig, 'EndEffector_Link');
            cameraZ = cameraTransf(3,4);
            zDistance = cameraZ - part.Z;
            centerBox = [bboxes(i,2)+ round(bboxes(i,4)/2), bboxes(i,1)+ round(bboxes(i,3)/2)];
            centerBoxwrtCenterPixel = centerBox - centerPixel; % in pixels
            worldCenterBoxwrtCenterPixel = (zDistance/focalLength)*centerBoxwrtCenterPixel; % in meters
            actualCameraTransf = cameraTransf * trvec2tform([0, 0.041, 0.0]);
            actualpartXY = actualCameraTransf(1:2,4)' + worldCenterBoxwrtCenterPixel;
            part.centerPoint = [actualpartXY(1),actualpartXY(2),part.Z];
            coordinator.Parts{i} = part;
        end
     end
    coordinator.NextPart = 0;
    if ~isempty(coordinator.Parts) && coordinator.NextPart<=length(coordinator.Parts)
        coordinator.DetectedParts = coordinator.Parts;
        % Trigger event 'partsDetected' on Stateflow
        coordinator.FlowChart.partsDetected;
        return;
    end
    coordinator.NumDetectionRuns = coordinator.NumDetectionRuns +1;

    % Trigger event 'noPartsDetected' on Stateflow
    coordinator.FlowChart.noPartsDetected; 
   
end