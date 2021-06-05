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
     disp('1');
     % Read image from simulated Gazebo camera
     rgbImg = readImage(coordinator.ROSinfo.rgbImgSub.LatestMessage);
     depthImg = readImage(coordinator.ROSinfo.pointCloudSub.LatestMessage);
     centerPixel = [round(size(rgbImg,2)/2), round(size(rgbImg,1)/2)];
     disp(centerPixel);
     % Detect parts and show labels
     % figure;
     imshow(rgbImg);
     [bboxes,scores,labels]=percept(coordinator.DetectorModel,rgbImg);
     disp(bboxes);
     %[bboxes,~,labels] = detect(coordinator.DetectorModel,rgbImg); % 检测出物体
     tftree = rostf;
     pause(1);
     camera_transf = getTransform(tftree,'base_link','camera_link');
     camera_transl = camera_transf.Transform.Translation;
     camera_translations = [camera_transl.X,camera_transl.Y,camera_transl.Z];
     camera_rotation = camera_transf.Transform.Rotation;
     camera_quaternions = [camera_rotation.W, camera_rotation.X, camera_rotation.Y, camera_rotation.Z];
     rotm = quat2rotm(camera_quaternions);
     translVect = camera_translations';
     fixedRotation = eul2rotm([0 pi 0], "XYZ");
     rotm = rotm*fixedRotation;
     tform = [rotm translVect; 0 0 0 1];
     
     if ~isempty(labels) 
        labeledImg = insertObjectAnnotation(rgbImg,'Rectangle',bboxes,cellstr(labels));
        imshow(labeledImg); % 显示bbox           
        numObjects = size(bboxes,1); % 物体数量
        allLabels =table(labels);
        %需要修改
        for i=1:numObjects
            %disp(bboxes(i,2)+round(bboxes(i,4)/2));
            zDistance = depthImg(round(bboxes(i,2)+bboxes(i,4)/2),round(bboxes(i,1)+bboxes(i,3)/2));
            centerBox = [bboxes(i,1)+ bboxes(i,3)/2, bboxes(i,2)+ bboxes(i,4)/2];
            centerBoxwrtCenterPixel = centerBox - centerPixel; % in pixels
            xycamera = (zDistance/focalLength)*centerBoxwrtCenterPixel;
            point_camera = [xycamera(1),xycamera(2),zDistance,1];
            point_base = tform * point_camera'; % base coordinate
            part.centerpoint = point_base;
            if allLabels.labels(i)=='red'
                % Height of objects is known according to type
                %part.Z = 0.052;
                part.type = 2;
            else  % yellow
                %part.Z = 0.17;
                part.type = 1;
            end
%             cameraTransf = getTransform(coordinator.Robot, coordinator.CurrentRobotJConfig, 'EndEffector_Link');
%             cameraZ = cameraTransf(3,4);
%             zDistance = cameraZ - part.Z;
%             centerBox = [bboxes(i,2)+ round(bboxes(i,4)/2), bboxes(i,1)+ round(bboxes(i,3)/2)];
%             centerBoxwrtCenterPixel = centerBox - centerPixel; % in pixels
%             worldCenterBoxwrtCenterPixel = (zDistance/focalLength)*centerBoxwrtCenterPixel; % in meters
%             actualCameraTransf = cameraTransf * trvec2tform([0, 0.041, 0.0]);
%             actualpartXY = actualCameraTransf(1:2,4)' + worldCenterBoxwrtCenterPixel;
%             part.centerPoint = [actualpartXY(1),actualpartXY(2),part.Z];
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
    %coordinator.FlowChart.noPartsDetected; 
   
end
function [Bbox,scores,labels]=percept(model,I)
    inputSize=[224 224 3];
    imageSize=size(I);
    scale=imageSize(1:2)./inputSize(1:2);
    
    I = imresize(I,inputSize(1:2));
    [bboxes,scores,labels]  = detect(model,I);
    Bbox=[bboxes(:,1)*scale(2),bboxes(:,2)*scale(1),bboxes(:,3)*scale(2),bboxes(:,4)*scale(1)];
    
end