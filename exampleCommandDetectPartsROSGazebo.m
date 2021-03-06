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
     width = size(rgbImg,2);
     % Detect parts and show labels
     % figure;
     imshow(rgbImg);
     [bboxes,~,labels]=percept(coordinator.DetectorModel,rgbImg);
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
     minindex = 1;
     if ~isempty(labels) 
        labeledImg = insertObjectAnnotation(rgbImg,'Rectangle',bboxes,cellstr(labels));
        imshow(labeledImg); % 显示bbox           
        numObjects = size(bboxes,1); % 物体数量
        allLabels =table(labels);
        for i=1:numObjects
            %disp(bboxes(i,2)+round(bboxes(i,4)/2));
            breakflag = 0;
            leftbound = 1;
            rightbound = width;
            leftpoint = depthImg(round(bboxes(i,2)+bboxes(i,4)/2),round(bboxes(i,1)+bboxes(i,3)/3));
            rightpoint = depthImg(round(bboxes(i,2)+bboxes(i,4)/2),round(bboxes(i,1)+bboxes(i,3)*2/3));
            if abs(leftpoint-rightpoint)>0.08
                continue
            end
            y = round(bboxes(i,2)+bboxes(i,4)/2);
            x = round(bboxes(i,1)+bboxes(i,3)/2);
            p = x;
            while p > 1
                if depthImg(y,p)-depthImg(y,p-1)>0.08
                    breakflag = 1;
                    break
                end
                if depthImg(y,p-1)-depthImg(y,p)>0.08
                    leftbound = p;
                    break
                end
                p = p-1;
            end
            if breakflag == 1
                continue
            end
            q = x;
            while q < width
                if depthImg(y,q)-depthImg(y,q+1)>0.08
                    breakflag = 1;
                    break
                end
                if depthImg(y,q+1)-depthImg(y,q)>0.08
                    rightbound = q;
                    break
                end
                q = q+1;
            end
            if breakflag == 1
                continue
            end
            zDistance = depthImg(y,round((leftbound+rightbound)/2));
            centerBox = [round((leftbound+rightbound)/2), y];
            centerBoxwrtCenterPixel = centerBox - centerPixel; % in pixels
            xycamera = (zDistance/focalLength)*centerBoxwrtCenterPixel;
            point_camera = [xycamera(1),xycamera(2),zDistance,1];
            point_base = tform * point_camera'; % base coordinate
            part.centerpoint = point_base;
            if allLabels.labels(i)=='can'
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
        minz = 10;
        
        for i = 1:numObjects
            if isempty(coordinator.Parts{i})
                continue
            end
            if coordinator.Parts{i}.centerpoint(1) < minz
                minz = coordinator.Parts{i}.centerpoint(1);
                minindex = i;
            end
        end
        
            
     end
         
    coordinator.NextPart = 0;
    if ~isempty(coordinator.Parts) %&& coordinator.NextPart<=length(coordinator.Parts)
        coordinator.DetectedParts = coordinator.Parts{minindex};
        % Trigger event 'partsDetected' on Stateflow
        coordinator.FlowChart.partsDetected;
        return;
    end
    coordinator.NumDetectionRuns = coordinator.NumDetectionRuns +1;

    % Trigger event 'noPartsDetected' on Stateflow
    coordinator.FlowChart.noPartsDetected; 
   
end
function [Bbox,scores,labels]=percept(model,I)
    inputSize=[224 224 3];
    imageSize=size(I);
    scale=imageSize(1:2)./inputSize(1:2);
    
    I = imresize(I,inputSize(1:2));
    [bboxes,scores,labels]  = detect(model,I);
    Bbox=[bboxes(:,1)*scale(2),bboxes(:,2)*scale(1),bboxes(:,3)*scale(2),bboxes(:,4)*scale(1)];
    
end