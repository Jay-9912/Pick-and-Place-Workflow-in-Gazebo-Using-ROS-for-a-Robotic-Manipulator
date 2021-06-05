function exampleCommandComputeGraspPoseROSGazebo(coordinator) 

%   Compute the grasp pose for a part with a known pose
%   This command computes the task-space grasping pose required for the
%   manipulator to pick up a part. The grasp pose is based on the pose of
%   the part to be picked up, which is saved as part.centerPoint during the object detection.
%
% Copyright 2020 The MathWorks, Inc.

        %   This functioncomputes the grasping pose by applying a
        %   pre-defined transform relative the pose of the object. This
        %   could be replaced by more advanced methods. For example,
        %   the grasp pose could be found using point cloud data
        %   together with machine / deep learning based on object
        %   poses.

        coordinator.GraspPose = trvec2tform(coordinator.Parts{coordinator.NextPart}.centerpoint(1:3)' + [0 0 0])*axang2tform([0 1 0 pi/2])*axang2tform([0 0 1 pi/2]);
        %coordinator.GraspPose(3,4)=0.465;
        % Trigger Stateflow chart Event
        coordinator.FlowChart.nextAction; 
end