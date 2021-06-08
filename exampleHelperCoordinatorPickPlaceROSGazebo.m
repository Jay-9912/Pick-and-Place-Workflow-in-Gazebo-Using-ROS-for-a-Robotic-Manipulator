classdef exampleHelperCoordinatorPickPlaceROSGazebo < handle
% This class is for internal use and may be removed in a future release
%
%exampleHelperCoordinatorPickPlaceROSGazebo Class used to run the Stateflow
%chart and setup ROS communication with Gazebo
%   This class is used to control the pick-and-place workflow execution.
%   The class serves two main purposes:
%      1. It holds information about ROS-MATLAB communication (subscribers,
%      publishers etc.)
%      2. It stores all data about the current pick and place job
%
% Copyright 2020 The MathWorks, Inc.

    properties         
        FlowChart
        Robot
        World = {};
        Parts = {};
        Obstacles = {};
        ObstaclesVisible = {};
        DetectedParts = {};
        RobotEndEffector
        CurrentRobotJConfig
        CurrentRobotTaskConfig
        NextPart = 0;
        PartOnRobot = 0;
        HomeRobotTaskConfig 
        PlacingPose
        GraspPose
        Figure
        TimeStep
        MotionModel
        NumJoints
        NumDetectionRuns = 0;
        CollisionHelper
        ROSinfo
        DetectorModel
    end
    
    methods
        function obj = exampleHelperCoordinatorPickPlaceROSGazebo(robot, initialRobotJConfig, robotEndEffector)
            obj.Robot = robot;
            
            % Initialize ROS utilities
            obj.ROSinfo.jointsSub = rossubscriber('/my_gen3/joint_states');
            obj.ROSinfo.configClient = rossvcclient('/gazebo/set_model_configuration');
            obj.ROSinfo.gazeboJointNames = {'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7'}; % joint names of robot model in GAZEBO
            obj.ROSinfo.controllerStateSub = rossubscriber('/my_gen3/gen3_joint_trajectory_controller/state');
            obj.ROSinfo.rgbImgSub = rossubscriber('/my_gen3/camera/rgb/image_raw');
            obj.ROSinfo.pointCloudSub = rossubscriber('/my_gen3/camera/depth/image_raw');
            % Initialize robot configuration in GAZEBO
            configResp = setCurrentRobotJConfig(obj, initialRobotJConfig);
            
            % Unpause GAZEBO physics
            physicsClient = rossvcclient('gazebo/unpause_physics');
            physicsResp = call(physicsClient,'Timeout',3);
            
            % Update robot properties
            obj.CurrentRobotJConfig = getCurrentRobotJConfig(obj);
            obj.RobotEndEffector = robotEndEffector;
            obj.CurrentRobotTaskConfig = getTransform(obj.Robot, obj.CurrentRobotJConfig, obj.RobotEndEffector);
            obj.TimeStep = 0.01; % used by Motion Planner
            obj.NumJoints = numel(obj.CurrentRobotJConfig);
            
            % Load deep learning model for object detection
            temp = load('task_2_model_2.mat'); % 替换成自己的
            obj.DetectorModel = temp.detector;                     
        end
        
        function JConfig = getCurrentRobotJConfig(obj)
            jMsg = receive(obj.ROSinfo.jointsSub);
            JConfig =  jMsg.Position(2:8)';
        end
        
        function configResp = setCurrentRobotJConfig(obj, JConfig)            
            configReq = rosmessage(obj.ROSinfo.configClient);
            configReq.ModelName = "my_gen3";
            configReq.UrdfParamName = "/my_gen3/robot_description";
            configReq.JointNames = obj.ROSinfo.gazeboJointNames;
            configReq.JointPositions = JConfig; 
            configResp = call(obj.ROSinfo.configClient, configReq, 'Timeout', 3);
        end
        
        function isMoving = getMovementStatus(obj)
            statusMsg = receive(obj.ROSinfo.controllerStateSub);
            velocities = statusMsg.Actual.Velocities;
            if all(velocities<0.03)
                isMoving = 0;
            else
                isMoving = 1;
            end
        end
        
        % Display current job state
        function displayState(obj, message)
            disp(message);
            set(obj.Figure, 'NumberTitle', 'off', 'Name', message)
        end
        
        % Delete function
        function delete(obj)
            delete(obj.FlowChart)
        end
            
    end
  
end

