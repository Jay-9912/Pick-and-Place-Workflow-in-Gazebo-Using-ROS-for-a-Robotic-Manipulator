function exampleCommandActivateGripperROSGazebo(coordinator, state)% This class is for internal use and may be removed in a future release
%
% Command function to activate gripper  
%   This command activates the gripper by sending a ROS action command. 

% Copyright 2020 The MathWorks, Inc.

        %   Based on the state, decide whether to activate or
        %   deactivate the gripper
       if strcmp(state,'on') == 1
           % Activate gripper
            pause(1);
            [gripAct,gripGoal] = rosactionclient('/my_gen3/custom_gripper_controller/gripper_cmd');
            gripperCommand = rosmessage('control_msgs/GripperCommand');
            gripperCommand.Position = 0.04; % 0.04 fully closed, 0 fully open
            gripperCommand.MaxEffort = 500;
            gripGoal.Command = gripperCommand;            
            pause(1);
            
            % Send command
            sendGoal(gripAct,gripGoal); 
            disp('Gripper closed...');
       else
           % Deactivate gripper
            pause(1);
            [gripAct,gripGoal] = rosactionclient('/my_gen3/custom_gripper_controller/gripper_cmd');
            gripperCommand = rosmessage('control_msgs/GripperCommand');
            gripperCommand.Position = 0.0; % 0.04 fully closed, 0 fully open
            gripperCommand.MaxEffort = 500;
            gripGoal.Command = gripperCommand;            
            pause(1);
            
            % Send command
            sendGoal(gripAct,gripGoal);
            disp('Gripper open...');
       end
       
       pause(2);
       
       % Trigger Stateflow chart Event
       coordinator.FlowChart.nextAction; 
end