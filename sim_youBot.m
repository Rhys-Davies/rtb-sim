%% sim_youBot %%
%
%  A class to represent the stock V-REP youBot.
%
%  Properties:
%   
%   arm                     % A sim_arm object representing the youBot's
%                             arm. Name formatting is: ('youBot','%sArmJoint%d') 
%   gripper                 % A sim_entity object representing the youBot's
%                             gripper.
%   gripper_target          % A sim_entity object representing the youBot's
%                             gripper target. This sets the target position
%                             for the arm tip when kinematic_mode = 2.
%   ref                     % A sim_entity object representing the youBot's
%                             central reference point.
%   arm_ref                 % A sim_entity object representing the youBot's
%                             arm's reference point on the robot.
%   wheels                  % An array of sim_joint objects that represt
%                             the youBot's four omniwheels. To work with
%                             sim_joint.setTargetVelocity, joint motors
%                             must be enabled and joint motor control
%                             disabled.
%
%  Methods:
%
%   move                    % Takes [x_dot, y_dot, theta_dot] and converts them into
%                             velocities for each individual wheel.
%
%   set_wheel_vel           % Takes a vector of velocities with length 4 
%                             and sets each wheel in the following order
%                             [fl,rl,rr,fr].
%
%   set_kinematicmode       % Sets kinematic mode for all joints with both 
%                             motors and motor control enabled. Valid modes
%                             are 0, 1, and 2.
%


classdef sim_youBot < sim_entity
    
    properties
        arm
        gripper
        gripper_target
        ref
        arm_ref
        wheels
    end
    
    methods
        
        function obj = sim_youBot(sim,ident)
            
            obj = obj@sim_entity(sim,ident);
            
            obj.arm = obj.sim.arm('youBot_ref','youBot','%sArmJoint%d');
            obj.arm.enable_control;
            obj.gripper = obj.sim.entity('youBot_gripperPositionTip');
            obj.gripper_target = obj.sim.entity('youBot_gripperPositionTarget');
            obj.ref = obj.sim.entity('youBot_center');
            obj.arm_ref = obj.sim.entity('youBot_ref');

            
            wjoints(1) = obj.sim.joint('rollingJoint_fl');
            wjoints(2) = obj.sim.joint('rollingJoint_rl');
            wjoints(3) = obj.sim.joint('rollingJoint_rr');
            wjoints(4) = obj.sim.joint('rollingJoint_fr');          
            obj.wheels = wjoints;
            
                       
            for i=1:4
                obj.wheels(i).enable_motor;
                obj.wheels(i).disable_control;
            end
            
        end
        
        function move(obj, x_dot, y_dot, theta_dot)
        % sim_youBot.move
        %
        % When given velocities in the x and y directions and a rotational
        % velocity, will set the wheel velocities as appropriate. Linear
        % velocities must be in m/s and angular in rad/s.
        %
        % Arguments:
        %
        %   x_dot           % Velocity in x direction. M/s
        %   y_dot           % Velocity in y direction. M/s
        %   theta_dot       % Rotational velocity. Rad/s
        %

        
            %x_dot = x_dot * 20;
            %y_dot = y_dot * 20;
            %theta_dot = theta_dot * 10;
        
            vel(1) = -x_dot-y_dot+theta_dot;
            vel(2) = -x_dot+y_dot+theta_dot;
            vel(3) = -x_dot-y_dot-theta_dot;
            vel(4) = -x_dot+y_dot-theta_dot;
            
            obj.setwheelvel(vel)
            
        end
        
        

        function setwheelvel(obj,vel)
        % sim_youBot.setwheelvel
        %
        % Sets velocities of all 4 wheels. Order of input vector is: 
        % [front_left, rear_left, rear_right, front_right.]
        %
        % Arguemnts:
        %
        %   vel         % 4-vector of velocities. Rad/s.   
        %
        
            obj.sim.pauseComms(true);

            for l=1:4
                obj.wheels(l).set_tgt_vel(vel(l));
            end

            obj.sim.pauseComms(false);
        
        end
        
        function setkinematicmode(obj,mode)
        % sim_youBot.setkinematicsmode
        %
        % Sets the kinematic mode signal. Must be enabled to use V-REP's IK
        % solver to move the arm.
        %
        % Arguments:
        %
        %   mode        % 2 for IK mode. 0 for disabled.
        %
   
           obj.sim.setIntegerSignal('km_mode', mode);
            
        end
        
        function open_gripper(obj)
        % sim_youBot.open_gripper
        %
        % Opens the youBot's gripper.
        %
        
            
            obj.sim.setIntegerSignal('gripper_open',1)
            
        end
        
        function close_gripper(obj)
        % sim_youBot.close_gripper
        %
        % Closes the youBot's gripper.
        %
            
            obj.sim.setIntegerSignal('gripper_open',0)
            
        end
        
        function ang = get_wheel_ang(obj)
        % sim_youBot.get_wheel_ang
        %
        % Retrieves the current wheel angles of all four wheels. Returns 4
        % vector with order [front_left, rear_left, rear_right, front_right.]
        % 
        % Returns:
        %
        %   ang         % Current wheel angles in radians. Will be value
        %                 from 0 to 2pi.
        %
        
            for i=1:4
               a(i) = obj.wheels(i).get_angle;
            end
            
            ang = a;
            
        end
        
    end
    
end

