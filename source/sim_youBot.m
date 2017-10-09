%% Class SIM_TRS_YOUBOT
%  A class to represent the TRS modified youBot.
%  The stock VREP youBot does not contain the RGBD sensor or Hokuyo
%  The naming scheme for the wheels and arm joints is identicle however.
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
%   move                    % Takes x_dot, y_dot, theta_dot and converts them into
%                             velocities for each individual wheels.
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
            
            obj.arm = obj.sim.arm('youBot','%sArmJoint%d');
            obj.arm.enable_control;
            obj.gripper = obj.sim.entity('youBot_gripperPositionTip');
            obj.gripper_target = obj.sim.entity('youBot_gripperPositionTarget');
            obj.ref = obj.sim.entity('youBot_center');
            obj.arm_ref = obj.sim.entity('youBot_ref');
            obj.hokuyo = sim.hokuyo('fastHokuyo',obj.ref);
            obj.rgbdcamera = sim.rgbdCamera('rgbdSensor'); % no stream
            
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
            
            vel(1) = -x_dot-y_dot+theta_dot;
            vel(2) = -x_dot+y_dot+theta_dot;
            vel(3) = -x_dot-y_dot-theta_dot;
            vel(4) = -x_dot+y_dot-theta_dot;
            
            obj.setwheelvel(vel)
            
        end
        
        

        function setwheelvel(obj,vel)
        
            obj.sim.pauseComms(true);

            for l=1:4
                obj.wheels(l).set_tgt_vel(vel(l));
            end

            obj.sim.pauseComms(false);
        
        end
        
        function setkinematicmode(obj,mode)
   
           obj.sim.setIntegerSignal('km_mode', mode);
            
        end
        
        function open_gripper(obj)
            
            obj.sim.setIntegerSignal('gripper_open',1)
            
        end
        
        function close_gripper(obj)
            
            obj.sim.setIntegerSignal('gripper_open',0)
            
        end
        
        function ang = get_wheel_ang(obj)
        
            for i=1:4
               a(i) = obj.wheels(i).angl;
            end
            
            ang = a;
            
        end
        
    end
    
end

