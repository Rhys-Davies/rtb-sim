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
%   hokuyo                  % A sim_fast_hokuyo object representing the
%                             hokuyo laser scanner mounted to the TRS 
%                             version of the youBot. 
%   rgbdcamera              % A sim_rgdb_camera object that represents the
%                             combination xyz and rgb camera mounted to the
%                             TRS version of the youBot.
%   wheels                  % An array of sim_joint objects that represt
%                             the youBot's four omniwheels. To work with
%                             sim_joint.setTargetVelocity, joint motors
%                             must be enabled and joint motor control
%                             disabled.
%                           
%   mo_ctrl                 % The youBot motion controller. See
%                             yb_motion_controller.m.
%
%  Methods:
%
%   drive                   % Calls the youBot's motion controller object.
%                             and applies the resulting velocities to the
%                             youBot's wheels.
%   set_kinematicmode       % Sets kinematic mode for all joints with both 
%                             motors and motor control enabled. Valid modes
%                             are 0, 1, and 2.



classdef sim_TRS_youBot < sim_entity
    
    properties
        arm
        gripper
        gripper_target
        ref
        arm_ref
        hokuyo
        rgbdcamera
        wheels
        mo_ctrl
    end
    
    methods
        
        function obj = sim_TRS_youBot(sim,ident)
        
            obj = obj@sim_entity(sim,ident);
            obj.arm = obj.sim.arm('youBot','%sArmJoint%d');
            obj.arm.enable_control;
            obj.gripper = obj.sim.entity('youBot_gripperPositionTip');
            obj.gripper_target = obj.sim.entity('youBot_gripperPositionTarget');
            obj.ref = obj.sim.entity('youBot_center');
            obj.arm_ref = obj.sim.entity('youBot_ref');
            obj.hokuyo = sim.hokuyo('fastHokuyo',obj.ref);
            obj.rgbdcamera = sim.rgbdCamera('rgbdSensor');
            obj.mo_ctrl = yb_motion_controller(20,12,10,4,0.05);
            
            wjoints(1) = obj.sim.joint('rollingJoint_fl',true); 
            wjoints(2) = obj.sim.joint('rollingJoint_rl',true); 
            wjoints(3) = obj.sim.joint('rollingJoint_rr',true); 
            wjoints(4) = obj.sim.joint('rollingJoint_fr',true);            
            obj.wheels = wjoints;
            
                       
            for l=1:4
                obj.wheels(l).enable_motor;
                obj.wheels(l).disable_control;
            end
            
        end
        
        function drive(obj, forwBackVel, leftRightVel, rotVel)
            
            vel = obj.mo_ctrl.update(forwBackVel, leftRightVel, rotVel);
            
            %obj.sim.pauseComms(1);
            for l=1:4
                obj.wheels(l).set_tgt_vel(vel(l));
            end
            %obj.sim.pauseComms(0);
            
        end
        
        
        %% This section may be more appropriate in sim_arm.
        function setkinematicmode(obj,mode)
   
           obj.sim.setIntegerSignal('km_mode', mode);
            
        end
        
        function open_gripper(obj)
            
            obj.sim.setIntegerSignal('gripper_open',1)
            
        end
        
        function close_gripper(obj)
            
            obj.sim.setIntegerSignal('gripper_open',0)
            
        end
        
    end
    
end

