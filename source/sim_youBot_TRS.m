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
%


classdef sim_youBot_TRS < sim_youBot
    
    properties
      
        hokuyo
        rgbdcamera

    end
    
    methods
        
        function obj = sim_youBot_TRS(sim,ident)
            
            obj = obj@sim_youBot(sim,ident);
            
           
            obj.hokuyo = sim.hokuyo('fastHokuyo',obj.ref);
            obj.rgbdcamera = sim.rgbdCamera('rgbdSensor');

            
            
        end
        
        
        function res = hokuyo_scan()
            
            
            
        end
        
        function res = rgb_image()
            
            
        
        end
        
        function res = xyz_image()
            
            
            
        end
        
        
    end
    
end

