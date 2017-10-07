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


%% Notes: The Vehicle class as used by RTB EKF.
% This class models the kinematics of a car-like vehicle (bicycle model) on
% a plane that moves in SE(2).  For given steering and velocity inputs it
% updates the true vehicle state and returns noise-corrupted odometry
% readings.
%
% Methods::
%   init         initialize vehicle state
%   f            predict next state based on odometry
%   step         move one time step and return noisy odometry
%   control      generate the control inputs for the vehicle
%   update       update the vehicle state
%   run          run for multiple time steps
%   Fx           Jacobian of f wrt x
%   Fv           Jacobian of f wrt odometry noise
%   gstep        like step() but displays vehicle
%   plot         plot/animate vehicle on current figure
%   plot_xy      plot the true path of the vehicle
%   add_driver   attach a driver object to this vehicle
%   display      display state/parameters in human readable form
%   char         convert to string
%
% Class methods::
%   plotv        plot/animate a pose on current figure
%
% Properties (read/write)::
%   x               true vehicle state: x, y, theta (3x1)
%   V               odometry covariance (2x2)
%   odometry        distance moved in the last interval (2x1)
%   rdim             dimension of the robot (for drawing)
%   L               length of the vehicle (wheelbase)
%   alphalim        steering wheel limit
%   maxspeed        maximum vehicle speed
%   T               sample interval
%   verbose         verbosity
%   x_hist          history of true vehicle state (Nx3)
%   driver          reference to the driver object
%   x0              initial state, restored on init()


%%

classdef sim_youBot_TRS < sim_youBot
    
    properties
      
        hokuyo
        rgbdcamera

    end
    
    methods
        
        function obj = sim_youBot_TRS(sim,ident)
            
            obj = obj@sim_youBot(sim,ident);
            
           
            obj.hokuyo = sim.hokuyo('fastHokuyo',obj.ref);
            obj.rgbdcamera = sim.rgbdCamera('rgbdSensor'); % no stream

            
            
        end
        
        
        function res = hokuyo_scan()
        end
        
        function res = rgb_image()
        
        end
        
        function res = xyz_image()
        end
        
        
    end
    
end

