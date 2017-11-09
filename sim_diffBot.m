classdef sim_diffBot < sim_entity
%% sim_diffBot %%
%
% A class to represent the beautiful piece of art that is the diffBot in
% demoScene2. A simple differential drive robot with a camera. Encoders are
% cumulatively count up and never wrap. The count is handled server (V-REP)
% side and the value published to signals. See the child script attached to
% the diffBot in the scene.
%
% Properties
%
%   wheels          % 2 element array of joint objects representing the
%                     wheels.
%   rgbdcamera      % A sim_cameraRGBD object representing the same RGBD
%                     camera as found in the TRS Task.
%
% Methods
%
%   setMotorVel     % A two element array containing target wheel
%                     velocities in rads/sec.
%   getImage        % Retrieve an RGB Image from the camera.
%   getEncoder      % Retrieve wheel encoder readings in radians. Reading is
%                     cumulative and doesn't wrap. Returns a 2-vector.
%
    
    properties
        
        wheels
        rgbdcamera
        
    end
    
    methods
        
        function obj = sim_diffBot(sim,ident)
    
            obj = obj@sim_entity(sim,ident);
            obj.rgbdcamera = sim.rgbdCamera('rgbdSensor');
            wheels(1) = sim.joint('diffBot_rightMotor');
            wheels(2) = sim.joint('diffBot_leftMotor');
            obj.wheels = wheels;
        end
        
        function setMotorVel(obj,new)
        % diffBot.setMotorVel
        %
        % Sets the wheel motors to a velocity. Velocity must be expressed
        % as radians/sec, and given in the form [right_motor,left_motor].
        %
        
            obj.sim.pauseComms(true);
            obj.wheels(1).set_tgt_vel(new(1))
            obj.wheels(2).set_tgt_vel(new(2))
            obj.sim.pauseComms(false);
        end
        
        function im = getImage(obj)
        % diffBot.getImage
        %
        % Gets an RGB image from the diffbot.
        %
        
            im = obj.rgbdcamera.image;
        
        end
        
        function enc = getEncoder(obj)
        % diffBot.getEncoder
        %
        % Gets an encoder reading. Encoder cumulatively counts degrees 
        % rotated by the wheel, and never overflows/wraps. Value is returned
        % in radians. The cumulative counting of the joint angle is done in 
        % a V-REP child script attached to the diffBot in V-REP. 
        %
        % Returns [right_enc, left_enc]
        
             a(1) = obj.sim.getFloatSignal('r_encoder');
             a(2) = obj.sim.getFloatSignal('l_encoder');
             enc = a;
             
        end 
        
    end
        
end

