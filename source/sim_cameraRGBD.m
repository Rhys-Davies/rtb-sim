%% sim_cameraRGBD
% The TRS Task combination RGB/XYZ camera.
% ident = handle/ID of the camera body (eg. 'rgbdSensor')
%
% Properties
%
%   rgbsensor           % A sim_rgb_sensor object to handle the rgb camera
%   xyzsensor           % A sim_xyz_sensor object to handle the xyz camera
%
% Methods
%
%   image               % Takes an image from the rgb sensor
%                         TODO: Add argument to allow greyscale capture
%   point_cloud         % Captures a point_cloud from the xyz sensor 
%   

classdef sim_cameraRGBD < sim_entity
    
    properties
        rgbsensor
        xyzsensor
    end
    
    methods
        
        function obj = sim_cameraRGBD(sim,ident,varargin)
            
            obj = obj@sim_entity(sim,ident);
            obj.rgbsensor = obj.sim.rgb_sensor('rgbSensor');
            obj.xyzsensor = obj.sim.xyz_sensor('xyzSensor');
            
        end
        
        function im = image(obj)
            
            im = obj.rgbsensor.grab;
            
        end
        
        function pnts = point_cloud(obj)
            
            pnts = obj.xyzsensor.grab;
            
        end
        
    end
    
end

