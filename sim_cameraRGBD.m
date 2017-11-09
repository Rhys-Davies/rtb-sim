%% sim_cameraRGBD
%
% This class handles the TRS Task combination RGB/XYZ camera.
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
%                        
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
            obj.rgbsensor = obj.sim.camera('rgbSensor');
            obj.xyzsensor = obj.sim.xyz_sensor('xyzSensor');
            
        end
        
        function im = get_image(obj)
        % sim_cameraRGBD.get_image
        %
        % Retrives an RGB image from the camera's RGB vision sensor.
        %
            
            im = obj.rgbsensor.get_image;
            
        end
        
        function pnts = get_point_cloud(obj)
        % sim_cameraRGBD.get_point_cloud
        %
        % Retrives a point cloud image from the camera's point cloud sensor. 
        %
            
            pnts = obj.xyzsensor.scan;
            
        end
        
        function res = rgbFOV(obj)
        % sim_cameraRGBD.rgbFOV
        %
        % Retrives the current FOV of the camera's RGB vision sensor.
        % Value is retuned in radians.
        %
            
            res = obj.rgbsensor.fov();
            
        end
        
        function res = xyzFOV(obj)
        % sim_cameraRGBD.xyzFOV
        %
        % Retrives the current FOV of the camera's point cloud sensor.
        % Value is retuned in radians.
        % 
            
            
            res = obj.rgbsensor.fov();
            
        end
        
        function res = xyzResolution(obj)
        % sim_cameraRGBD.xyzResolution
        %
        % Retrives the current resolution of the camera's point cloud sensor.
        % 
        
            res = obj.xyzsensor.resolution();
            
        end
        
        function res = rgbResolution(obj)
        % sim_cameraRGBD.rgbResolution
        %
        % Retrives the current resolution of the camera's RGB vision sensor.
        % 
        
            
            res = obj.rgbsensor.resolution();
        
        end
          
        function set_rgbFOV(obj,new)
        % sim_cameraRGBD.set_rgbFOV
        %
        % Sets the FOV of the camera's RGB vision sensor. New angle must be
        % in radians.
        % 
        
            obj.rgbsensor.set_fov(new);
            
        end
        
        function set_xyzFOV(obj,new)
        % sim_cameraRGBD.set_xyzFOV
        %
        % Sets the FOV of the camera's point cloud vision sensor. New angle
        % must be in radians.
        % 
            
            obj.xyzsensor.set_fov(new);
            
        end
        
        function set_xyzResolution(obj,new)
        % sim_cameraRGBD.set_xyzResolution
        %
        % Sets the resolution of the camera's point cloud vision sensor.
        % 
                  
            obj.xyzsensor.set_resolution(new);
        
        end
        
        function set_rgbResolution(obj,new)
        % sim_cameraRGBD.set_rgbResolution
        %
        % Sets the resolution of the camera's RGB vision sensor.
        %
            
            obj.rgbsensor.set_resolution(new);
            
        end
        
        
    end
    
end

