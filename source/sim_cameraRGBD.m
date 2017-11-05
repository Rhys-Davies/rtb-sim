%% sim_cameraRGBD %%
%
% This class handles he TRS Task combination RGB/XYZ camera.
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
            
            im = obj.rgbsensor.get_image;
            
        end
        
        function pnts = get_point_cloud(obj)
            
            pnts = obj.xyzsensor.scan;
            
        end
        
        function res = rgbFOV(obj)
            
            res = obj.rgbsensor.fov();
            
        end
        
        function res = xyzFOV(obj)
            
            res = obj.rgbsensor.fov();
            
        end
        
        function res = xyzResolution(obj)
        
            res = obj.xyzsensor.resolution();
            
        end
        
        function res = rgbResolution(obj)
            
            res = obj.rgbsensor.resolution();
        
        end
          
        function set_rgbFOV(obj,new)
            
            obj.rgbsensor.set_fov(new);
            
        end
        
        function set_xyzFOV(obj,new)
            
            obj.xyzsensor.set_fov(new);
            
        end
        
        function set_xyzResolution(obj,new)
            
            obj.xyzsensor.set_resolution(new);
        
        end
        
        function set_rgbResolution(obj,new)
            
            obj.rgbsensor.set_resolution(new);
            
        end
        
        
    end
    
end

