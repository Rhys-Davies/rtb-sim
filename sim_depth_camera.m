%% sim_depth_sensor
%
% A depth camera that returns a greyscale depth image of the scene.
% Inherits from sim_camera.
%
% Properties
%
% Methods
%
%   get_image           % Retrieves depth image. Returns w*h*1 image
%                         matrix.
%   resolution          % Retrieves scan resolution of sensor. Returns [w,h]
%   fov                 % Retrieves the scan angle, or field of view, of
%                         the sensor.
%
%   set_resolution      % Sets the sensor's resolution.
%   set_fov             % Sets the sensor's scan angle.
%

classdef sim_depth_camera < sim_camera
    
    properties
    end
    
    methods
        function obj = sim_depth_camera(sim,ident)
        
            obj = obj@sim_camera(sim,ident);
            
        end
        
        function im = get_image(obj)
        % sim_depth_sensor.get_image
        %
        % Gets a greyscale depth image from a vision sensor.
        % 
        % Returns:
        %
        %   im          % An m*n image matrix, m = horizontal resolution
        %                 and n = verticle resolution.
        %
        
        
            im = obj.sim.readVisionSensorDepth(obj.id);
        
        end
        
    end
    
end

