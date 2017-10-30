%% sim_depth_sensor
% A depth camera that returns a greyscale image representation of relative
% depths of everything in the scene.
%
% Properties
%
% Methods
%
%   frame               % Retrieves data from from the sensor.
%   resolution          % Retrieves scan resolution of sensor.
%   fov                 % Retrieves the scan angle, or field of view, of
%                         the sensor.
%
%   set_resolution      % Sets the sensor's resolution.
%   set_fov             % Sets the sensor's scan angle.
%

classdef sim_depth_sensor < sim_sensor
    
    properties
    end
    
    methods
        function obj = sim_depth_sensor(sim,ident)
        
            obj = obj@sim_sensor(sim,ident);
            
        end
        
        function data = grab(obj)
        
            data = obj.sim.readVisionSensorDepth(obj.id);
        
        end
        
    end
    
end

