%% sim_sensor
%   A base class for vision sensors and laser sensors
%   simulated as vision sensors. 
%
%   %TODO: Move common Methods and Properties from:
%          sim_rgb_sensor
%          sim_xyz_sensor
%          sim_xy_sensor
%
%   % The above classes can then be made to inherit from this class.
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


classdef sim_sensor < sim_entity
   
   
    
    properties
    end
    
    methods
        
        function obj = sim_sensor(sim,ident)
        
            obj = obj@sim_entity(sim,ident);
            
        end
        
        
    end
    
end

