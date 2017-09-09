%% XYZ Sensor Class %%
%
% A class to handle objects that generate point cloud data about the virtual
% world they are in. Inherits from entity class.
%
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



classdef sim_xyz_sensor < sim_entity
    
    properties
    end
    
    methods
        
        function obj = sim_xyz_sensor(sim,ident)
         
            obj = obj@sim_entity(sim,ident);
            
        end
        
        function pts = frame(obj)
        %% sim_xyz_sensor.frame     
        % Gets a frame from the depth camera.
        
           pts = obj.sim.readPointCloudSensor(obj.id);
            
        end
        
        function res = resolution(obj)
        %% sim_xyz_sensor.resolution
        % Gets the resolution of the sensor.
        
            res = obj.sim.getVisionSensorRes(obj.id);
        
        end
        
        function f = fov(obj)
        %% sim_xyz_sensor.fov
        % Gets the FOV of the sensor.
        
            f = obj.sim.getVisionSensorFOV(obj.id);
        
        end
        
        function set_resolution(obj,new)
        %% sim_xyz_sensor.set_resolution(new)
        % Sets the resolution of the sensor to 'new'
        % 
        % Arguments
        %   new         % A 2-vector that contains [width, height]
        %
        
            obj.sim.setVisionSensorRes(obj.id,new);
        
        end
        
        function set_fov(obj,new)
        %% sim_xyz_sensor.set_resolution(new)
        % Sets the resolution of the sensor to 'new'
        % 
        % Arguments
        %   new         % An angle in radians.
        %
            obj.sim.setVisionSensorFOV(obj.id,new);
        
        end
        
    end
    
end



