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
        
        function obj = sim_xyz_sensor(sim,ident,varargin)
         
            obj = obj@sim_entity(sim,ident,varargin);
            
        end
        
        function data = grab(obj)
           
            obj.sim.setIntegerSignal('handle_xyz_sensor',1) 
            
            pause(1)
            
            data = obj.sim.readPointVisionSensor(obj.id);
            
            
        end
        
        
    end
    
end



