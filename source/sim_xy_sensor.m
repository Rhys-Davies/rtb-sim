%% XY Sensor Class %%
%
% A class to handle objects that generate a 2D point cloud data about the virtual
% world they are in. Inherits from entity class.
%
%
% Properties
%
% Methods
%
%   grab               % Retrieves data from from the sensor.




classdef sim_xy_sensor < sim_sensor
    
    properties
    end
    
    methods
        
        function obj = sim_xy_sensor(sim,ident,varargin)
         
            obj = obj@sim_sensor(sim,ident,varargin);
            % if streaming then
            %   obj.sim.setIntegerSignal('handle_xy_sensor',2);
            % end
            
            
        end
        
        function data = grab(obj)
        %% sim_xy_sensor.frame     
        % Gets a frame from the depth camera.
            
           % if not streaming
           obj.sim.setIntegerSignal('handle_xy_sensor',1);
           % end
           
           data = obj.sim.readPointVisionSensor(obj.id);
            
        end
        
        
        
    end
    
end



