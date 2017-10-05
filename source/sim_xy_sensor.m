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
        
        function obj = sim_xy_sensor(sim,ident)
         
            obj = obj@sim_sensor(sim,ident);

            
            
        end
        
        function data = grab(obj)
        %% sim_xy_sensor.frame     
        % Gets a frame from the depth camera.
            

           obj.sim.setIntegerSignal('handle_xy_sensor',1);

           data = obj.sim.readPointVisionSensor(obj.id);
            
        end
        
        
        
    end
    
end



