classdef sim_force_sensor < sim_entity
%% sim_force_sensor
% Class to represent torque/force sensors.
%
% Methods:
%   read
%

    properties
    end
    
    methods
        function obj = sim_force_sensor(sim,ident)
            
            obj = obj@sim_entity(sim,ident);
        
        end
        
        
    end
    
end

