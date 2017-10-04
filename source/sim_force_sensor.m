classdef sim_force_sensor < sim_entity
%% sim_force_sensor
% Class to represent torque/force sensors.
%
% Methods:
%
%   torque
%   state
%   force
%

    properties
    end
    
    methods
        
        function obj = sim_force_sensor(sim,ident)
            
            obj = obj@sim_entity(sim,ident);
        
        end
        
        
        function res = torque(obj)
           
            [~,res,~] = obj.readForceSensor(obj.id);
            
        end
        
        function res = state(obj)
        
            [res,~,~] = obj.readForceSensor(obj.id);
            
        end
        
        function res = force(obj)
            
            [~,~,res] = obj.readForceSensor(obj.id);
        
        end
        
    end
    
end

