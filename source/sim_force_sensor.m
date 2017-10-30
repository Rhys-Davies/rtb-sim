classdef sim_force_sensor < sim_entity
%% sim_force_sensor
% Class to represent torque/force sensors.
%
% Methods:
%
%   torque          %
%   state           %
%   force           %
%

    properties
    end
    
    methods
        
        function obj = sim_force_sensor(sim,ident)
            
            obj = obj@sim_entity(sim,ident);
        
        end
        
        
        function res = torque(obj)
        % sim_force_sensor.torque
        % Returns the angular force being exerted on the force sensor 
        %
           
            [~,res,~] = obj.readForceSensor(obj.id);
            
        end
        
        function res = state(obj)
        % sim_force_sensor.state
        % Returns true if the force sensor has been broken. False if it is
        % still intact.
        %
        
            [res,~,~] = obj.readForceSensor(obj.id);
            
        end
        
        function res = force(obj)
        % sim_force_sensor.force
        % Returns the linear force acting on the force sensor.
        %
            
            [~,~,res] = obj.readForceSensor(obj.id);
        
        end
        
    end
    
end

