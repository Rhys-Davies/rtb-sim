%% sim_force_sensor %%
% Class to represent torque/force sensors. Inherits from sim_entity.
%
% Methods:
%
%   torque          % Retreives 3 element torques vector acting on the 
%                     sensor in Newton meters.
%   broken          % Retrieves the state of the sensor. true = sensor
%                     broken, false = sensor intact.
%   force           % Retrieves 3 element linear force vector acting on the
%                     sensor in Newtons.
%

classdef sim_force_sensor < sim_entity


    properties
    end
    
    methods
        
        function obj = sim_force_sensor(sim,ident)
            
            obj = obj@sim_entity(sim,ident);
        
        end
        
        
        function trq = torque(obj)
        % sim_force_sensor.torque
        %
        % Returns the torque acting on the force sensor.
        %
        % Returns:
        %   
        %   trq         % A torque in Newton meters. 
        %
           
            [~,trq,~] = obj.readForceSensor(obj.id);
            
        end
        
        function brk = broken(obj)
        % sim_force_sensor.state
        %
        % Returns true if the force sensor has been broken. False if it is
        % still intact.
        %
        % Returns:
        %
        %   brk      % A logical value. 
        %
        
            [brk,~,~] = obj.readForceSensor(obj.id);
            
        end
        
        function frc = force(obj)
        % sim_force_sensor.force
        %
        % Returns the linear force acting on the force sensor.
        %
        % Returns:
        % 
        %   frc     % A force in Newtons
        %
            
            [~,~,frc] = obj.readForceSensor(obj.id);
        
        end
        
    end
    
end

