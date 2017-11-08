%% sim_vision_sensor
%
%   A base class for V-REP vision sensors. This includes sensors used for 
%   fast "laser" sensors, which are simulated as vision sensors. Inherits
%   from sim_entity.
%
% Methods
%
%   get_data            % Returns [dataPackets, dataIndex], raw sensor data.
%                         The contents of dataPackets will depend entirely
%                         on which filters are active.
%   resolution          % Retrieves scan resolution of sensor.
%   fov                 % Retrieves the scan angle, or field of view, of
%                         the sensor.
%
%   set_resolution      % Sets the sensor's resolution.
%   set_fov             % Sets the sensor's scan angle.
%


classdef sim_vision_sensor < sim_entity
   
   
    
    properties
    end
    
    methods
        
        function obj = sim_vision_sensor(sim,ident)
        
            obj = obj@sim_entity(sim,ident);
            
        end
        
        function [dataPackets, dataIndex] = get_data(obj)
        
            [dataPackets, dataIndex] = obj.sim.readPointVisionSensor(obj.id);
            
        end
        
        function res = resolution(obj)
        %% sim_sensor.resolution
        % Gets the resolution of the sensor.
        
           res(1) = obj.get_IntParam(1002);
           res(2) = obj.get_IntParam(1003);

        end
        
        function f = fov(obj)
        %% sim_sensor.fov
        % Gets the FOV of the sensor.
        
        f = obj.get_FloatParam(1004);
        

        
        end
        
        function set_resolution(obj,new)
        %% sim_sensor.set_resolution(new)
        % Sets the resolution of the sensor to 'new'
        % 
        % Arguments:
        %   new         % A 2-vector that contains [width, height]
        %
        
            obj.set_IntParam(1002,new(1));
            obj.set_IntParam(1003,new(2));
            
        
        end
        
        function set_fov(obj,new)
        %% sim_sensor.set_resolution(new)
        % Sets the resolution of the sensor to 'new'
        % 
        % Arguments:
        %   new         % An angle in radians.
        %
        

            obj.set_FloatParam(1004,new);
            
        end
        
        
    end
    
end