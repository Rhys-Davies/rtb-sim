%% sim_proximity_sensor %%
%
% A V-REP proximity sensor. These sensors can return their state (false = 
% not triggered and true = triggered) and the coordinate of the detected 
% point relative to the sensor.   
%
% Properties
%
% Methods
%
%   read            % Retrieves both the state (triggered true or false)
%                     and the coordinates of the point that broke the
%                     "beam"
%   state           % Returns true if the sensor has been triggered,
%                     otherwise returns false.
%   point           % Coordinates of the detected point.
%




classdef sim_proximity_sensor < sim_entity

    properties
    end
    
    methods
        
        function obj = sim_proximity_sensor(sim,ident)
            
            obj = obj@sim_entity(sim,ident);
            
        end
        
        function [st,pnt] = read(obj)
            
            [st,pnt] = readProximitySensor(obj.id); 

        end
        
        function out = state(obj)
            
            [out,~] = readProximitySensor(obj.id); 
            
        end
        
        function out = point(obj)
            
            [~,out] = readProximitySensor(obj.id);
            
        end
        
    end
    
end

