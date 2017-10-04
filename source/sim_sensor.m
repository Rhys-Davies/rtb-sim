%% sim_sensor
%   A base class for vision sensors and laser sensors
%   simulated as vision sensors. 
%   This class is non-functional on its own, as most sensors need a
%   sensor specific signal to be sent to enable a capture.
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


classdef sim_sensor < sim_entity
   
   
    
    properties
    end
    
    methods (Abstract)
    
        data = grab(obj)
    
    end
    
    methods
        
        function obj = sim_sensor(sim,ident,varargin)
        
            obj = obj@sim_entity(sim,ident,varargin);
            
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
            
            % f = obj.getObjFloatParam('sim_visionfloatparam_perspective_angle',obj.id);
            f = obj.get_FloatParam(1004); % This will need to be something like obj.sim.getCamFOV or similar
        
        end
        
        function set_resolution(obj,new)
        %% sim_sensor.set_resolution(new)
        % Sets the resolution of the sensor to 'new'
        % 
        % Arguments
        %   new         % A 2-vector that contains [width, height]
        %
        
            obj.set_IntParam(1002,new(1));
            obj.set_IntParam(1003,new(2));
            
%             obj.setObjIntParam(obj.id,'sim_visionintparam_resolution_x',new(1));
%             obj.setObjIntParam(obj.id,'sim_visionintparam_resolution_y',new(2));
            
        
        end
        
        function set_fov(obj,new)
        %% sim_sensor.set_resolution(new)
        % Sets the resolution of the sensor to 'new'
        % 
        % Arguments
        %   new         % An angle in radians.
        %
        

            obj.set_FloatParam(1004,new);
            
        end
        
    end
    
end