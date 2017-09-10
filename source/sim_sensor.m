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
        
        function obj = sim_sensor(sim,ident)
        
            obj = obj@sim_entity(sim,ident);
            
        end
        
        function res = resolution(obj)
        %% sim_sensor.resolution
        % Gets the resolution of the sensor.
        
           res(1) = obj.getIntegerParam('sim_visionintparam_resolution_x',new(1));
           res(2) = obj.getIntegerParam('sim_visionintparam_resolution_y',new(2));
            
%          res(1) = obj.setObjIntParam(obj.id,'sim_visionintparam_resolution_x',new(1));
%          res(2) = obj.setObjIntParam(obj.id,'sim_visionintparam_resolution_y',new(2));

        end
        
        function f = fov(obj)
        %% sim_sensor.fov
        % Gets the FOV of the sensor.
            
            % f = obj.getObjFloatParam('sim_visionfloatparam_perspective_angle',obj.id);
            f = obj.getFloatParam('sim_visionfloatparam_perspective_angle');
        
        end
        
        function set_resolution(obj,new)
        %% sim_sensor.set_resolution(new)
        % Sets the resolution of the sensor to 'new'
        % 
        % Arguments
        %   new         % A 2-vector that contains [width, height]
        %
        
            obj.setIntegerParam('sim_visionintparam_resolution_x',new(1));
            obj.setIntegerParam('sim_visionintparam_resolution_y',new(2));
            
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
        
            % obj.setObjFloatParam('sim_visionfloatparam_perspective_angle',obj.id);
            obj.setFloatParam('sim_visionfloatparam_perspective_angle',new);
            
        end
        
    end
    
end