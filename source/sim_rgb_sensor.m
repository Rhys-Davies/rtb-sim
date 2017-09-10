%% RGB Sensor Class %%
%
% A class to handle objects that generate image pixel data about the virtual
% world they are in. Inherits from entity class.
%
%
% Properties
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

classdef sim_rgb_sensor < sim_entity
    
    properties
    end
    
    methods
        
        function obj = sim_rgb_sensor(sim,ident,varargin)
        
            obj = obj@sim_entity(sim,ident,varargin);
            
        end
        
        function data = grab(obj,varargin)
        %% sim_rgb_sensor.grab
        % Retrives a single frame from camera
        %
        % Arguments
        %   grey               % Boolean. When true a greyscale image will
        %                        be returned.
           
            grey = false; %TODO
            
            obj.sim.setIntegerSignal('handle_rgb_sensor',1);
            
            data = obj.sim.readVisionSensor(obj.id,grey);
            
        end
        
        
    end
    
end

