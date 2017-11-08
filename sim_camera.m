%% sim_camera %%
%
% A class to handle vision sensors that generate RGB or Greyscale images.
% Not to be confused with VREP Cameras, those are simply for viewing
% scenes, not retrieving images. Inherits from sim_vision_sensor.
%
% Properties
%
% Methods
%
%   get_image           % Retrieves an image from the camera. Will return a
%                         w*h*3 image matrix, or a w*h*1 image matrix if in
%                         greyscale mode. get_image(true) will return a
%                         greyscale image.
%   resolution          % Retrieves scan resolution of sensor. Returns [w,h]
%   fov                 % Retrieves the scan angle, or field of view, of
%                         the sensor.
%
%   set_resolution      % Sets the sensor's resolution.
%   set_fov             % Sets the sensor's scan angle.
%

classdef sim_camera < sim_vision_sensor

    
    properties
    end
    
    methods
        
        
        function obj = sim_camera(sim,ident)
        
            obj = obj@sim_vision_sensor(sim,ident);
            
        end
         
        function im = get_image(obj,grey)
        % sim_rgb_sensor.get_image
        %
        % Retrives a single frame from camera
        %
        % Arguments
        %   grey               % Boolean. When true a greyscale image will
        %                        be returned.
            if nargin < 2
                grey = false;
            else
                if ~islogical(grey)
                    error('sim_rgb_sensor: Argument grey must be logical');
                end
            end
            
            obj.sim.setIntegerSignal('handle_rgb_sensor',1);
            
            im = obj.sim.readVisionSensor(obj.id,grey);
            
        
        end
        
    end
    
end

