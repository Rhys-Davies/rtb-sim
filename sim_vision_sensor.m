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
%                         on which filters are active. dataIndex is a
%                         vector containing the starting indicies of each
%                         packet in dataPackets.
%                         
%   resolution          % Retrieves scan resolution of sensor.
%   fov                 % Retrieves the fov, field of view, of
%                         the sensor.
%
%   set_resolution      % Sets the sensor's resolution.
%   set_fov             % Sets the sensor's fov.
%


classdef sim_vision_sensor < sim_entity
   
   
    
    properties
    end
    
    methods
        
        function obj = sim_vision_sensor(sim,ident)
        
            obj = obj@sim_entity(sim,ident);
            
        end
        
        function [dataPackets, dataIndex] = get_data(obj)
        % sim_vision_sensor.get_data
        %
        % Retrieves packets of data from the sensor. The first packet,
        % containing 15 entries, is always the min, max and average of: 
        % Intensity, red, green, blue, and depth. 
        %
        % The data that follows is entirely dependant on the filter
        % components active. These filters can be found in a vision sensor
        % object's properties dialog. See the following V-REP documentation
        % page for more information. 
        % http://www.coppeliarobotics.com/helpFiles/en/visionSensorFilterComposition.htm
        %
        % Returns:
        %
        %   dataPackets             % The data returned by the vision
        %                             sensor.
        %   dataIndex               % A list of indicies that represent the
        %                             beginning of each packet.
        %
        
            [dataPackets, dataIndex] = obj.sim.readPointVisionSensor(obj.id);
            
        end
        
        function res = resolution(obj)
        % sim_vision_sensor.resolution
        %
        % Gets the current resolution of the sensor.
        %
        % Returns:
        %
        %   res         % A 2-vector containing the sensor's resolution in
        %                 the form [width, height]
        %
        
           res(1) = obj.get_IntParam(1002);
           res(2) = obj.get_IntParam(1003);

        end
        
        function f = fov(obj)
        % sim_sensor.fov
        %
        % Gets the FOV of the sensor.
        %
        % Returns:
        %
        %   f           % The sensor's FOV in radians.
        %
        
        f = obj.get_FloatParam(1004);
        

        
        end
        
        function set_resolution(obj,new)
        % sim_sensor.set_resolution(new)
        %
        % Sets the resolution of the sensor to 'new'
        % 
        % Arguments:
        %   new         % A 2-vector that contains the sensor's new resolution
        %                 in the form [width, height].
        %
        
            obj.set_IntParam(1002,new(1));
            obj.set_IntParam(1003,new(2));
            
        
        end
        
        function set_fov(obj,new)
        % sim_sensor.set_resolution(new)
        %
        % Sets the resolution of the sensor to 'new'
        % 
        % Arguments:
        %
        %   new         % An angle in radians.
        %
        

            obj.set_FloatParam(1004,new);
            
        end
        
        
    end
    
end