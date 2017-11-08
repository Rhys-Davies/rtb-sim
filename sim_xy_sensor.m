%% XY Sensor Class %%
%
% A class to handle objects that generate a point cloud using a vision
% sensor that is triggered with the signal 'handle_xy_sensor'.
% Inherits from sim_vision_sensor.
%
%
% Properties
%
% Methods
%
%   scan               % Retrieves point cloud data from from the sensor.
%




classdef sim_xy_sensor < sim_vision_sensor
    
    properties
    end
    
    methods
        
        function obj = sim_xy_sensor(sim,ident)
         
            obj = obj@sim_vision_sensor(sim,ident);
            
        end
        
        function data = scan(obj)
        % sim_xy_sensor.frame     
        % Gets a frame from the depth camera.
            

            obj.sim.setIntegerSignal('handle_xy_sensor',1);

            [auxData, dataIndex] = obj.get_data();
            
                        
            % First 15 data entries are min, max and average of: Intensity,
            % red, green, blue, and depth. This is the first packet as defined
            % by dataIndex(1). Entries 16 and 17 are image height and width.
            
            pixel_count = auxData(dataIndex(1)+1)*auxData(dataIndex(1)+2);
            
            % dataIndex(1) + 2 + 1 the start of the sensor output data 
            % (second packet).
            % This data is in the form:
            % [x1, y1, z1, dist1, x2, y2, z2, dist2, ... xN, yN, zN, distN]
            % where N = w*h (pixels).
            
            data = reshape(auxData((dataIndex(1)+3):end), 4, pixel_count);
            
            % Data is now...
            % 4 rows, w*h columns
            % The rows represet:
            % x
            % y
            % z
            % dist
            % Each column is one point (pixel).
            
            
            %data = [shaped(1:2,:);shaped(4,:)]; 
            
            % Data is now...
            % 3 rows, w*h columns
            % x
            % y
            % dist
            % Each column is one point (pixel).
            
        end
        
        
        
    end
    
end



