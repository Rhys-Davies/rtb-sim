%% XYZ Sensor Class %%
%
% A class to handle objects that generate point cloud data about the virtual
% world they are in. Inherits from entity class. Is tailored to the xyz
% sensor as found in the TRS Task youBot.
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



classdef sim_xyz_sensor < sim_entity
    
    properties
    end
    
    methods
        
        function obj = sim_xyz_sensor(sim,ident)
         
            obj = obj@sim_entity(sim,ident);
            
        end
        
        function pts = grab(obj)
           
            obj.sim.setIntegerSignal('handle_xyz_sensor',1) 
 
            [auxData, dataIndex] = obj.sim.readPointVisionSensor(obj.id);
            
                        
            % First 15 data entries are min max and average of: Intensity,
            % red, green, blue, and depth. This is the first packet as defined
            % by dataIndex(1). Entries 16 and 17 are image height and width.
            
            w = auxData(dataIndex(1)+1); % Image width (15 + 1)
            h = auxData(dataIndex(1)+2); % Image height (15 + 2)
            
            % dataIndex(1) + 2 + 1 the start of the sensor output data.
            % 4 rows, w*h columns
            % Each colum is x,y,z,dist
            
            %auxData(dataIndex(1)+3:end)
            
            pts = reshape(auxData((dataIndex(1)+3):end), 4, w*h);
            
            
        end
        
        
    end
    
end



