%% sim_trs_youBot
%  A class to represent the TRS modified youBot.
%  The stock VREP youBot does not contain the RGBD sensor or Hokuyo
%  The naming scheme for the wheels and arm joints is identicle however.
%
%  Properties:
%   
%   hokuyo                  % A sim_fast_hokuyo object representing the
%                             hokuyo laser scanner mounted to the TRS 
%                             version of the youBot. 
%   rgbdcamera              % A sim_rgdb_camera object that represents the
%                             combination xyz and rgb camera mounted to the
%                             TRS version of the youBot.
%
%  Methods:
%
%   hokuyo_scan             % Retrieves a Hokuyo scan. Alternatively could 
%                             call youBot.hokuyo.scan.
%   get_image               % Retrieves a RGB Image from the camera.
%   get_point_cloud         % Retrieves a point cloud from the camera.
%



classdef sim_youBot_TRS < sim_youBot
    
    properties
      
        hokuyo
        rgbdcamera

    end
    
    methods
        
        function obj = sim_youBot_TRS(sim,ident)
            
            obj = obj@sim_youBot(sim,ident);
            
           
            %obj.hokuyo = sim.hokuyo('fastHokuyo',obj.ref);
            obj.hokuyo = sim.hokuyo('fastHokuyo');
            obj.rgbdcamera = sim.rgbdCamera('rgbdSensor');

            
            
        end
        
        
        function [pnts,contacts] = hokuyo_scan(obj)
            
            [pnts,contacts] = obj.hokuyo.scan();
            
        end
        
        function im = get_image(obj)
            
            im = obj.rgbdcamera.get_image();
        
        end
        
        function pnts = get_point_cloud(obj)
            
            pnts = obj.rgbdcamera.get_point_cloud();
            
        end
        
        
    end
    
end

