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
        ybtransform
        plot_ref
  
    end
    
    methods
        
        function obj = sim_youBot_TRS(sim,ident)
            
            obj = obj@sim_youBot(sim,ident);
            
            obj.hokuyo = sim.hokuyo('fastHokuyo');
            obj.rgbdcamera = sim.rgbdCamera('rgbdSensor');

            obj.ybtransform = obj.hokuyo.ref.pose(obj.ref.id);
            obj.plot_ref = obj.hokuyo.ref.position(obj.ref.id);
            
        end
        
        
        function [pnts,contacts] = hokuyo_scan(obj)
        % sim_youBot_TRS.hokuyo_scan
        %
        % Returns a full scan from the hokuyo. Retrives a hokuyo scan and
        % transforms it into the youBot's reference frame (sim_youBot.ref).
        %
        % Returns:
        %
        %  pnts         % 4-by-n array where n is 2*a. a = horizontal
        %                 resolution of the individual sensors. The 4 rows 
        %                 are x, y, z, distance. All coordinates are w.r.t 
        %                 sim_youBot.ref.  
        %  contacts     % A n-vector, where n = total number of pixels 
        %                 scanned. The n-th element corresponds to 
        %                 the n-th column of "pnts", and represents the state
        %                 of the "beam" (true = broken, false = continued to 
        %                 infinity.
        %
 
            
            [data,con] = obj.hokuyo.scan();
            
            pnts = homtrans(obj.ybtransform, data);
            contacts = con;
            
        end
        
        function im = get_image(obj)
        % sim_youBot_TRS.get_image
        %
        % Returns an RGB image from the RGBD Camera.
        %
        % Returns:
        %
        %   im                 % A h-by-v-by-3 image matrix.
        %
            
            im = obj.rgbdcamera.get_image();
        
        end
        
        function pnts = get_point_cloud(obj)
        % sim_youBot_TRS.get_point_cloud
        %
        % Gets a point cloud from the RGBD Camera.
        %
        % Returns:
        %
        %   data                      % 4 rows, w*h columns
        %                               The rows represet:
        %                               x of detected point (w.r.t. vision sensor)
        %                               y of detected point (w.r.t. vision sensor)
        %                               z of detected point (w.r.t. vision sensor)
        %                               dist to detected point
        %                               Each column is one point (pixel).
        %

            
            pnts = obj.rgbdcamera.get_point_cloud();
            
        end
        
        
    end
    
end

