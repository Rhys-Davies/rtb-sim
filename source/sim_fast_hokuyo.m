%% sim_fast_hokuyo %%
% 
% Class to represent the V-REP fast Hokuyo object as used in the TRS Task 
% scene. This "LiDAR" is approximated with two vision sensors with a 
% vertical resolution of 1 pixel. Data is transformed into the reference 
% frame of 'faskHokuyo_ref'.
%
% Properties
%
%   ref         % Reference frame the scan data is transformed into.
%   sensor1     % sim_xy_sensor object representing sensor 1
%   sensor2     % sim_xy_sensor object representing sensor 2
%   range       % The maximum scan distance. All points outside this
%                 distance are discarded. Range = 5.
%   trans1      % Transform from sensor 1 reference frame to ref
%   trans2      % Transform from sensor 2 reference frame to ref
%   h1pos       % Position of sensor 1 w.r.t to ref
%   h2pos       % Position of sensor 2 w.r.t to ref
%   h1eul       % Orientation of sensor 1 w.r.t to ref
%   h2eul       % Orientation of sensor 2 w.r.t to ref
%
% Methods
%
%   scan        % Retrieve's a scan. Returns [pnts,strikes]. pnts = the 
%                 coordinates of all points scanned. strikes = a logical
%                 array containing the strike state for each point. (1 =
%                 beam was blocked, 0 = beam continued to infinity).
% 


classdef sim_fast_hokuyo < sim_entity
    
    properties
        
        ref
        sensor1
        sensor2
        range = 5 % Range of Hokuyo Sensor in meters
        trans1
        trans2
        h1pos
        h2pos
        h1eul
        h2eul

       
    end
    
    methods
        
        function obj = sim_fast_hokuyo(sim,ident)
            
            obj = obj@sim_entity(sim,ident);
            obj.sensor1 = obj.sim.xy_sensor('fastHokuyo_sensor1');
            obj.sensor2 = obj.sim.xy_sensor('fastHokuyo_sensor2'); 
            obj.ref = obj.sim.entity('fastHokuyo_ref');
            
            
             
            obj.h1pos = obj.sensor1.position(obj.ref.id); 
            obj.h2pos = obj.sensor2.position(obj.ref.id);
            
            obj.h1eul = obj.sensor1.orientation(obj.ref.id);
            obj.h2eul = obj.sensor2.orientation(obj.ref.id);
            
            obj.trans1 = transl(obj.h1pos) * trotx(obj.h1eul(1)) * troty(obj.h1eul(2)) * trotz(obj.h1eul(3)); %eul2tr(obj.h1eul)
            obj.trans2 = transl(obj.h2pos) * trotx(obj.h2eul(1)) * troty(obj.h2eul(2)) * trotz(obj.h2eul(3)); %eul2tr(obj.h2eul)
             
            obj.sim.setIntegerSignal('displaylasers', 1);
            
        end
        
        function [pnts,numdet] = scan(obj)
        
            p1 = obj.sensor1.scan;
            p2 = obj.sensor2.scan;
            
            obs1 = p1(4,:) < obj.range;
            obs2 = p2(4,:) < obj.range;
            
            p1 = p1(1:3,:);
            p2 = p2(1:3,:);  

            pnts  = [ homtrans(obj.trans1, p1), homtrans(obj.trans2, p2) ];
            numdet = [obs1, obs2];

        end
            
    end

    
end
    


