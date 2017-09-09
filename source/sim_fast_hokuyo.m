classdef sim_fast_hokuyo < sim_entity
    %SIM_HYOKU Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        sensor1
        sensor2
        range = 4.999 % Range of Hokuyo Sensor
        transform
        trans1
        trans2
        h1pos
        h2pos
        h1eul
        h2eul
       
    end
    
    methods
        
        function obj = sim_fast_hokuyo(sim,ident,ref)
            
            obj = obj@sim_entity(sim,ident);
            obj.sensor1 = obj.sim.xy_sensor('fastHokuyo_sensor1',true);
            obj.sensor2 = obj.sim.xy_sensor('fastHokuyo_sensor2',true);  
            
            if nargin < 3
                obj.transform = false;
                obj.h1pos = obj.sensor1.position; 
                obj.h2pos = obj.sensor2.position;
                obj.h1eul = obj.sensor1.orientation;
                obj.h2eul = obj.sensor2.orientation;
            else
                obj.transform = true;
                if isa(ref,'sim_entity')
                    [obj.trans1,obj.trans2] = obj.calculateTransform(ref);
                else
                    error('Invalid Reference Object');
                end
            end
            
            obj.sim.setIntegerSignal('displaylasers', 1);
            
        end
        
        function [pnts,numdet] = scan(obj)
        
            p1 = obj.sensor1.frame;
            p2 = obj.sensor2.frame;
            
            obs1 = p1(4,:) < obj.range;
            obs2 = p2(4,:) < obj.range;
            
            p1 = p1(1:3,:);
            p2 = p2(1:3,:);  
        
            if obj.transform
                pnts  = [ homtrans(obj.trans1, p1), homtrans(obj.trans2, p2) ];
                numdet = [obs1, obs2];
            else
                pnts = [p1, p2];
                numdet = [obs1, obs2];
            end
        end

        function [t1,t2] = calculateTransform(obj,ref)
            
            obj.h1pos = obj.sensor1.position(ref.id); 
            obj.h2pos = obj.sensor2.position(ref.id);
            obj.h1eul = obj.sensor1.orientation(ref.id);
            obj.h2eul = obj.sensor2.orientation(ref.id);
            
            t1 = transl(obj.h1pos) * trotx(obj.h1eul(1)) * troty(obj.h1eul(2)) * trotz(obj.h1eul(3));
            t2 = transl(obj.h2pos) * trotx(obj.h2eul(1)) * troty(obj.h2eul(2)) * trotz(obj.h2eul(3));
            
        end
    
    end
    
end

