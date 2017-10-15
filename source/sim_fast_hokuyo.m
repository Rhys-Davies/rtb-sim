%% Class to represent a VREP Fast Hokuyo sensor

classdef sim_fast_hokuyo < sim_entity
    
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
            obj.sensor1 = obj.sim.xy_sensor('fastHokuyo_sensor1');
            obj.sensor2 = obj.sim.xy_sensor('fastHokuyo_sensor2');  
            
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
        
            p1 = obj.sensor1.grab;
            p2 = obj.sensor2.grab;
            
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
        
        
%         plotData
%         % Read data from the Hokuyo
%            
%         s1 = yb.hokuyo.h1pos;
%         s2 = yb.hokuyo.h2pos;
%         
%       
%         subplot(211)
% %         plot(X(in), Y(in), '.g', pts(1, contacts), pts(2, contacts), '*r',...
% %              [s1(1), pts(1, :), s2(1)], [s1(2), pts(2, :), s2(2)], 'r',...
% %              0, 0, 'ob', s1(1), s1(2), 'or', s2(1), s2(2), 'or');
%          
%          plot(pts(1, contacts), pts(2, contacts), '*r',...
%              [s1(1), pts(1, :), s2(1)], [s1(2), pts(2, :), s2(2)], 'b',...
%              0, 0, 'og', s1(1), s1(2), 'ob', s2(1), s2(2), 'ob');
%          
%          
%         axis([-5.5, 5.5, -5.5, 2.5]);
%         axis equal;
%         drawnow;
%         
        
    
    end
    
end

