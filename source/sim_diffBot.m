classdef sim_diffBot < sim_entity
%% A rudimentary diff-drive robot for demoScene2
%
% Properties
%
%
% Methods
%
%
    
    properties
        
        wheels
        rgbdcamera
        
    end
    
    methods
        
        function obj = sim_diffBot(sim,ident)
    
            obj = obj@sim_entity(sim,ident);
            obj.rgbdcamera = sim.rgbdCamera('rgbdSensor');
            wheels(1) = sim.joint('diffBot_rightMotor');
            wheels(2) = sim.joint('diffBot_leftMotor');
            obj.wheels = wheels;
        end
        
        function setMotorVel(obj,new)
        
            obj.sim.pauseComms(true);
            obj.wheels(1).set_tgt_vel(new(1))
            obj.wheels(2).set_tgt_vel(new(2))
            obj.sim.pauseComms(false);
        end
        
        function im = getImage(obj)
        
            im = obj.rgbdcamera.image;
        
        end
        
        function enc = getEncoder(obj)
        
             a(1) = obj.sim.getFloatSignal('r_encoder');
             a(2) = obj.sim.getFloatSignal('l_encoder');
             enc = a;
             
        end 
        
    end
        
end

