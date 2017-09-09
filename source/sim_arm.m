classdef sim_arm < handle

    % A class to represent an assembly that consists of linked joints 
    % such as a robot arm. Can also be used for legs on a humanoid robot.
    % ident in this case should be the string name or simulator ID
    % of the hightest level joint (eg. Joint0)  'youBotArmJoint%d'
    
    properties
        
        numj %Number of joints in assembly.
        joints %Array of joint objects.
        sim
        
    end
    
    methods
        function obj = sim_arm(sim, list, num)
            
            obj.sim = sim;
            obj.numj = num;
            
            for i=1:obj.numj
               joints(i) = obj.sim.joint(list(i),true);
            end
            
            obj.joints = joints;
            
        end
        
        function move(obj,state)
            
            if size(state,2) ~= obj.numj
                error('Number of new angles does not match number of joints');
            end
            
            for i=1:obj.numj
               state(i)
               obj.joints(i).set_euler(state(i));
            end
            
        end
        
        function enable_control(obj)
            
            for i=1:obj.numj
               obj.joints(i).enable_motor; 
               obj.joints(i).enable_control;
            end
            
        end
        
        function disable_control(obj)
            
            for i=1:obj.numj
               obj.joints(i).disable_motor
               obj.joints(i).disable_control;
            end
            
        end
        
        
    end
    
end

