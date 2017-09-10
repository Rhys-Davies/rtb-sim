%% sim_arm
% A class to represent an assembly that consists of linked joints 
% such as a robot arm. Can also be used for legs on a humanoid robot.
% ident in this case should be the string name or simulator ID
% of the hightest level joint (eg. Joint0)  'youBotArmJoint%d'
%
%
% Possible %TODO: Make inherit from sim_entity where the entity in question
% is the reference point of the arm. eg. you would pass the base of the arm
% model in the case of an arm robot, or the point where the arm mounts to 
% the robot in the case of combination mobile+arm robots (such as 
% youBot_ref in the case of the youBot.
% 
% Properties
%   
%   numj                % Number of joints in the arm
%   joints              % Array of sim_joint objects that comprise the arm
%   sim                 % Sim object. Not needed if above TODO implemented
%
% Methods
%
%   state               % An array of the intrinsic values of all joints in
%                         the arm
%   set_state           % Set the intrinsic values of all joints in the arm
%                         using an m-vector, where m = numj
%
%   enable_control      % Enables motor control for all arm joints.
%   disable_control     % Disables motor control for all arm joints.
%


classdef sim_arm < handle
    
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
               joints(i) = obj.sim.joint(list(i));
            end
            
            obj.joints = joints;
            
        end
        
        function s = state(obj)
            
            a = [];
                        
            for i=1:obj.numj
               a(i) = obj.joints(i).euler(state(i));
            end
            
            s = a;
            
        end
        
        function set_state(obj,state)
            
            if size(state,2) ~= obj.numj
                error('Number of new angles does not match number of joints');
            end
            
            %% If pausing communication to set all arms at once in VREP, 
            % vrep.simx_opmode_oneshot must be the opmode used.
            
            % obj.sim.pauseComms(true)
            
            for i=1:obj.numj
               state(i)
               obj.joints(i).set_tgt_pos(state(i));
            end
            
            % obj.sim.pauseComms(false)
            
        end
        
        % TODO: Figure out what default is.
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

