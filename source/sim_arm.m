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


classdef sim_arm < handle % < sim_entity
    
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
        %% sim_arm.state    
        % Returns the intrinsic position of each joint in the arm
        
            
            a = [];
                        
            for i=1:obj.numj
               a(i) = obj.joints(i).jpos(state(i));
            end
            
            s = a;
            
        end
        
        function set_state(obj,state)
        %% sim_arm.set_state
        % Sets the intrinsic position of each joint in the arm
        %
        % Arguments:
        %   state           % A vector of new joint target positions for the
        %                     arm. Angles need to be in radians.
        
            
            if size(state,2) ~= obj.numj
                error('Number of new angles does not match number of joints');
            end
                       
            for i=1:obj.numj
               obj.joints(i).set_tgt_pos(state(i));
            end
            
        end
        
        
        function enable_control(obj)
        %% sim_arm.enable_control
        % Configures all joints in the arm to enable joint motors and joint
        % motor control.
            
            for i=1:obj.numj
               obj.joints(i).enable_motor; 
               obj.joints(i).enable_control;
            end
            
        end
        
        function disable_control(obj)
        %% sim_arm.enable_control
        % Configures all joints in the arm to disable joint motors and joint
        % motor control.
            
            for i=1:obj.numj
               obj.joints(i).disable_motor
               obj.joints(i).disable_control;
            end
            
        end
        
        
    end
    
end

