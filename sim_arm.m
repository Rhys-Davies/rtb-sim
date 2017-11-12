%% sim_arm
%
% A class to represent an assembly that consists of linked joints 
% such as a robot arm. Can also be used for legs on a humanoid robot.
%
% Argument base should be the object that anchors the arm, eg. Its mounting point on
% the youBot. 
%
% An arm joint in V-REP is usually named as: robotNamejointName# where # is
% the n-th joint.
%
% 'handle' is the name of the assembly (eg. 'youBot').
%
% 'fmt' is the format of the joint name. This will be '%sjointName%d 
% (eg. %sArmJoint%d for the youBot). 
%
% Properties
%   
%   numj                % Number of joints in the arm
%   joints              % Array of sim_joint objects that comprise the arm
%
% Methods
%
%   state               % An array of the all joint angles of all arm joints
%   set_state           % Set the joint angle of all joints in the arm
%                         using an m-vector, where m = numj
%
%   enable_control      % Enables motor control for all arm joints.
%   disable_control     % Disables motor control for all arm joints.
%


classdef sim_arm < sim_entity
    
    properties
        
        numj %Number of joints in assembly.
        joints %Array of joint objects.

    end
    
    methods
  
        function obj = sim_arm(sim, base, handle, fmt)

            obj = obj@sim_entity(sim,base);
            
            if nargin < 4
                [list,num] = obj.sim.armhelper(handle);
            else
                [list,num] = obj.sim.armhelper(handle,fmt);
            end
            
            obj.numj = num;
            
            for i=1:obj.numj
               joints(i) = obj.sim.joint(list(i));
            end
            
            obj.joints = joints;
            
        end
        
        function s = state(obj)
        % sim_arm.state
        %
        % Returns the intrinsic position of each joint in the arm. This
        % will be represented by an n-vector, where n = number of joints.
        % The order will be [joint 0, joint 1, ... joint n].
        %
        % Returns:
        %
        %   s           % Angle of all arms joints in radians.
        %
        %
        
        
            
            a = [];
                        
            for i=1:obj.numj
               a(i) = obj.joints(i).get_angle(state(i));
            end
            
            s = a;
            
        end
        
        function set_state(obj,state)
        % sim_arm.set_state
        %
        % Sets the intrinsic position of each joint in the arm. You must
        % have joint motors enabled and joint motor control enabled for all
        % joints in the arm. See sim_arm.enable_control.  
        %
        % Arguments:
        %
        %   state           % A vector of new joint target positions for the
        %                     arm. Angles need to be in radians.
        %
        
            
            if size(state,2) ~= obj.numj
                error('sim_arm: Number of new angles does not match number of joints');
            end
                       
            for i=1:obj.numj
               obj.joints(i).set_tgt_pos(state(i));
            end
            
        end
        
        
        function enable_control(obj)
        % sim_arm.enable_control
        %
        % Configures all joints in the arm to enable joint motors and joint
        % motor control.
        %
        % This is required if you intend to use sim_arm.set_state.
        %
            
            for i=1:obj.numj
               obj.joints(i).enable_motor; 
               obj.joints(i).enable_control;
            end
            
        end
        
        function disable_control(obj)
        % sim_arm.disable_control
        %
        % Configures all joints in the arm to disable joint motors and joint
        % motor control.
        %
            
            for i=1:obj.numj
               obj.joints(i).disable_motor
               obj.joints(i).disable_control;
            end
            
        end
        
        
    end
    
end

