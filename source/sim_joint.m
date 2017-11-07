%% sim_joint %%
%
%
%
% A class to handle joint objects in the simulation
% environment. Inherits from entity class. Not intended for spherical
% joints, use sim_spherical_joint instead.
%
% Properties
%       
% Methods:
%
%   get_angle                % Retrieves joint's current intrinsic position 
%                              in radians. 
%                               
%   force                    % Retrieves magnitude of force acting on joint in
%                              Newton meters.
%   mode                     % Returns [m,c], where m = joint motor enabled
%                              or disabled, and c = joint motor control
%                              enabled or disabled. 
%
%   set_angle                % Sets joint's intrinsic position in radians.
%   set_tgt_vel              % Sets joint's target velocity. Only 
%                              applicable if the joint motor is enabled.  
%   set_tgt_pos              % Sets joint's target position in radians. 
%                              Only applicable if the joint motor and 
%                              position control are enabled.
%
%   enable_mcontrol          % Enables joint motor control
%   disable_mcontrol         % Disables joint motor control
%

classdef sim_joint < sim_entity
    
    properties     
    end
    
    methods
        
        function obj = sim_joint(sim,ident)
    
            obj = obj@sim_entity(sim,ident);
            
        end
        
        function pos = get_angle(obj)
        % sim_joint.get_angle
        %
        % Returns the current intrinsic angle of the joint. 
        %
        % Returns:
        %
        %   pos         % The angle of the joint in radians.
        %

            pos = obj.sim.getJointPosition(obj.id);

        end
               
         
        function frc = force(obj)
        % sim_joint.force
        % 
        % Returns the magnitude of force currently being applied to the
        % joint.
        %
        % Returns:
        % 
        %   frc         % The force applied around the axis of the joint in
        %                 Newton meters.   
        %
            
            frc = obj.sim.getJointForce(obj.id);
            
        end
        
        function res = mode(obj)
        % sim_joint.mode
        %
        % Returns [m,c].
        % m = motor enabled (1 = yes, 0 = no)
        % c = motor control (1 = yes, 0 = no)
        %
        
           m = obj.get_IntParam(2000);
           c = obj.get_IntParam(2001);
           
           res = [m,c];
           
        end
        
        %% Setters
        
        function set_angle(obj,new)
        % sim_joint.set_angle
        %
        % Sets the angle of the joint.
        %
        % Arguments:
        %
        %   new         % A new angle in radians.
        %
                obj.sim.setJointPosition(obj.id,new);
            
        end
        
        function set_force(obj,new)
        % sim_joint.set_force
        %
        % Sets the force applied around the axis of the joints.
        %
        % Arguments:
        %
        %   new         % A new force in Newton Meters
        %
        
            obj.sim.setJointForce(obj.id,new);
            
        end
        
        function set_tgt_vel(obj,vel)
        % sim_joint.set_tgt_vel
        %
        % Sets a target velocity for the joint motor. Joint motor and joint
        % motor control must be enabled.
        %
        % Arguments
        %
        %   vel         % A target velocity in radians per second.
        %
            
            obj.sim.setJointTargetVelocity(obj.id,vel);
            
        end
        
        function set_tgt_pos(obj,pos)
        % sim_joint.set_tgt_pos
        %
        % Sets a target angle for the joint to move to. The speed of
        % movemet is dependant on the maximum velocity and torque set in
        % the joint's dynamic properties dialog.
        %
        % Arguments:
        %
        %   pos         % A target position in radians.
        %
            
            obj.sim.setJointTargetPosition(obj.id,pos);
            
        end
        
        
        function enable_motor(obj)
        % sim_joint.enable_motor
        %
        % Enables the joint motor.
        %
            
            obj.set_IntParam(2000,1);
            
        end
        
        function enable_control(obj)
        % sim_joint.enable_control
        %
        % Enables joint motor control. Has no effect unless joint motor is
        % enabled.
        %
        
            obj.set_IntParam(2001,1); 
                                     
        end
        
        function disable_motor(obj)
        % sim_joint.disable_motor
        %
        % Disables the joint motor.
        %
        
            obj.set_IntParam(2000,0);
        
        end
        
        function disable_control(obj)
        % sim_joint.disable_control
        %
        % Disables joint motor control.
        %

            obj.set_IntParam(2001,0);
            
        end
               
        
        
    end
    
end

