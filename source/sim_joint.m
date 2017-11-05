%% sim_joint %%
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
            
            pos = obj.sim.getJointPosition(obj.id);

        end
               
         
        function frc = force(obj)
            % Returns the magnitude of force currently being applied to the
            % joint.
            
            frc = obj.sim.getJointForce(obj.id);
            
       
        end
        
        function res = mode(obj)
        % Returns [m,c].
        % m = motor enabled (1 = yes, 0 = no)
        % c = motor control (1 = yes, 0 = no)
        
           m = obj.get_IntParam(2000);
           c = obj.get_IntParam(2001);
           
           res = [m,c];
           
        end
        
        %% Setters
        
        function set_angle(obj,new)
            
                obj.sim.setJointPosition(obj.id,new);
            
        end
        
        function set_force(obj,new)
            
            obj.sim.setJointForce(obj.id,new);
            
        end
        
        function set_tgt_vel(obj,vel)
            
            obj.sim.setJointTargetVelocity(obj.id,vel);
            
        end
        
        function set_tgt_pos(obj,pos)
            
            obj.sim.setJointTargetPosition(obj.id,pos);
            
        end
        
        
        function enable_motor(obj)
            
            obj.set_IntParam(2000,1);
            
        end
        
        function enable_control(obj)
        
            obj.set_IntParam(2001,1); 
                                     
        end
        
        function disable_motor(obj)
        
            obj.set_IntParam(2000,0);
        
        end
        
        function disable_control(obj)

            obj.set_IntParam(2001,0);
            
        end
               
        
        
    end
    
end

