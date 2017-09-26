%% Joint object Class %%
%
% A class to handle joint objects in the simulation
% environment. Inherits from entity class. Not intended for spherical
% joints, use sim_spherical_joint instead.
%
% Properties
%       
% Methods:
%
% jpos                     % Retrieves joint's current intrinsic position.
% force                    % Retrieves magnitue of force acting on joint.
%
% set_jpos                 % Sets joint's intrinsic position.
% set_tgt_vel              % Sets joint's target velocity. Only 
%                             applicable if the joint motor is enabled.  
% set_tgt_pos              % Sets joint's target position. Only applicable
%                             if the joint motor and position control are
%                             enabled.
%
% enable_mcontrol          % Enables joint motor control
% disable_mcontrol         % Disables joint motor control
%

classdef sim_joint < sim_entity
    
    properties     
    end
    
    methods
        
        function obj = sim_joint(sim,ident)
    
            obj = obj@sim_entity(sim,ident);
            
        end
        
        function pos = jpos(obj)
            
            pos = obj.sim.getJointPosition(obj.id);

        end
               
         
        function frc = force(obj)
            % Returns the magnitude of force currently being applied to the
            % joint.
            
            frc = obj.sim.getJointForce(obj.id);
            
       
        end
        
        %% Setters
        
        function set_jpos(obj,new)
            
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

