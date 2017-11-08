%% sim_spherical_joint %%
%
% A class to handle spherical joints. For all other joint
% types see its parent class, sim_joint. Methods inherited from ‘sim_joint’ % that have no effect on spherical joints have been overloaded to simply 
% throw an error message.
%
% Properties
%       
% Methods:
%
%   get_angle           % Returns 3 vector containing euler angles 
%                         representing the position of the joint in radians.
%   set_angle           % Sets the position of the joint to the euler angles 
%                         given as a 3 vector.
%


classdef sim_spherical_joint < sim_joint
    
    properties
    end
    
    methods
        function obj = sim_spherical_joint(sim,ident)
                           
           obj = obj@sim_joint(sim,ident);
            
        end
        
        function pos = get_angle(obj)
            
                pos = obj.sim.getJointMatrix(obj.id);

        end
        
        function set_angle(obj,new)

               obj.sim.setJointMatrix(obj.id,new);
               
        end
        
        function force(obj)
            
            error('sim_spherical_joint: Operation not valid for spherical joints');
       
        end
        
        function mode(obj)
            
           error('sim_spherical_joint: Operation not valid for spherical joints');
           
        end
        
         function set_angl(obj,new)
            
                obj.sim.setJointPosition(obj.id,new);
            
        end
        
        function set_force(obj,new)
            
            error('sim_spherical_joint: Operation not valid for spherical joints');
            
        end
        
        function set_tgt_vel(obj,vel)
            
            error('sim_spherical_joint: Operation not valid for spherical joints');
            
        end
        
        function set_tgt_pos(obj,pos)
            
            error('sim_spherical_joint: Operation not valid for spherical joints');
            
        end
        
        
        function enable_motor(obj)
            
            error('sim_spherical_joint: Operation not valid for spherical joints');
            
        end
        
        function enable_control(obj)
        
            error('sim_spherical_joint: Operation not valid for spherical joints');
                                     
        end
        
        function disable_motor(obj)
        
           error('sim_spherical_joint: Operation not valid for spherical joints');
        
        end
        
        function disable_control(obj)

           error('sim_spherical_joint: Operation not valid for spherical joints');
            
        end

        
    end
    
end

