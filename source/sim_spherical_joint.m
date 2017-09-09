%% Spherical Joint object Class %%
%
% A class to specifically handle spherical joints. For all other joint
% types see sim_joint.
%
% Properties
%       joint_mode
% Methods:
%
%   intrinsic            % Returns the intrinsic matrix of the joint.
%   set_intrinsic        % Sets the intrinsic matrix of the joint.
%

classdef sim_spherical_joint < sim_entity
    
    properties
    end
    
    methods
        function obj = sim_spherical_joint(sim,ident)
                           
           obj = obj@sim_entity(sim,ident);
            
        end
        
        function pos = intrinsic(obj)
            
                pos = obj.sim.getJointMatrix(obj.id);

        end
        
        function set_intrisic(obj,new)

               obj.sim.setJointMatrix(obj.id,new);
               
        end
        
    end
    
end

