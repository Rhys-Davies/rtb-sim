classdef Gazebo < handle %Gazebo < simulator

    %% MATLAB Examples demonstrate that at least the following functionality is possible.
    %   ExampleHelperGazeboCommunicator methods:
    %       pauseSim            - Calls service to pause simulation
    %       resumeSim           - Calls service to resume simulation after a pause
    %       resetSim            - Calls service to reset simulation
    %       resetWorld          - Calls service to reset the world
    %       readPhysics         - Returns struct of simulation physics properties
    %       setPhysics          - Sets simulation physics properties
    %       getSpawnedModels    - Returns a list of all spawned models
    %       spawnModel          - Creates a new model in the simulation
    %       removeModel         - Removes a specified model from the simulation
    %
    %   ExampleHelperGazeboCommunicator properties:
    %       Physics                     - Physics parameters of the simulation engine
    %       ModelsList                  - List of models in Gazebo world
    %       IsModelServicesRunning      - Flag to check if model services are running
    %       BodyForceClient             - Service Client for applying force
    %       JointTorqueClient           - Service Client for applying joint torque
    %       SetModStateClient           - Service Client for setting model state
    %       GetModPropClient            - Service Client for fetching model properties
    %       GetModStateClient           - Service Client for fetching model state
    %       SetModConfigClient          - Service Client for setting joint configurations

%% Avaliable Services
% /gazebo/apply_body_wrench
% /gazebo/apply_joint_effort
% /gazebo/clear_body_wrenches
% /gazebo/clear_joint_forces
% /gazebo/delete_light
% /gazebo/delete_model
% /gazebo/get_joint_properties
% /gazebo/get_light_properties
% /gazebo/get_link_properties
% /gazebo/get_link_state
% /gazebo/get_loggers
% /gazebo/get_model_properties
% /gazebo/get_model_state
% /gazebo/get_physics_properties
% /gazebo/get_world_properties
% /gazebo/pause_physics
% /gazebo/reset_simulation
% /gazebo/reset_world
% /gazebo/set_joint_properties
% /gazebo/set_light_properties
% /gazebo/set_link_properties
% /gazebo/set_link_state
% /gazebo/set_logger_level
% /gazebo/set_model_configuration
% /gazebo/set_model_state
% /gazebo/set_parameters
% /gazebo/set_physics_properties
% /gazebo/spawn_sdf_model
% /gazebo/spawn_urdf_model
% /gazebo/unpause_physics
% /rosout/get_loggers
% /rosout/set_logger_level


%% ROS Topics Avaliable
% /clock                        
% /gazebo/link_states           
% /gazebo/model_states          
% /gazebo/parameter_descriptions
% /gazebo/parameter_updates     
% /gazebo/set_link_state        
% /gazebo/set_model_state       
% /rosout                       
% /rosout_agg 




    properties     
        IP
        PORT
    end
    
    methods
        
        function obj = Gazebo(varargin)
        % if path not specified then assume required files are in a
            % folder already on the MATLAB path.
            
            %% input parsing
            
            p = inputParser;
            
            defaultIP = '127.0.0.1';
            defaultPORT = 11311;
            
            addOptional(p,'IP',defaultIP,@isstring);
            addOptional(p,'PORT',defaultPORT,@isnumber);
            
            parse(p,varargin{:});
            
            obj.IP = p.Results.IP;
            obj.PORT = p.Results.PORT;
            
            rosinit(obj.IP,obj.PORT);
            
        
        end
        
        %% Generic Simulation Management
        

        function startSim(obj)
        end
        
        
        function stopSim(obj)
        end
        
        function pauseSim(obj)  
        end

        
        function loadScene(obj,path,opt)
        end
        
        function id = loadSimObject(obj,handle,opt)
        end
        
        function deleteSimObject(obj,handle)
        end
        
        function closeScene(obj)
        end
        
        
        %% Generic object management
        
        function [objid,name] = getObjects(obj,type)
        end
        
        function [handle] = getHandle(obj,name)
        end
        
        function name = getName(obj,objhandle)
        end
        
        function children = getChildren(obj,handle)
        end
        
        function orient = getOrientation(obj,handle,rel2)
        end
        
        function pos = getPosition(obj,handle,rel2)
        end
        
        function setPosition(obj,handle,new,rel2)
        end
        
        function setOrientation(obj,handle,orient,rel2)
        end
        
        
        %% Joint Specific Methods
        function pos = getJointPosition(obj,handle)
        end
        
        
        function force = getJointForce(obj,handle)
        end
        
        function setSphericalJointMatrix(obj,handle,matrix)
        end
        
        function setJointForce(obj,handle,force)
        end
        
        function setJointPosition(obj,handle,pos)
        end
        
        function setJointTargetPosition(obj,handle,pos)
        end
        
        function setJointTargetVelocity(obj,handle,vel)
        end
        
      
        
        
        
    end
    
end

