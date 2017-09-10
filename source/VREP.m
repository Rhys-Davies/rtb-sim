%% VREP Class %%
%
% Inherits from the simulator superclass. Methods are overridden where 
% appropriate to deal with VREPs interface.
%


%% TODO!!
% Getters and setters need to allow a seperate opmode to be specified.
% Also need to know when a command is set to stream and use opmode_buffer
% instead of the currently set opmode.
% Wrap inputParser or switch to RTB input parser.
%%

% properties
%         clientID

% 
% These methods are abstract methods in the simulator class that must be
% defined
%         
%     % Generic object management
%         handle = getHandle(obj_name)
%         name = getName(obj_handle)
%         children = getChildren(obj)
%         pos = getPosition(obj)
%         pose = getPose(obj)
%         orint = getOrientation(obj)
%         status = setPosition(obj,new_pos)
%         status = setPose(obj,new_pose)
%         status = setOrientation(obj,new_orint)
%         
%      % Joint Specific Methods
%         pos = getJointPosition(joint)
%         vel = getJointVelocity(joint)
%         status = setJointPosition(joint,new_pos)
%         status = setJointVelocity(joint,new_vel)
%         
%      % Sensor Specific Methods
%         res = getSensorFrame(sensor)
%         
%      % Generic simulation management
%         status = pauseSim(delay)
%         status = startSim(delay)
%         status = stopSim(delay)
%         status = loadSimWorld(pathTOworld)
%         status = loadSimObject(pathTOobject)
%         status = deleteSimObject(obj)
%         time = pingSim(id)
%         
%      % Signal setters
%         res = setIntegerSignal(target)
%         res = setFloatSignal(target)
%         res = setBooleanSignal(target)
%         res = setStringSignal(target)
%         
%      % Signal getters
%         sig = getIntegerSignal(target)
%         sig = getFloatSignal(target)
%         sig = getBooleanSignal(target)
%         sig = getStringSignal(target)
%
%
% These are VREP specific methods:
%         res = setStatusMessage(message) - Not implemented Yet
%
%
%
%
%% Opmodes
%
%
% simx_opmode_oneshot
%
%   Non-blocking - Command send and a previous reply to the same
%   command returned. The function does not wait for the actual reply.
%
% simx_opmode_blocking
%
%   Blocking - The command is sent, and the function will wait for the
%   actual reply and return that as long as the command doesn't time
%   out. The recieved reply is removed from the inbox buffer.
%
% simx_opmode_streaming + alpha
%
%   Non-blocking - The command is sent and any previous reply to the 
%   same command is returned. This command will be continuously
%   executed on the server side and the function does not wait for the
%   actual reply. Alpha is a value between 100 and 65535 representing
%   the delay between function executions in ms.
%
% simx_opmode_oneshot_split + beta
%
%   Non-blocking - NOT RECOMMENDED - The command is sent in small
%   chunks and any previous reply for the same command is returned. The
%   command is executed server side and the reply will also be returned
%   in small chunks. The function does not wait for the actual reply.
%   Beta is a value between 100 and 65535 representing the maximum
%   chunk size in bytes to send. Small values won't slow down the
%   communication framework but it will take longer for the full
%   command to be transferred. Larger values will result in commands
%   being transferred faster but the communication framework might
%   appear frozen while chunks are being trasferred.
%
% simx_opmode_streaming_split + beta
%
%   Non-blocking - NOT RECOMMENDED - The command is sent in small
%   chunks and any previous reply to the same command is returned. The
%   command will be continuously executed on the server side, and
%   replys will be sent in small chunks. The function doesn't wait for
%   the actual reply. Beta is a value between 100 and 65535
%   representing the maximum chunk size in bytes to send. Small values
%   won't slow down the communication framework, but it will take more
%   time until the full commmand has been transferred. With large
%   values, command are transferred faster, but the communication
%   framework might appear frozen while the chunks are being
%   transferred.
%
% simx_opmode_discontinue
%   
%   Non-blocking - The command is sent and any previous reply to the
%   same command returned. A same command will be erased from the
%   server side if the command is of streaming or continuous type. The
%   same will happen on the client's input buffer. The function doesn't
%   wait for the actual reply.
%
% simx_opmode_buffer
%
%   Non-blocking - A previous reply to the same command is returned if
%   avaliable. The command is not sent nor does the function wait for
%   the actual reply. 
%
% simx_opmode_remove
%
%   Non-blocking - A previous reply to the same command is cleared from
%   the input buffer if present. The command is not sent and the
%   function doesn't return any values aside from the return code. Can
%   be used to free memory client side.
%

%% V-REP Parameters
% See http://www.coppeliarobotics.com/helpFiles/en/objectParameterIDs.htm
% for a complete list of parameters.

%% Valid Object Type Arguments for getObjects() (and their respective VREP object types)

%         Show a list of all objects in the scene. Optionally specify a
%         particular type of object to return only objects of that type.
%         
%         -----------------
%         Object Types:
%         -----------------
%         shape - sim_object_shape_type
%         joint - sim_object_joint_type
%         graph - sim_object_graph_type
%         camera - sim_object_camera_type
%         light - sim_object_light_type
%         dummy - sim_object_dummy_type
%         proximity_sensor - sim_object_proximitysensor_type
%         path - sim_object_path_type
%         vision_sensor - sim_object_visionsensor_type
%         mill - sim_object_mill_type
%         force_sensor - sim_object_forcesensor_type
%         mirror - sim_object_mirror_type
%         -----------------
%         Object Subtypes  
%         -----------------
%         omni_light - sim_light_omnidirectional_subtype
%         spot_light - sim_light_spot_subtype
%         directional_light - sim_light_directional_subtype
%         revolute_joint - sim_joint_revolute_subtype
%         prismatic_joint - sim_joint_prismatic_subtype
%         spherical_joint - sim_joint_spherical_subtype
%         simple_shape - sim_shape_simpleshape_subtype
%         multi_shape - sim_shape_multishape_subtype
%         ray_proximity_sensor - sim_proximitysensor_ray_subtype
%         pyramid_proximity_sensor - sim_proximitysensor_pyramid_subtype
%         cylinder_proximity_sensor - sim_proximitysensor_cylinder_subtype
%         disc_proximity_sensor - sim_proximitysensor_disc_subtype
%         cone_proximity_sensor - sim_proximitysensor_cone_subtype
%         pyramid_mill -  sim_mill_pyramid_subtype
%         cylinder_mill - sim_mill_cylinder_subtype
%         disc_mill - sim_mill_disc_subtype
%         cone_mill - sim_mill_cone_subtype   

%%        Vision Sensor Parameters
%         sim_visionfloatparam_near_clipping (1000): float parameter : near clipping plane
%         sim_visionfloatparam_far_clipping (1001): float parameter : far clipping plane
%         sim_visionintparam_resolution_x (1002): int32 parameter : resolution x
%         sim_visionintparam_resolution_y (1003): int32 parameter : resolution y
%         sim_visionfloatparam_perspective_angle (1004): float parameter : perspective projection angle
%         [r,msg] = obj.sim.simxGetFloatParameter(onj.clientID,target,obj.mode);
%         [r,msg] = obj.sim.simxGetIntegerParameter(obj.clientID,target,obj.mode);
        
classdef VREP < simulator
    
    properties
        mode
    end
    
    methods
        function obj = VREP(varargin)
            
            %vrep = remApi('remoteApi','extApi.h'); % This option requires a compiler
            obj.sim = remApi('remoteApi');
            p = inputParser;
            
            disp('VREP Constructor Called')
            %timeout    Timeout in ms default 2000
            %cycle      cycle time in ms default is 5
            %port       default 19997
            %reconnect  reconnect on error default is false
            %path       path to vrep remote API files
            
            % if path not specified then assume required files are in a
            % folder already on the MATLAB path.
            
            %% input parsing
            defaultIP = '127.0.0.1';
            defaultPORT = 19997;
            defaulttimeout = 2000;
            defaultcycle = 5;
            defaultwaitforconnect = true;
            defaultreconnect = true;
            defaultpath = 'path'; % Assumes API files are in a folder currently on your MATLAB path.
            defaultmode = obj.sim.simx_opmode_oneshot_wait;
            
            addOptional(p,'IP',defaultIP,@isstring);
            addOptional(p,'PORT',defaultPORT,@isnumeric);
            addOptional(p,'timeout',defaulttimeout,@isnumeric);
            addOptional(p,'mode',defaultmode,@isstring);
            addOptional(p,'cycle',defaultcycle,@isnumeric);
            addOptional(p,'wait',defaultwaitforconnect,@islogical); %wait for connect
            addOptional(p,'reconnect',defaultreconnect,@islogical); %reconnect on error
            addOptional(p,'path',defaultpath,@isstring);
            
            parse(p,varargin{:});
            
            
            obj.IP = p.Results.IP;
            obj.PORT = p.Results.PORT;
            %%
                   
            % TODO: Some path handling stuff
            
            %obj.sim.simxFinish(-1);
            obj.mode = obj.sim.simx_opmode_blocking; % Default to blocking
            obj.clientID = obj.sim.simxStart(obj.IP,obj.PORT,p.Results.wait, ...
                p.Results.reconnect,p.Results.timeout,p.Results.cycle);
            
        end
        
%       function display(obj)
%       
%                       
%       
%       end


        % Destroy the simulator object and cleanup
        function delete(obj)
            
            obj.sim.simxFinish(obj.clientID);
            obj.sim.delete();
            obj.clientID = -1;
            obj.sim = [];
            
        end
        
        %% Generic simulation management
 
         
        function pauseComms(obj,status)
            % Pauses the communication thread, preventing it from sending
            % data. Useful for sending multiple commands that are to be
            % recieved and evaluated simultaniously.
            % status = 1 pauses
            % status = 0 resumes
            
            r = obj.sim.simxPauseCommunication(obj.clientID,status);
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end
        
        function pauseSim(obj)  
            
            r = obj.sim.simxPauseSimulation(obj.clientID,obj.mode);
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end

        function startSim(obj) 
            r = obj.sim.simxStartSimulation(obj.clientID,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end
        
        function stopSim(obj)
            r = obj.sim.simxStopSimulation(obj.clientID,obj.mode);
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end
        
        function loadScene(obj,path,opt)
            % Loads a given VREP scene. Specify opt = 1 if scene is client
            % side, leave blank or specify opt = 0 if scene is server side.
            
            obj.stopSim();
            
            if nargin < 3
                opt = 0; % Add documentation for opt.
            end
            
            r = obj.sim.simxLoadScene(obj.clientID,path,opt,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end
        
        function id = loadSimObject(obj,handle,opt)
            % Loads an object/model into the currently open scene. Specify
            % opt = 1 if file is client side, leave blank or specify opt =
            % 0 if file is server side.
            
            if nargin < 3
                opt = 0; % 0 for server side model, 1 for client side.
            end
            
            [r,id] = obj.sim.simxLoadModel(obj.clientID,handle,opt,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end
        
        function deleteSimObject(obj,handle)
            
            r = obj.sim.simxRemoveModel(obj.clientID,handle,obj.mode);
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end
        
        function closeScene(obj)
            % Closes the currently open scene and then switches to the next
            % open scene. If no other scenes are open, a new scene will be
            % created.
            
            obj.stopSim();
            r = obj.sim.simxCloseScene(obj.clientID,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end
        
        function time = pingSim(obj,n)
            % Ping the VREP API n times.
            % Returns a matrix of all resulting ping times.
            
            temp = [];
            
            for i = 0:n
                [r,temp(n)] = obj.sim.simxGetPingTime(obj.clientID);
                if r ~= 0
                    throw(obj.except(r)); 
                end
            end
            
            time = temp;
            
        end
        
        
        %% Generic object management
        
        
        function [objid,name] = getObjects(obj,type)      
                
        % Show a list of all objects in the scene. Optionally specify a
        % particular type of object to return only objects of that type.
        
        % -----------------
        % Object Types:
        % -----------------
        % shape - sim_object_shape_type
        % joint - sim_object_joint_type
        % graph - sim_object_graph_type
        % camera - sim_object_camera_type
        % light - sim_object_light_type
        % dummy - sim_object_dummy_type
        % proximity_sensor - sim_object_proximitysensor_type
        % path - sim_object_path_type
        % vision_sensor - sim_object_visionsensor_type
        % mill - sim_object_mill_type
        % force_sensor - sim_object_forcesensor_type
        % mirror - sim_object_mirror_type
        % -----------------
        % Object Subtypes  
        % -----------------
        % omni_light - sim_light_omnidirectional_subtype
        % spot_light - sim_light_spot_subtype
        % directional_light - sim_light_directional_subtype
        % revolute_joint - sim_joint_revolute_subtype
        % prismatic_joint - sim_joint_prismatic_subtype
        % spherical_joint - sim_joint_spherical_subtype
        % simple_shape - sim_shape_simpleshape_subtype
        % multi_shape - sim_shape_multishape_subtype
        % ray_proximity_sensor - sim_proximitysensor_ray_subtype
        % pyramid_proximity_sensor - sim_proximitysensor_pyramid_subtype
        % cylinder_proximity_sensor - sim_proximitysensor_cylinder_subtype
        % disc_proximity_sensor - sim_proximitysensor_disc_subtype
        % cone_proximity_sensor - sim_proximitysensor_cone_subtype
        % pyramid_mill -  sim_mill_pyramid_subtype
        % cylinder_mill - sim_mill_cylinder_subtype
        % disc_mill - sim_mill_disc_subtype
        % cone_mill - sim_mill_cone_subtype   
            
            
            if nargin < 2
                stype = obj.sim.sim_appobj_object_type;
            else
                switch (type)
                    case 'shape'
                        stype = obj.sim.sim_object_shape_type;
                    case 'joint'
                        stype = obj.sim.sim_object_joint_type;
                    case 'graph'
                        stype = obj.sim.sim_object_graph_type;
                    case 'camera'
                        stype = obj.sim.sim_object_camera_type;
                    case 'light'
                        stype = obj.sim.sim_object_light_type;
                    case 'dummy'
                        stype = obj.sim.sim_object_dummy_type;
                    case 'proximity_sensor'
                        stype = obj.sim.sim_object_proximitysensor_type;
                    case 'path'
                        stype = obj.sim.sim_object_path_type;
                    case 'vision_sensor'
                        stype = obj.sim.sim_object_visionsensor_type;
                    case 'mill'
                        stype = obj.sim.sim_object_mill_type;
                    case 'force_sensor'
                        stype = obj.sim.sim_object_forcesensor_type;
                    case 'mirror'
                        stype = obj.sim.sim_object_mirror_type;
                    case 'omni_light'
                        stype = obj.sim.sim_light_omnidirectional_subtype;
                    case 'spot_light'
                        stype = obj.sim.sim_light_spot_subtype;
                    case 'directional_light'
                        stype = obj.sim.sim_light_directional_subtype;
                    case 'revolute_joint'
                        stype = obj.sim.sim_joint_revolute_subtype;
                    case 'prismatic_joint'
                        stype = obj.sim.sim_joint_prismatic_subtype;
                    case 'spherical_joint'
                        stype = obj.sim.sim_joint_spherical_subtype;
                    case 'simple_shape'
                        stype = obj.sim.sim_shape_simpleshape_subtype;
                    case 'multi_shape'
                        stype = obj.sim.sim_shape_multishape_subtype;
                    case 'ray_proximity_sensor'
                        stype = obj.sim.sim_proximitysensor_ray_subtype;
                    case 'pyramid_proximity_sensor'
                        stype = obj.sim.sim_proximitysensor_pyramid_subtype;
                    case 'cylinder_proximity_sensor'
                        stype = obj.sim.sim_proximitysensor_cylinder_subtype;
                    case 'disc_proximity_sensor'
                        stype = obj.sim.sim_proximitysensor_disc_subtype;
                    case 'cone_proximity_sensor'
                        stype = obj.sim.sim_proximitysensor_cone_subtype;
                    case 'pyramid_mill'
                        stype = obj.sim.sim_mill_pyramid_subtype;
                    case 'cylinder_mill'
                        stype = obj.sim.sim_mill_cylinder_subtype;
                    case 'disc_mill'
                        stype = obj.sim.sim_mill_disc_subtype;
                    case 'cone_mill'
                        stype = obj.sim.sim_mill_cone_subtype;
                    otherwise
                        disp('Unknown object type')
                        % TODO - Throw exception
                end
            end

            [r,objid,~,~,name] = obj.sim.simxGetObjectGroupData(obj.clientID, stype, 0, obj.mode);

            if r ~= 0
                throw(obj.except(r)); 
            end      

        end        
        
        function [handle] = getHandle(obj, in, varargin)
            
            if nargin < 3
                name = in;
            else
                name = sprintf(in, varargin{:});
            end
            
            [r,handle] = obj.sim.simxGetObjectHandle(obj.clientID,name,obj.mode);
        
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end      
        
        function name = getName(obj,objhandle)
            
            [r,objid,~,~,str] = obj.sim.simxGetObjectGroupData(obj.clientID, obj.sim.sim_appobj_object_type, 0, obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
            
            name = str(objid == objhandle,1);
            
        end
        
        function types = getTypes(obj,objhandle)
        
            [r,objid,types,~,~] = obj.sim.simxGetObjectGroupData(obj.clientID, obj.sim.sim_appobj_object_type, 1, obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
            
            types = types(objid == objhandle);
        
        end
        
        function children = getChildren(obj,handle)
            
            i = 0;
            while true
                
                [r,child] = obj.sim.simxGetObjectChild(obj.clientID, handle, i, obj.mode);
            	
                if child < 0
                    break
                end
                
                if r ~= 0
                    throw(obj.except(r));
                end
                
                i = i + 1;
                children(i) = child;
                
            end
            
        end
        
        %% TODO Add opmode_streaming support.
        function orient = getOrientation(obj,handle,rel2,varargin)
                        
            if nargin < 3
                rel2 = -1;
            end
            
            [r,orient] = obj.sim.simxGetObjectOrientation(obj.clientID,handle,rel2,obj.mode); 
            
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end
            
        %% TODO Add opmode_streaming support.
        function pos = getPosition(obj,handle,rel2,varargin)
            
            if nargin < 3
                rel2 = -1;
            end
        
            [r,pos] = obj.sim.simxGetObjectPosition(obj.clientID,handle,rel2,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end
        %%
        
        function setPosition(obj,handle,new,rel2)
            % rel2 = -1 for global frame
            % else rel2 is the clientID of an object.
            
            if nargin < 4
                rel2 = -1;
            end
            
            r = obj.sim.simxSetObjectPosition(obj.clientID,handle,rel2,new,obj.mode);
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end
        
        function setOrientation(obj,handle,orient,rel2)
            % rel2 = -1 for global frame
            % else
            
            if nargin < 4
                rel2 = -1;
            end
                
            r = obj.sim.simxSetObjectOrientation(obj.clientID,handle,rel2,orient,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end
        
%%      Joint Specific Methods
        %% TODO Add opmode_streaming support.
        function pos = getJointPosition(obj,handle,varargin)
        % Returns the intrinsic position of a joint. Cannot be used with
        % spherical joints
        
                
                [r,pos] = obj.sim.simxGetJointPosition(obj.clientID,handle,obj.mode);

                if r ~= 0
                    throw(obj.except(r)); 
                end
                

        
        end
        
        function matrix = getJointMatrix(obj,handle,varargin)
        % Returns the intrisic matrix of a joint.
        
            [r,matrix] = obj.sim.simxGetJointMatrix(obj.clientID,handle,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
        
        end
        
        %% TODO Add opmode_streaming support.
        function force = getJointForce(obj,handle,varargin)
        % For prismatic and revolute joints only
        % When using the bullet physics engine, returns force or torque
        % applied to the joint motor.
        % When using the ODE or Vortex engines, returns the total force or
        % torque applied to a joint around/along its z-axis.
        
            [r,force] = obj.sim.simxGetJointForce(obj.clientID,handle,obj.mode);
            
            if r ~= 0
               throw(obj.except(r)); 
            end
        
        end
          
        function setSphericalJointMatrix(obj,handle,matrix)
        % TODO: Check to make sure matrix contains 12 elements
        % Can only be used for spherical joints.
        
            r = obj.sim.simxSetSphericalJointMatrix(obj.clientID,handle,matrix,obj.mode);
                      
            if r ~= 0
                throw(obj.except(r)); 
            end
        
        end
        
        function setJointForce(obj,handle,force)
        % Has no effect if joint is not dynamically enabled.
        % Also has no effect if joint is spherical
        
            r = obj.sim.simxSetJointForce(obj.clientID,handle,force,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
        
        end
        
        function setJointPosition(obj,handle,pos)
        % Cannot be used on spherical joints
        % May have no effect with certain joint modes
        
            r = obj.sim.simxSetJointPosition(obj.clientID,handle,pos,obj.sim.simx_opmode_oneshot);
           
%             if r ~= 0
%                 throw(obj.except(r)); 
%             end
%              
        end
            
        function setJointTargetPosition(obj,handle,pos)
        % Sets joint target position. Only works if joint is in
        % torque/force mode and if its motor and position control are
        % enabled
        
            r = obj.sim.simxSetJointTargetPosition(obj.clientID,handle,pos,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
        
        end
        
        function setJointTargetVelocity(obj,handle,vel)
        % Sets a non-spherical joint's target velocity. Joint needs to be
        % in torque/force mode with dynamics and joint motor enabled, and
        % position control disabled.
        
            r = obj.sim.simxSetJointTargetVelocity(obj.clientID,handle,vel,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
        
        end
            
%% Signal management

 
        function sig = getIntegerSignal(obj,signalName)
            
            [r,sig] = obj.sim.simxGetIntegerSignal(obj.clientID,signalName,obj.mode);
           
            if r ~= 0
                throw(obj.except(r)); 
            end
           
        end

        function sig = getFloatSignal(obj,signalName)
            
            [r,sig] = obj.sim.simxGetFloatingSignal(obj.clientID,signalName,obj.mode);
           
            if r ~= 0
                throw(obj.except(r)); 
            end
           
        end

        function sig = getBooleanSignal(obj,signalName)
            
            [r,sig] = obj.sim.simxGetBooleanSignal(obj.clientID,signalName,obj.mode);
           
            if r ~= 0
                throw(obj.except(r)); 
            end
           
        end

        function sig = getStringSignal(obj,signalName)
            
            [r,sig] = obj.sim.simxGetStringSignal(obj.clientID,signalName,obj.mode);
        
            if r ~= 0
                throw(obj.except(r)); 
            end
        
        end

        function setIntegerSignal(obj,signalName,value)
            
            r = obj.sim.simxSetIntegerSignal(obj.clientID,signalName,value,obj.sim.simx_opmode_oneshot_wait);
        
            if r ~= 0
                throw(obj.except(r)); 
            end
        
        end

        function setFloatSignal(obj,signalName,value)
            
            r = obj.sim.simxSetFloatSignal(obj.clientID,signalName,value,obj.mode);
           
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end

        function setBooleanSignal(obj,signalName,value)
            
            r = obj.sim.simxSetBooleanSignal(obj.clientID,signalName,value,obj.mode);
           
            if r ~= 0
                throw(obj.except(r)); 
            end
           
        end

        function setStringSignal(obj,signalName,value)
            
            r = obj.sim.simxSetStringSignal(obj.clientID,signalName,value,obj.mode);
           
            if r ~= 0
                throw(obj.except(r)); 
            end
           
        end
        
        function clearIntegerSignal(obj,signalName)
            
            r = obj.sim.simxClearIntegerSignal(obj.clientID,signalName,obj.mode);
           
            if r ~= 0
                throw(obj.except(r)); 
            end
           
        end

        function clearFloatSignal(obj,signalName)
               
            r = obj.sim.simxClearFloatSignal(obj.clientID,signalName,obj.mode);
        
            if r ~= 0
               throw(obj.except(r)); 
            end
        
        end

        function clearBooleanSignal(obj,signalName)
            
            r = obj.sim.simxClearBooleanSignal(obj.clientID,signalName,obj.mode);
        
            if r ~= 0
                throw(obj.except(r)); 
            end
        
        end

        function clearStringSignal(obj,signalName)
            
            r = obj.sim.simxClearStringSignal(obj.clientID,signalName,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end
        
        function setSignal(obj,target,msg,dtype)
            
            switch (dtype)
                case 'str'
                    r = obj.sim.simxSetStringSignal(obj.clientID,target,msg,obj.mode);
                case 'float'
                    r = obj.sim.simxSetFloatSignal(obj.clientID,target,msg,obj.mode);
                case 'int'
                    r = obj.sim.simxSetIntegerSignal(obj.clientID,target,msg,obj.mode);
                case 'bool'
                    r = obj.sim.simxSetBooleanSignal(obj.clientID,target,msg,obj.mode);
                otherwise 
                    throw(MException('RTB-SIM:Method:setSignal','Unknown Datatype'))
            end
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end
       
        function sig = getSignal(obj,target,dtype)
            
            switch (dtype)
                case 'str'
                    [r,sig] = obj.sim.simxGetStringSignal(obj.clientID,target,obj.mode);
                case 'float'
                    [r,sig] = obj.sim.simxGetFloatSignal(obj.clientID,target,obj.mode);
                case 'int'
                    [r,sig] = obj.sim.simxGetIntegerSignal(obj.clientID,target,obj.mode);
                case 'bool'
                    [r,sig] = obj.sim.simxGetBooleanSignal(obj.clientID,target,obj.mode);
                otherwise 
                    throw(MException('RTB-SIM:Method:getSignal','Unknown Datatype'))
            end
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end
        
        function clearSignal(obj,target,dtype)
        
            switch (dtype)
                case 'str'
                    r = obj.sim.simxClearStringSignal(obj.clientID,target,obj.mode);
                case 'float'
                    r = obj.sim.simxClearFloatSignal(obj.clientID,target,obj.mode);
                case 'int'
                    r = obj.sim.simxClearIntegerSignal(obj.clientID,target,obj.mode);
                case 'bool'
                    r = obj.sim.simxClearBooleanSignal(obj.clientID,target,obj.mode);
                otherwise 
                    throw(MException('RTB-SIM:Method:clearSignal','Unknown Datatype'))
            end
            
            if r ~= 0
               throw(obj.except(r)); 
            end
        
        end
        
%% Parameter Management

        function param = getIntegerParam(obj,target)
            
            [r,param] = obj.sim.simxGetIntegerParameter(obj.clientID,target,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end

        function param = getFloatParam(obj,target)
            
            [r,param] = obj.sim.simxGetFloatingParameter(obj.clientID,target,obj.mode);
          
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end

        function param = getBooleanParam(obj,target)
            
            [r,param] = obj.sim.simxGetBooleanParameter(obj.clientID,target,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end

        function param = getStringParam(obj,target)
            
            [r,param] = obj.sim.simxGetStringParameter(obj.clientID,target,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end

        function setIntegerParam(obj,target, param)
            
            r = obj.sim.simxGetIntegerParameter(obj.clientID,target,param,obj.mode);
            
            if r ~= 0
                throw(obj.except(r)); 
            end
            
        end

        function setFloatParam(obj, target, param)
            
            r = obj.sim.simxGetFloatParameter(obj.clientID,target,param,obj.mode);
          
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end

        function setBooleanParam(obj, target, param)
            
            r = obj.sim.simxGetBooleanParameter(obj.clientID,target,param,obj.mode);
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end

        function setStringParam(obj, target, param)
            
            r = obj.sim.simxGetStringParameter(obj.clientID,target,param,obj.mode);
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end
        
        function setObjIntParam(obj,target,param,new)
            
            r = obj.sim.simxSetObjectIntParameter(obj.clientID, target, param, new, obj.mode);
            
            if r ~= 0
               throw(obj.except(r)); 
            end      
            
        end
        
        function setObjFloatParam(obj,target,param,new)
            
            r = obj.sim.simxSetObjectFloatParameter(obj.clientID,target,param, new, obj.mode);
            
            if r ~= 0
               throw(obj.except(r)); 
            end
                    
        end
        
        function param = getObjIntParam(obj,target,param)
            
            [r, param] = obj.sim.simxGetObjectIntParameter(obj.clientID,target,param,obj.mode);
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end
        
        function param = getObjFloatParam(obj,target,param)
            
            [r, param] = obj.sim.simxGetObjectFloatParameter(obj.clientID,target,param,obj.mode);
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
            
        end  
        
        
        function setParam(obj,target,msg,dtype)
        
            switch (dtype)
                
                case 'str'
                    r = obj.sim.simxSetStringParameter(obj.clientID,target,msg,obj.mode);
                case 'float'
                    r = obj.sim.simxSetFloatParameter(obj.clientID,target,msg,obj.mode);
                case 'int'
                    r = obj.sim.simxSetIntegerParameter(obj.clientID,target,msg,obj.mode);
                case 'bool'
                    r = obj.sim.simxSetBooleanParameter(obj.clientID,target,msg,obj.mode);
                otherwise 
                    throw(MException('RTB-SIM:Method:setParam','Unknown Datatype'))
            end
            
            if r ~= 0
               throw(obj.except(r)); 
            end
        
        end
        
        function msg = getParam(obj,target,dtype)
            
            switch (dtype)
                case 'str'
                    [r,msg] = obj.sim.simxGetStringParameter(obj.clientID,target,obj.mode);
                case 'float'
                    [r,msg] = obj.sim.simxGetFloatParameter(onj.clientID,target,obj.mode);
                case 'int'
                    [r,msg] = obj.sim.simxGetIntegerParameter(obj.clientID,target,obj.mode);
                case 'bool'
                    [r,msg] = obj.sim.simxGetBooleanParamter(obj.clientID,target,obj.mode);
                otherwise 
                    throw(MException('RTB-SIM:Method:getParam','Unknown Datatype'))
            end
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end
        
%% Image Sensor Handling

        % Again, condense into one function that takes differnet modes and
        % datatypes as arguments.
%           TODO: Check if these are global or object specific. 
%         sim_visionfloatparam_near_clipping (1000): float parameter : near clipping plane
%         sim_visionfloatparam_far_clipping (1001): float parameter : far clipping plane
%         sim_visionintparam_resolution_x (1002): int32 parameter : resolution x
%         sim_visionintparam_resolution_y (1003): int32 parameter : resolution y
%         sim_visionfloatparam_perspective_angle (1004): float parameter : perspective projection angle
%         [r,msg] = obj.sim.simxGetFloatParameter(onj.clientID,target,obj.mode);
%         [r,msg] = obj.sim.simxGetIntegerParameter(obj.clientID,target,obj.mode);
        
        
        %% TODO Add opmode_streaming support.
        function img = readVisionSensor(obj,target,varargin)

            grey = false; % TODO
            
            [r,~,img] = obj.sim.simxGetVisionSensorImage2(obj.clientID,target,grey,obj.sim.simx_opmode_oneshot_wait);
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end
        
        %% TODO Add opmode_streaming support.
        function pts = readPointVisionSensor(obj,target,varargin)
                 
            %obj.setIntegerSignal('handle_xyz_sensor', 1);
            %obj.setIntegerSignal('handle_xy_sensor', 1);
            [r, ~, auxData, dataIndex] = obj.sim.simxReadVisionSensor(obj.clientID, target, obj.mode);
        
            if r ~= 0
               throw(obj.except(r)); 
            end
            
            
            % First 15 data entries are min max and average of: Intensity,
            % red, green, blue, and depth. This is the first packet as defined
            % by dataIndex(1). Entries 16 and 17 are image height and width.
            
            w = auxData(dataIndex(1)+1); % Image width
            h = auxData(dataIndex(1)+2); % Image height
            
            %dataIndex(1) + 2 + 1 the start of the sensor output data.
            
            
            % 4 rows, w*h columns
            % Each colum is x,y,z,dist
            %
            pts = reshape(auxData((dataIndex(1)+2+1):end), 4, w*h);
        
        end
        
        
        %% TODO Add opmode_streaming support.
        function [res,img] = readVisionSensorDepth(obj,target,varargin)
            
            [r,res,img] = obj.sim.simxGetVisionSensorDepthBuffer2(obj.clientID,target,obj.sim.simx_opmode_oneshot_wait);
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end
   
        

%% Other sensors
        %% TODO Add opmode_streaming support.
        function [state,torque,force] = readForceSensor(obj,ident,varargin)
        
            [r,state,torque,force] = obj.sim.simxReadForceSensor(obj.clientID,ident,obj.mode);
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end
        
        %% TODO Add opmode_streaming support.
        function [state,point] = readProximitySensor(obj,ident,varargin)
            
            [r,state,point,~,~] = obj.sim.simxReadProximitySensor(obj.clientID,ident,obj.mode); % [r,state,point,found_obj,found_surface] 
            
            if r ~= 0
               throw(obj.except(r)); 
            end
            
        end
        
        
%% Simulation Objects

        function out = entity(obj,ident,varargin)
            
            out = sim_entity(obj,ident);

        end
        
        function out = joint(obj,ident,varargin)
            
            out = sim_joint(obj,ident,varargin);
            
        end
                
        function out = rgb_sensor(obj,ident,varargin)
            
            out = sim_rgb_sensor(obj,ident,varargin);
            
        end
        
        function out = xyz_sensor(obj,ident,varargin)
            
            out = sim_xyz_sensor(obj,ident,varargin);
            
        end
        
        function out = xy_sensor(obj,ident,varargin)
            

            out = sim_xy_sensor(obj,ident,varargin);
        
        end
        
        function out = rgbdCamera(obj,ident,varargin)
            
            out = sim_cameraRGBD(obj,ident,varargin);
        
        end
        
        function out = forceSensor(obj,ident,breakable,varargin)
            
            error('Not Implemented')
            
        end
        
        function out = hokuyo(obj,ident,ref,varargin)

            
             
            if nargin < 3
                out = sim_fast_hokuyo(obj,ident);
            else
                out = sim_fast_hokuyo(obj,ident,ref);
            end
            
        end
        
        
        function out = arm(obj,ident,fmt,varargin)

            if nargin < 3
                [list,num] = obj.armhelper(ident);
            else
                [list,num] = obj.armhelper(ident,fmt);
            end
            
             out = sim_arm(obj, list, num);
            
            
        end
        
        function out = youbot(obj,ident)

            out = sim_TRS_youBot(obj,ident);
            
        end
        
        %% Misc Helpers

        
        
        function [out,num] = armhelper(obj, name, fmt)
            % Returns a list of valid VREP ID's for an arm
            % Name of joint 0
            % fmt = naming format of the arm joints.
            
            h = obj.getHandle(name);
            if h == 0
                error('no such object as %s in the scene', name);
            end
            
            if nargin < 3
                fmt = '%s_joint%d';
            end    
            
            id = [];
            j = 1;
            while true
                
                try
                h = obj.getHandle(fmt, name, j-1);
                catch ME
                   if size(id) == 0
                       rethrow(ME)
                   else
                       break
                   end    
                end
                id(j) = h;
                j = j+1;
            end
            
            num = j - 1;
            out = id;
            
        end
        
        
    end 
    
    %% Private stuff. 
    
    methods(Access=private)
    
        % Convert VREP error codes into human readable form
        function e = except(obj,s)
            
            switch (s)
                case obj.sim.simx_return_ok                
                    return
                case obj.sim.simx_return_novalue_flag % Bit 0
                    msgid = 'VREP:NoReply';
                    error = 'Input buffer does not contain a command reply.';
                case obj.sim.simx_return_timeout_flag % Bit 1
                    msgid = 'VREP:Timeout';
                    error = 'Function timed out. Network connection is down or slow.';
                case obj.sim.simx_return_illegal_opmode_flag % Bit 2
                    msgid = 'VREP:IllegalOperationMode';
                    error = 'Function does not support the use of the selected operation mode.';
                case obj.sim.simx_return_remote_error_flag % Bit 3
                    msgid = 'VREP:ServerSideError';
                    error = 'Server-side function error. Check function handle is valid.';
                case obj.sim.simx_return_split_progress_flag % Bit 4
                    msgid = 'VREP:Busy';
                    error = 'Previous split command is still being processed. Try an opmode that does not split commands if this is an issue.';
                case obj.sim.simx_return_local_error_flag % Bit 5
                    msgid = 'VREP:ClientSideError';
                    error = 'Client-side function error.';
                case obj.sim.simx_return_initialize_error_flag % Bit 6
                    msgid = 'VREP:NotStarted';
                    error = 'Please call simxStart first. This error may also occur when you have a Remote API instance already active .';
                otherwise
                    msgid = 'VREP:UnknownError';
                    error = 'I''m sorry Dave, I''m afraid I can''t do that. (Unknown Error)';       
            end
           % Error
           
           e = MException(msgid,error);
           
        end   
 
    end
   
end

