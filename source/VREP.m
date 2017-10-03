%% VREP Class %%
%
% Inherits from the simulator superclass. Methods are overridden where 
% appropriate to deal with VREPs interface.
%


%% TODO!!
% Force Sensor
% Move any VREP specific signal calls or data processing into the methods in this file.  
%

%% Notes:
%   Return codes can be returned as any combination of bits... error
%   catching method needs a rewrite.
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
%         pauseSim()
%         startSim()
%         stopSim()
%         loadSimWorld(pathTOworld)
%         loadObject(pathTOobject)
%         deleteObject(obj)
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
        path
        libpath
        mode
        getter_mode
        setter_mode
        blocking_mode
    end
    
    methods
        function obj = VREP(varargin)
            
            %vrep = remApi('remoteApi','extApi.h'); % This option requires a compiler
            obj.sim = remApi('remoteApi');
            
            obj.sim.simxFinish(-1);
            
            p = inputParser;

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
            defaultpath = getenv('VREP');
            defaultgettermode = obj.sim.simx_opmode_oneshot_wait;
            defaultsettermode = obj.sim.simx_opmode_oneshot;
            
            addOptional(p,'IP',defaultIP,@isstring);
            addOptional(p,'PORT',defaultPORT,@isnumeric);
            addOptional(p,'timeout',defaulttimeout,@isnumeric);
            addOptional(p,'getter_mode',defaultgettermode,@isstring);
            addOptional(p,'setter_mode',defaultsettermode,@isstring);
            addOptional(p,'cycle',defaultcycle,@isnumeric);
            addOptional(p,'wait',defaultwaitforconnect,@islogical); %wait for connect
            addOptional(p,'reconnect',defaultreconnect,@islogical); %reconnect on error
            addOptional(p,'path',defaultpath,@isstring);
            
            parse(p,varargin{:});
            
            obj.getter_mode = p.Results.getter_mode;
            obj.setter_mode = p.Results.setter_mode;
            obj.IP = p.Results.IP;
            obj.PORT = p.Results.PORT;
            path = p.Results.path;
            %%
            
            obj.blocking_mode = obj.sim.simx_opmode_blocking;
            
            
            %% Check if all the VREP API files are already on path
            if ispc == true
                file1 = exist('remoteApi.dll');
            elseif isunix == true
                file1 = exist('remoteApi.so');
            elseif ismac == true
                file1 = exist('remoteApi.dylib');
            else
                error('RTB-SIM:VREP:', 'Operating system not identified');
            end
                        
            file2 = exist('remApi.m');
            file3 = exist('remoteApiProto.m');
            
  
            %% Prioritize specified path over any files found on path, 
            % but if no install folder found, fall back to files already found 
            % on MATLAB path (if any).
            
            if isempty(path)
                
                 if file1 ~= 2 || file2 ~= 2 || file3 ~= 2
            
                    error('RTB-SIM:VREP:API Files not Found','No or invalid VREP path given');
                    
                 end
                 
                 obj.path = 'None';
                 
            else
                
                if ~exist(path,'dir')
                    
                    error('RTB-SIM:VREP:Invalid Path','VREP Folder %s not found', path);
                    
                end

                obj.path = path;

                libpath = { fullfile(obj.path, 'programming', 'remoteApi')
                    fullfile(obj.path, 'programming', 'remoteApiBindings', 'matlab', 'matlab')
                    fullfile(obj.path, 'programming', 'remoteApiBindings', 'lib', 'lib')
                    };

                obj.libpath = libpath;

                addpath( libpath{:} );
                
            end
            
            %% Initialize the simulator.
            
            obj.clientID = obj.sim.simxStart(obj.IP,obj.PORT,p.Results.wait, ...
                p.Results.reconnect,p.Results.timeout,p.Results.cycle);
            
            if obj.clientID < 0
                
                error('RTB-SIM:VREP:Connection Failed','Connection to VREP failed!');
                
            end
                  
        end
        
        function display(obj)
            %VREP.display Display parameters
            %
            % V.display() displays the VREP parameters in compact format.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is a VREP object and the command has no trailing
            %   semicolon.
            %
            % See also VREP.char.
            
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(obj) );
        end 
        
        function s = char(obj)
            %VREP.char Convert to string
            %
            % V.char() is a string representation the VREP parameters in human
            % readable foramt.
            %
            % See also VREP.display.

            s = sprintf('V-REP robotic simulator interface (active=%d)', obj.checkcomms() );

            s = strvcat(s, sprintf('path: %s ', obj.path));
            
            switch obj.getter_mode
                case obj.sim.simx_opmode_oneshot
                    s = strvcat(s, 'Getter mode: simx_opmode_oneshot (non-blocking)');
                case obj.sim.simx_opmode_oneshot_wait
                    s = strvcat(s, 'Getter mode: simx_opmode_oneshot_wait (blocking)');
                case obj.sim.simx_opmode_streaming
                    s = strvcat(s, 'Getter mode: simx_opmode_streaming (non-blocking)');
                case obj.sim.simx_opmode_buffer
                    s = strvcat(s, 'Getter mode: simx_opmode_buffer (non-blocking)');
            end
            
            switch obj.setter_mode
                case obj.sim.simx_opmode_oneshot
                    s = strvcat(s, 'Setter mode: simx_opmode_oneshot (non-blocking)');
                case obj.sim.simx_opmode_oneshot_wait
                    s = strvcat(s, 'Setter mode: simx_opmode_oneshot_wait (blocking)');
                case obj.sim.simx_opmode_streaming
                    s = strvcat(s, 'Setter mode: simx_opmode_streaming (non-blocking)');
                case obj.sim.simx_opmode_buffer
                    s = strvcat(s, 'Setter mode: simx_opmode_buffer (non-blocking)');
            end
            
            switch obj.blocking_mode
                case obj.sim.simx_opmode_oneshot
                    s = strvcat(s, 'Blocking mode: simx_opmode_oneshot (non-blocking)');
                case obj.sim.simx_opmode_oneshot_wait
                    s = strvcat(s, 'Blocking mode: simx_opmode_oneshot_wait (blocking)');
                case obj.sim.simx_opmode_streaming
                    s = strvcat(s, 'Blocking mode: simx_opmode_streaming (non-blocking)');
                case obj.sim.simx_opmode_buffer
                    s = strvcat(s, 'Blocking mode: simx_opmode_buffer (non-blocking)');
            end 

            
        end

        % Destroy the simulator object and cleanup
        function delete(obj)
            

            obj.sim.simxFinish(obj.clientID);
            obj.sim.simxFinish(-1);
            obj.sim.delete();
           
        end
        
        %% Generic simulation management
 
         
        function pauseComms(obj,status)
        %% VREP.pauseComms
        % Pauses the communication thread, preventing it from sending
        % and receiving data. Useful for sending multiple commands that
        % are to be recieved and evaluated simultaniously.
        % 
        % Arguments
        %
        %   status            % 1 or true to pause, 0 or false to resume
        %

            r = obj.sim.simxPauseCommunication(obj.clientID,status);
            
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        function pauseSim(obj)  
        %% VREP.pauseSim
        % Pauses the simulation.
        %
            
            r = obj.sim.simxPauseSimulation(obj.clientID,obj.setter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
  
        end

        function startSim(obj) 
        %% VREP.startSim
        % Starts the simulation. This must be run before any other commands
        % are called.
        %
            
            r = obj.sim.simxStartSimulation(obj.clientID,obj.setter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end

        end
        
        function stopSim(obj)
        %% VREP.stopSim
        % Stops the simulation.
            
            r = obj.sim.simxStopSimulation(obj.clientID,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end

            
        end
        
        function r = checkcomms(obj)
            
            r = obj.sim.simxGetConnectionId(obj.clientID);
            
        end
        
        
        function loadScene(obj,scene,varargin)
        %% VREP.loadScene
        % Loads a specified scene.
        %
        %
        %
            
            p = inputParser;
            defaultFN = obj.path;
            defaultLoc = false;
            
            addOptional(p,'fn',defaultFN,@isstring);
            addOptional(p,'local',defaultLoc,@islogical);
            
            parse(p,varargin{:});
            
            a = p.Results.fn;
            b = p.Results.local;
            
            if ~strcmp(a,'None') && ~b
                
                error('RTB-SIM:VREP:','A full path to scene must be given if using API files on path.');
                
            else    
                
               if ~b
                   
                   if scene(1) ~= '/'
                       
                       scene = fullfile(obj.path, 'models', [scene '.ttm']);
                       
                   end
                   
               end
                   
            end
            
            obj.stopSim();
            
            r = obj.sim.simxLoadScene(obj.clientID,scene,b,obj.blocking_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        function id = loadObject(obj,model,varargin)
        %% VREP.loadSimObject
        % Loads an object/model into the currently open scene. Specify
        % opt = 1 if file is client side, leave blank or specify opt =
        % 0 if file is server side.
            
            p = inputParser;
            defaultFN = obj.path;
            defaultLoc = false;
            defaultClient = true;
            
            addOptional(p,'fn',defaultFN,@isstring);
            addOptional(p,'local',defaultLoc,@islogical);
            addOptional(p,'client',defaultClient,@islogical);
            
            parse(p,varargin{:});
            
            a = p.Results.fn;
            b = p.Results.local;
            c = p.Results.client;
            
            
            
            
%             if ~strcmp(a,'None') && ~b
%                 
%                 error('RTB-SIM:VREP:','A full path to object must be given if using API files on path.');
%                 
%             else    
%                 
%                if ~b
%                    
%                    if model(1) ~= '/'
%                        
%                        model = fullfile(obj.path, 'models', [model '.ttm']);                    
%                        
%                    end
%                end
%                    
%            end
            
            [r,id] = obj.sim.simxLoadModel(obj.clientID,model,b,obj.blocking_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end    
            
            pause(0.5)
            
        end
        
        function deleteObject(obj,handle)
        %% VREP.deleteSimObject
        % Deletes an object with a given handle from currently active V-REP
        % scene.
        %
            
            r = obj.sim.simxRemoveModel(obj.clientID,handle,obj.setter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
           end
            
        end
        
        function closeScene(obj)
            % Closes the currently open scene and then switches to the next
            % open scene. If no other scenes are open, a new scene will be
            % created.
            
            obj.stopSim();
            r = obj.sim.simxCloseScene(obj.clientID,obj.setter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        function time = pingSim(obj,n)
            % Ping the VREP API n times.
            % Returns a matrix of all resulting ping times.
            
            temp = [];
            
            for i = 0:n
                [r,temp(n)] = obj.sim.simxGetPingTime(obj.clientID);
                
               if r ~= 0 && r ~= 1
                    throw(obj.errcheck(r))
                end
                
            end
            
            time = temp;
            
        end
        
        
        %% Generic object management
        
        
        function [objid,name] = getObjects(obj,type)      
        %% VREP.getObjects
        % Show a list of all objects in the scene. Optionally specify a
        % particular type of object to return only objects of that type.
        % 
        % Arguments:
        %
        %   type            % Optionally specifies a type of object to
        %                     return.
        %
        % The following object types are valid:
        %
        % -----------------
        % Major Object Types:
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

            [r,objid,~,~,name] = obj.sim.simxGetObjectGroupData(obj.clientID, stype, 0, obj.blocking_mode);

           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end

        end        
        
        function [handle] = getHandle(obj, in, varargin)
        %% VREP.getHandle
        % Retrieves the V-REP identifier of an object given its string
        % name.
        %
        % Arguments:
        %
        %   in          % The string name
        %   
        % Optional Arguments
        %
        %   
        %
            
            if nargin < 3
                name = in;
            else
                name = sprintf(in, varargin{:});
            end
            
            [r,handle] = obj.sim.simxGetObjectHandle(obj.clientID,name,obj.blocking_mode);
        
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end      
        
        function name = getName(obj,objhandle)
        %% VREP.getName
        %
        %
        %
        %
        %
        %
            
            
            [r,objid,~,~,str] = obj.sim.simxGetObjectGroupData(obj.clientID, obj.sim.sim_appobj_object_type, 0, obj.blocking_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
            name = str(objid == objhandle,1);
            
        end
        
        function types = getTypes(obj,objhandle)
        %% VREP.getTypes
        %
        %
        %
        %
        %
        
            [r,objid,types,~,~] = obj.sim.simxGetObjectGroupData(obj.clientID, obj.sim.sim_appobj_object_type, 1, obj.blocking_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
            types = types(objid == objhandle);
        
        end
        
        function children = getChildren(obj,handle)
        %% VREP.getChildren
        %
        %
        %
        %
        %
        %
            
            i = 0;
            while true
                
                [r,child] = obj.sim.simxGetObjectChild(obj.clientID, handle, i, obj.blocking_mode);
            	
                if child < 0
                    break
                end

                if r ~= 0 && r ~= 1
                    throw(obj.errcheck(r))
                end
                
                i = i + 1;
                children(i) = child;
                
            end
            
        end
        
        %% TODO Add opmode_streaming support.
        function orient = getOrientation(obj,handle,rel2,varargin)
        %% VREP.getOrientation
        %
        %
        %
        %
                        
            if nargin < 3
                rel2 = -1;
            end
            
            opmode = obj.getter_mode;
            
            [r,orient] = obj.sim.simxGetObjectOrientation(obj.clientID,handle,rel2,opmode); 
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
            
        % TODO Add opmode_streaming support.
        function pos = getPosition(obj,handle,rel2)
        %% VREP.getPosition
        %
        %
        %
        %
        %
            
            if nargin < 3
                rel2 = -1;
            end
        
            opmode = obj.getter_mode;
            
            [r,pos] = obj.sim.simxGetObjectPosition(obj.clientID,handle,rel2,opmode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        
        function setPosition(obj,handle,new,rel2)
            % rel2 = -1 for global frame
            % else rel2 is the clientID of an object.
            
            if nargin < 4
                rel2 = -1;
            end
            
            r = obj.sim.simxSetObjectPosition(obj.clientID,handle,rel2,new,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        

        
        function setOrientation(obj,handle,orient,rel2)
            % rel2 = -1 for global frame
            % else
            
            if nargin < 4
                rel2 = -1;
            end
                
            r = obj.sim.simxSetObjectOrientation(obj.clientID,handle,rel2,orient,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
%%      Joint Specific Methods

        function pos = getJointPosition(obj,handle,varargin)
        % Returns the intrinsic position of a joint. Cannot be used with
        % spherical joints
        
            opmode = obj.getter_mode;
        
            [r,pos] = obj.sim.simxGetJointPosition(obj.clientID,handle,opmode);

            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end


        
        end
        
        function matrix = getJointMatrix(obj,handle,varargin)
        % Returns the intrisic matrix of a joint.
        
            opmode = obj.getter_mode;
        
            [r,matrix] = obj.sim.simxGetJointMatrix(obj.clientID,handle,opmode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end
        

        function force = getJointForce(obj,handle,varargin)
        % For prismatic and revolute joints only
        % When using the bullet physics engine, returns force or torque
        % applied to the joint motor.
        % When using the ODE or Vortex engines, returns the total force or
        % torque applied to a joint around/along its z-axis.
        
            opmode = obj.getter_mode;
        
            [r,force] = obj.sim.simxGetJointForce(obj.clientID,handle,opmode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        end
          
        function setSphericalJointMatrix(obj,handle,matrix)
        % TODO: Check to make sure matrix contains 12 elements
        % Can only be used for spherical joints.
        
            r = obj.sim.simxSetSphericalJointMatrix(obj.clientID,handle,matrix,obj.setter_mode);
                      
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end
        
        function setJointForce(obj,handle,force)
        % Has no effect if joint is not dynamically enabled.
        % Also has no effect if joint is spherical
        
            r = obj.sim.simxSetJointForce(obj.clientID,handle,force,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end
        
        function setJointPosition(obj,handle,pos)
        % Cannot be used on spherical joints
        % May have no effect with certain joint modes
        
            r = obj.sim.simxSetJointPosition(obj.clientID,handle,pos,obj.setter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
            
        function setJointTargetPosition(obj,handle,pos)
        % Sets joint target position. Only works if joint is in
        % torque/force mode and if its motor and position control are
        % enabled
        
            r = obj.sim.simxSetJointTargetPosition(obj.clientID,handle,pos,obj.setter_mode);

            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        function setJointTargetVelocity(obj,handle,vel)
        % Sets a non-spherical joint's target velocity. Joint needs to be
        % in torque/force mode with dynamics and joint motor enabled, and
        % position control disabled.
        
            r = obj.sim.simxSetJointTargetVelocity(obj.clientID,handle,vel,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end
            
%% Signal management

 
        function sig = getIntegerSignal(obj,signalName)
            
            [r,sig] = obj.sim.simxGetIntegerSignal(obj.clientID,signalName,obj.getter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function sig = getFloatSignal(obj,signalName)
            
            [r,sig] = obj.sim.simxGetFloatingSignal(obj.clientID,signalName,obj.getter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
           
        end

        function sig = getBooleanSignal(obj,signalName)
            
            [r,sig] = obj.sim.simxGetBooleanSignal(obj.clientID,signalName,obj.getter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
           
        end

        function sig = getStringSignal(obj,signalName)
            
            [r,sig] = obj.sim.simxGetStringSignal(obj.clientID,signalName,obj.getter_mode);
        
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end

        function setIntegerSignal(obj,signalName,value)
            
            r = obj.sim.simxSetIntegerSignal(obj.clientID,signalName,value,obj.getter_mode);
        
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function setFloatSignal(obj,signalName,value)
            
            r = obj.sim.simxSetFloatSignal(obj.clientID,signalName,value,obj.getter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function setBooleanSignal(obj,signalName,value)
            
            r = obj.sim.simxSetBooleanSignal(obj.clientID,signalName,value,obj.getter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
           
        end

        function setStringSignal(obj,signalName,value)
            
            r = obj.sim.simxSetStringSignal(obj.clientID,signalName,value,obj.getter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
           
        end
        
        function clearIntegerSignal(obj,signalName)
            
            r = obj.sim.simxClearIntegerSignal(obj.clientID,signalName,obj.setter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function clearFloatSignal(obj,signalName)
               
            r = obj.sim.simxClearFloatSignal(obj.clientID,signalName,obj.setter_mode);
        
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end

        function clearBooleanSignal(obj,signalName)
            
            r = obj.sim.simxClearBooleanSignal(obj.clientID,signalName,obj.setter_mode);
        
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end

        function clearStringSignal(obj,signalName)
            
            r = obj.sim.simxClearStringSignal(obj.clientID,signalName,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        
%% Parameter Management

        function param = getIntegerParam(obj,target)
            
            [r,param] = obj.sim.simxGetIntegerParameter(obj.clientID,target,obj.getter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function param = getFloatParam(obj,target)
            
            [r,param] = obj.sim.simxGetFloatingParameter(obj.clientID,target,obj.getter_mode);
          
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function param = getBooleanParam(obj,target)
            
            [r,param] = obj.sim.simxGetBooleanParameter(obj.clientID,target,obj.getter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function param = getStringParam(obj,target)
            
            [r,param] = obj.sim.simxGetStringParameter(obj.clientID,target,obj.getter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function setIntegerParam(obj,target, param)
            
            r = obj.sim.simxGetIntegerParameter(obj.clientID,target,param,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function setFloatParam(obj, target, param)
            
            r = obj.sim.simxGetFloatParameter(obj.clientID,target,param,obj.setter_mode);
          
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function setBooleanParam(obj, target, param)
            
            r = obj.sim.simxGetBooleanParameter(obj.clientID,target,param,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function setStringParam(obj, target, param)
            
            r = obj.sim.simxGetStringParameter(obj.clientID,target,param,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        
%% Object Parameters

        function setObjIntParam(obj,target,param,new)
            
            r = obj.sim.simxSetObjectIntParameter(obj.clientID, target, param, new, obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        function setObjFloatParam(obj,target,param,new)
            
            r = obj.sim.simxSetObjectFloatParameter(obj.clientID,target,param, new, obj.setter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
                    
        end
        
        function param = getObjIntParam(obj,target,param)
            
            [r, param] = obj.sim.simxGetObjectIntParameter(obj.clientID,target,param,obj.getter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        function param = getObjFloatParam(obj,target,param)
            
            [r, param] = obj.sim.simxGetObjectFloatParameter(obj.clientID,target,param,obj.getter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
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
        
        

        function img = readVisionSensor(obj,target,varargin)

            grey = false; % TODO: Optional Argument
            opmode = obj.getter_mode;
            
            [r,~,img] = obj.sim.simxGetVisionSensorImage2(obj.clientID,target,grey,opmode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        

        function pts = readPointVisionSensor(obj,target,varargin)
                 
            opmode = obj.getter_mode;
            
            [r, ~, auxData, dataIndex] = obj.sim.simxReadVisionSensor(obj.clientID, target, opmode);
        
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
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
        

        function [res,img] = readVisionSensorDepth(obj,target,varargin)
            
            opmode = obj.getter_mode;
            
            [r,res,img] = obj.sim.simxGetVisionSensorDepthBuffer2(obj.clientID,target,opmode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
   
        

%% Other sensors

        function [state,torque,force] = readForceSensor(obj,ident,varargin)
            
            opmode = obj.getter_mode;
        
            [r,state,torque,force] = obj.sim.simxReadForceSensor(obj.clientID,ident,opmode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        

        function [state,point] = readProximitySensor(obj,ident,varargin)
            
            opmode = obj.getter_mode;
            
            [r,state,point,~,~] = obj.sim.simxReadProximitySensor(obj.clientID,ident,opmode); % [r,state,point,found_obj,found_surface] 
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        
%% Simulation Objects

        function out = entity(obj,ident)
            
            out = sim_entity(obj,ident);

        end
        
        function out = joint(obj,ident)
            
            out = sim_joint(obj,ident);
            
        end
                
        function out = rgb_sensor(obj,ident)
            
            out = sim_rgb_sensor(obj,ident);
            
        end
        
        function out = xyz_sensor(obj,ident)
            
            out = sim_xyz_sensor(obj,ident);
            
        end
        
        function out = xy_sensor(obj,ident)
            

            out = sim_xy_sensor(obj,ident);
        
        end
        
        function out = rgbdCamera(obj,ident)
            
            out = sim_cameraRGBD(obj,ident);
        
        end
        
        function out = forceSensor(obj,ident,breakable)
            
            error('Not Implemented')
            
        end
        
        function out = hokuyo(obj,ident,ref)

            
             
            if nargin < 3
                out = sim_fast_hokuyo(obj,ident);
            else
                out = sim_fast_hokuyo(obj,ident,ref);
            end
            
        end
        
        
        function out = arm(obj,ident,fmt)

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
                
                %% This is nasty... TODO! Replace this.
                try
                h = obj.getHandle(fmt, name, j-1);
                % if h == 0
                %   break
                % end
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
        % TODO: Handle errors dependant on the opmode used.
        % TODO: Parse the bit coded return message to enable reporting
        % multiple error. err = de2bi(r,6)
        
        function e = errcheck(obj,r)
            
                switch (r)               
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
                        error = sprintf('I''m sorry Dave, I''m afraid I can''t do that. (Unknown Error Code: %d)',r);
                end
                    e = MException(msgid,error); 
        end
            
    end 
    
%     function e = errcheck(obj,r,b)
%
%           if nargin < 3
%               b = false;
%           end
%
%            a = de2bi(r,6)
%             
%                 switch (r)
%                         case obj.sim.simx_error_noerror
%                             return
%                         case obj.sim.simx_return_novalue_flag % Bit 0
%                             if b == true
%                                 return
%                             else
%                                 msgid = 'VREP:NoReply';
%                                 error = 'Input buffer does not contain a command reply.';
%                         case obj.sim.simx_return_timeout_flag % Bit 1
%                             msgid = 'VREP:Timeout';
%                             error = 'Function timed out. Network connection is down or slow.';
%                         case obj.sim.simx_return_illegal_opmode_flag % Bit 2
%                             msgid = 'VREP:IllegalOperationMode';
%                             error = 'Function does not support the use of the selected operation mode.';
%                         case obj.sim.simx_return_remote_error_flag % Bit 3
%                             msgid = 'VREP:ServerSideError';
%                             error = 'Server-side function error. Check function handle is valid.';
%                         case obj.sim.simx_return_split_progress_flag % Bit 4
%                             msgid = 'VREP:Busy';
%                             error = 'Previous split command is still being processed. Try an opmode that does not split commands if this is an issue.';
%                         case obj.sim.simx_return_local_error_flag % Bit 5
%                             msgid = 'VREP:ClientSideError';
%                             error = 'Client-side function error.';
%                         case obj.sim.simx_return_initialize_error_flag % Bit 6
%                             msgid = 'VREP:NotStarted';
%                             error = 'Please call simxStart first. This error may also occur when you have a Remote API instance already active .';
%                     otherwise
%                         msgid = 'VREP:UnknownError';
%                         error = sprintf('I''m sorry Dave, I''m afraid I can''t do that. (Unknown Error Code: %d)',r);  
%                 end
%                      
%         end
%             
%     end 
    
    
 
end
   


