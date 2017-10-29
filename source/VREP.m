%% VREP Simulator Class
% NOTE: All VREP API Functions use SI Units!
%
% Properties
%
%         clientID
%         sim
%         PORT
%         IP
%         clientID
%         path
%         libpath
%         getter_mode
%         setter_mode
%         blocking_mode

% 
% Methods:
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

        
classdef VREP < handle %simulator no need to inherit from this
    
    properties
        sim
        PORT
        IP
        clientID
        path
        libpath
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
            
            obj.blocking_mode = obj.sim.simx_opmode_blocking;
            obj.getter_mode = p.Results.getter_mode;
            obj.setter_mode = p.Results.setter_mode;
            
            obj.IP = p.Results.IP;
            obj.PORT = p.Results.PORT;
            path = p.Results.path;
            
            % Check if all the VREP API files are already on path
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
            
  
            % Prioritize specified path over any files found on path, 
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
            
            % Initialize the simulator.
            
            obj.clientID = obj.sim.simxStart(obj.IP,obj.PORT,p.Results.wait, ...
                p.Results.reconnect,p.Results.timeout,p.Results.cycle);
            
            if obj.clientID < 0
                
                error('RTB-SIM:VREP:Connection Failed','Connection to VREP failed!');
                
            end
                  
        end
        
        function display(obj)
            % VREP.display Display parameters
            %
            % V.display() displays the VREP parameters in compact format.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is a VREP object and the command has no trailing
            %   semicolon.
            %

            
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(obj) );
        end 
        
        function s = char(obj)
            % VREP.char Convert to string
            %
            % V.char() is a string representation the VREP parameters in human
            % readable foramt.
            %


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

        
        function delete(obj)
        % VREP.delete
        %
        % Destroy the simulator object and cleanup. 
        %

            obj.sim.simxFinish(obj.clientID);
            obj.sim.simxFinish(-1);
            obj.sim.delete();
           
        end
        
        %% Generic simulation management
 
         
        function pauseComms(obj,status)
        % VREP.pauseComms
        %
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
        % VREP.pauseSim
        %
        % Pauses the current simulation.
        %
            
            r = obj.sim.simxPauseSimulation(obj.clientID,obj.setter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
  
        end

        function startSim(obj) 
        % VREP.startSim
        %
        % Starts the simulation. This must be run before any other commands
        % are called.
        %
            
            r = obj.sim.simxStartSimulation(obj.clientID,obj.setter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end

        end
        
        function stopSim(obj)
        % VREP.stopSim
        %
        % Stops the simulation. V-REP will automatically reset the scene to
        % the state it was in before VREP.startSim was called.
        %
            
            r = obj.sim.simxStopSimulation(obj.clientID,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end

            
        end
        
        function r = checkcomms(obj)
        % VREP.checkcomms
        %
        % Retruns the VREP connection ID if a valid connection exists.
        %
            
            r = obj.sim.simxGetConnectionId(obj.clientID);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        
        function loadScene(obj,scene,varargin)
        % VREP.loadScene
        % %% TODO
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
        % VREP.loadSimObject
        % %% TODO
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
            
            
            
            
            if ~strcmp(a,'None') && ~b
                
               error('RTB-SIM:VREP:','A full path to object must be given if using API files on path.');
                
            else    
                
               if ~b
                   
                   if model(1) ~= '/'
                       
                       model = fullfile(obj.path, 'models', [model '.ttm']);                    
                       
                   end
               end
                   
           end
            
            [r,id] = obj.sim.simxLoadModel(obj.clientID,model,b,obj.blocking_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end    
            
            pause(0.5)
            
        end
        
        function deleteObject(obj,handle)
        % VREP.deleteSimObject
        %
        % Deletes the specified object with a given handle from currently 
        % active V-REP scene. 
        % 
        % Note: Objects deleted while a simulation is
        % running will be reinstated when the scene resets (after
        % VREP.stopSim is called or the simulation is stopped using V-REP's 
        % GUI.
        %
        % Arguments:
        %
        %   handle      % A VREP object ID
        %
            
            r = obj.sim.simxRemoveModel(obj.clientID,handle,obj.setter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
           end
            
        end
        
        function closeScene(obj)
        % VREP.closeScene
        %
        % Closes the currently open scene and then switches to the next
        % open scene. If no other scenes are open, a new scene will be
        % created.
        %
            
            obj.stopSim();
            r = obj.sim.simxCloseScene(obj.clientID,obj.setter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        function time = pingSim(obj,n)
        % VREP.pingSim
        %
        % Ping the VREP API n times.
        % Returns a matrix of all resulting ping times.
        %
        % Arguments:
        %
        %   n           % Number of pings to send.
        %
        %
        % Returns:
        %
        %   time        % An n-length vector containing the time of each
        %                 ping.
        %
        
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
        % VREP.getObjects
        %
        % Show a list of all objects in the scene. Optionally specify a
        % particular type of object to return only objects of that type.
        % 
        % -----------------
        % Major Object Types:
        % -----------------
        % shape = sim_object_shape_type
        % joint = sim_object_joint_type
        % graph = sim_object_graph_type
        % camera = sim_object_camera_type
        % light = sim_object_light_type
        % dummy = sim_object_dummy_type
        % proximity_sensor = sim_object_proximitysensor_type
        % path = sim_object_path_type
        % vision_sensor = sim_object_visionsensor_type
        % mill = sim_object_mill_type
        % force_sensor = sim_object_forcesensor_type
        % mirror = sim_object_mirror_type
        % -----------------
        % Object Subtypes  
        % -----------------
        % omni_light = sim_light_omnidirectional_subtype
        % spot_light = sim_light_spot_subtype
        % directional_light = sim_light_directional_subtype
        % revolute_joint = sim_joint_revolute_subtype
        % prismatic_joint = sim_joint_prismatic_subtype
        % spherical_joint = sim_joint_spherical_subtype
        % simple_shape = sim_shape_simpleshape_subtype
        % multi_shape = sim_shape_multishape_subtype
        % ray_proximity_sensor = sim_proximitysensor_ray_subtype
        % pyramid_proximity_sensor = sim_proximitysensor_pyramid_subtype
        % cylinder_proximity_sensor = sim_proximitysensor_cylinder_subtype
        % disc_proximity_sensor = sim_proximitysensor_disc_subtype
        % cone_proximity_sensor = sim_proximitysensor_cone_subtype
        % pyramid_mill =  sim_mill_pyramid_subtype
        % cylinder_mill = sim_mill_cylinder_subtype
        % disc_mill = sim_mill_disc_subtype
        % cone_mill = sim_mill_cone_subtype  
        %
        % Arguments:
        %
        %   type            % Optionally specifies a type of object to
        %                     return.
        %
        % Returns:
        %
        %   objid           % A list of VREP object IDs.
        %   name            % A list of the object names.
        %
        
            
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
                        error("VREP.getObjects: An invalid/unknown type was specified! (Type: %d)",types);
                end
            end

            [r,objid,~,~,name] = obj.sim.simxGetObjectGroupData(obj.clientID, stype, 0, obj.blocking_mode);

           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end

        end        
        
        function [handle] = getHandle(obj, in, varargin)
        % VREP.getHandle
        %
        % Retrieves the V-REP identifier of an object given its string
        % name.
        %
        % Arguments:
        %
        %   in          % The string name
        %   
        % Optional Arguments:
        %
        %   fmt         % A format specifier. %% TODO Fill out the rest of
        %                 this
        %
        % Returns:
        %
        %   handle      % The V-REP object ID of the object with a name 
        %                 matching the one specified.
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
        % VREP.getName
        %
        % Returns the string name assigned to an object in the VREP
        % environment.
        %
        % Arguments:
        %
        %   objhandle       % A V-REP object ID 
        %
        % Returns:
        %
        %   name            % The name associated to the object. 
        %
            
            
            [r,objid,~,~,str] = obj.sim.simxGetObjectGroupData(obj.clientID, obj.sim.sim_appobj_object_type, 0, obj.blocking_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
            name = str(objid == objhandle,1);
            
        end
        
        function stype = getType(obj,objhandle,str_out)
        % VREP.getType
        %
        % Retruns the type of an object when given a VREP objectID. By 
        % default returns a string name of the object type as per the list 
        % below. Set str_out = false to return the type as a VREP API type
        % identifier.
        %
        % -----------------
        % Major Object Types:
        % -----------------
        % shape = sim_object_shape_type
        % joint = sim_object_joint_type
        % graph = sim_object_graph_type
        % camera = sim_object_camera_type
        % light = sim_object_light_type
        % dummy = sim_object_dummy_type
        % proximity_sensor = sim_object_proximitysensor_type
        % path = sim_object_path_type
        % vision_sensor = sim_object_visionsensor_type
        % mill = sim_object_mill_type
        % force_sensor = sim_object_forcesensor_type
        % mirror = sim_object_mirror_type
        % -----------------
        % Object Subtypes  
        % -----------------
        % omni_light = sim_light_omnidirectional_subtype
        % spot_light = sim_light_spot_subtype
        % directional_light = sim_light_directional_subtype
        % revolute_joint = sim_joint_revolute_subtype
        % prismatic_joint = sim_joint_prismatic_subtype
        % spherical_joint = sim_joint_spherical_subtype
        % simple_shape = sim_shape_simpleshape_subtype
        % multi_shape = sim_shape_multishape_subtype
        % ray_proximity_sensor = sim_proximitysensor_ray_subtype
        % pyramid_proximity_sensor = sim_proximitysensor_pyramid_subtype
        % cylinder_proximity_sensor = sim_proximitysensor_cylinder_subtype
        % disc_proximity_sensor = sim_proximitysensor_disc_subtype
        % cone_proximity_sensor = sim_proximitysensor_cone_subtype
        % pyramid_mill = sim_mill_pyramid_subtype
        % cylinder_mill = sim_mill_cylinder_subtype
        % disc_mill = sim_mill_disc_subtype
        % cone_mill = sim_mill_cone_subtype  
        %
        % Arguments:
        %   
        %   objhandle       % A V-REP object ID.
        %
        % Optional Arguments:
        %
        %   str_out         % Logical argument that switches the output 
        %                     between a string (str_out = true) or the VREP
        %                     API representation of object types (an
        %                     integer).
        %
        % Returns:
        %
        %   stype           % The type of object. Depending on str_out,
        %                     this will be either a string or a VREP API
        %                     constant.
        %
            
            if nargin < 3
                str_out = true;
            else
                if ~islogical(vrepout)
                    error("VREP.getType: str_out must be logical (true or false)");
                end  
            end
            
        
            [r,objid,types,~,~] = obj.sim.simxGetObjectGroupData(obj.clientID, obj.sim.sim_appobj_object_type, 1, obj.blocking_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
            types = types(objid == objhandle);
            
            if str_out
            
                switch (types)
                    case obj.sim.sim_object_shape_type
                        stype = 'shape';
                    case obj.sim.sim_object_joint_type
                        stype = 'joint';
                    case obj.sim.sim_object_graph_type
                        stype = 'graph';
                    case obj.sim.sim_object_camera_type
                        stype = 'camera';
                    case obj.sim.sim_object_light_type
                        stype = 'light';
                    case obj.sim.sim_object_dummy_type
                        stype = 'dummy';
                    case obj.sim.sim_object_proximitysensor_type
                        stype = 'proximity_sensor';
                    case obj.sim.sim_object_path_type
                        stype = 'path';
                    case obj.sim.sim_object_visionsensor_type
                        stype = 'vision_sensor';
                    case obj.sim.sim_object_mill_type
                        stype = 'mill';
                    case obj.sim.sim_object_forcesensor_type
                        stype = 'force_sensor';
                    case obj.sim.sim_object_mirror_type
                        stype = 'mirror';
                    case obj.sim.sim_light_omnidirectional_subtype
                        stype = 'omni_light';
                    case obj.sim.sim_light_spot_subtype
                        stype = 'spot_light';
                    case obj.sim.sim_light_directional_subtype
                        stype = 'directional_light';
                    case obj.sim.sim_joint_revolute_subtype
                        stype = 'revolute_joint';
                    case obj.sim.sim_joint_prismatic_subtype
                        stype = 'prismatic_joint';
                    case obj.sim.sim_joint_spherical_subtype
                        stype = 'spherical_joint';
                    case obj.sim.sim_shape_simpleshape_subtype
                        stype = 'simple_shape';
                    case obj.sim.sim_shape_multishape_subtype
                        stype = 'multi_shape';
                    case obj.sim.sim_proximitysensor_ray_subtype
                        stype = 'ray_proximity_sensor';
                    case obj.sim.sim_proximitysensor_pyramid_subtype
                        stype = 'pyramid_proximity_sensor';
                    case obj.sim.sim_proximitysensor_cylinder_subtype
                        stype = 'cylinder_proximity_sensor';
                    case obj.sim.sim_proximitysensor_disc_subtype
                        stype = 'disc_proximity_sensor';
                    case obj.sim.sim_proximitysensor_cone_subtype
                        stype = 'cone_proximity_sensor';
                    case obj.sim.sim_mill_pyramid_subtype
                        stype = 'pyramid_mill';
                    case obj.sim.sim_mill_cylinder_subtype
                        stype = 'cylinder_mill';
                    case obj.sim.sim_mill_disc_subtype
                        stype = 'disc_mill';
                    case obj.sim.sim_mill_cone_subtype
                        stype = 'cone_mill';
                    otherwise
                        error("VREP.getType: Object is an unknown type! (Type: %d)",types);
                end
                
            else
                
               stype = types; 
               
            end
                
        end
        
        function children = getChildren(obj,handle)
        % VREP.getChildren
        %
        % Returns all child objects associated with an object.
        %
        % Argumets:
        %
        %   handle      % A VREP Object ID
        %
        % Returns:
        %
        %   children    % An array containing the VREP object ID's of all
        %                 child objects found.
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
        
        function orient = getOrientation(obj,handle,rel2)
        % VREP.getOrientation
        %
        % Returns a set of euler angles representing an object's 
        % orientation in 3D space.
        %
        % Arguments:
        %
        %   handle      % A VREP object ID. 
        %
        % Optional Arguments:
        %
        %   rel2         % If specified, the orentation will be returned in
        %                 the reference frame of this object. 
        %
        % Returns:
        %
        %   orient       % A three vector containing the object's [X, Y, Z] 
        %                  rotations in Radians.
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
            

        function pos = getPosition(obj,handle,rel2)
        % VREP.getPosition
        % Returns the object's position in 3D space.
        %
        % Arguments:
        %
        %   handle      % A VREP object ID
        %
        % Optional Arguments:
        %
        %   rel2        % If specified, the position will be returned in
        %                 the reference frame of this object. 
        %
        % Returns:
        %
        %   pos         % A 3-vector representing the object's position
        %                 [X, Y, Z] in Meters.
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
        % VREP.setPosition
        % Sets an object's position in 3D space.
        %
        % Arguments:
        %
        %   handle      % A VREP object ID
        %   new         % A 3-vector containing the object's new position
        %                 [X,Y,Z] in Meters.
        %
        % Optional Arguments:
        %
        %   rel2        % If specified, the position will be set in
        %                 the reference frame of this object. 
        %
            
            if nargin < 4
                rel2 = -1;
            end
            
            r = obj.sim.simxSetObjectPosition(obj.clientID,handle,rel2,new,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        

        
        function setOrientation(obj,handle,orient,rel2)
        % VREP.getOrientation
        %
        % Returns a set of euler angles representing an object's 
        % orientation in 3D space.
        %
        % Arguments:
        %
        %   handle      % A VREP object ID. 
        %
        % Optional Arguments:
        %
        %   rel2         % If specified, the orentation will be returned in
        %                 the reference frame of this object. 
        %   orient       % A three vector containing the object's [X, Y, Z] 
        %                  rotations in Radians.
        %

            
            if nargin < 4
                rel2 = -1;
            end
                
            r = obj.sim.simxSetObjectOrientation(obj.clientID,handle,rel2,orient,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
%%      Joint Specific Methods

        function pos = getJointPosition(obj,handle)
        % VREP.getJointPosition
        %
        % Returns the intrinsic position of a joint. Cannot be used with
        % spherical joints
        %
        % Argumets:
        %
        %   handle      % A VREP object ID.
        %
        % Returns:
        %
        %   pos         % The current joint position in Radians.
        %
        
            opmode = obj.getter_mode;
        
            [r,pos] = obj.sim.simxGetJointPosition(obj.clientID,handle,opmode);

            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end


        
        end
        
        function matrix = getJointMatrix(obj,handle)
        % VREP.getJointMatrix
        %
        % Returns the intrinsic position of a spherical joint as a matrix.
        % 
        % Arguments:
        %
        %   handle      % A VREP object ID
        %
        % Returns:
        %
        %   matrix      % A 12 element matrix containing the intrinsic
        %                 position of the joints. Applicable only to
        %                 spherical joints.
        %
        
            opmode = obj.getter_mode;
        
            [r,matrix] = obj.sim.simxGetJointMatrix(obj.clientID,handle,opmode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end
        

        function force = getJointForce(obj,handle)
        % VREP.getJointForce
        %
        % Gets the force applied to a joint. For prismatic and revolute
        % joints only.
        %
        % NOTE!: 
        % When using the bullet physics engine, returns force or torque
        % applied to the joint motor.
        % When using the ODE or Vortex engines, returns the total force or
        % torque applied to a joint around/along its z-axis. 
        %
        % Arguments:
        %
        %   handle      % A VREP object ID
        %
        % Returns:
        %
        %   force       % The force applied in Nm. The exact result of this
        %                 is dependant on the Physics Engine in use. Please
        %                 see the description above for specifics.
        %                 

        
            opmode = obj.getter_mode;
        
            [r,force] = obj.sim.simxGetJointForce(obj.clientID,handle,opmode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
          
        function setSphericalJointMatrix(obj,handle,matrix)
        % VREP.setSphericalJointMatrix
        %
        % Sets the position of a spherical joint. This method has no effect
        % on other joint types.
        %
        % Arguments:
        %
        %   handle      % A VREP object ID
        %   matrix      % 12 element matrix containing intrinsic position.
        %

        
            r = obj.sim.simxSetSphericalJointMatrix(obj.clientID,handle,matrix,obj.setter_mode);
                      
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end
        
        function setJointForce(obj,handle,force)
        % VREP.setJointForce
        %
        % Sets the force acting on a joint. Joint must be dynamically
        % enabled and must be revolute or prismatic. This method is not
        % applicable to spherical joints.
        %
        % Arguments:
        %
        %   handle      % A VREP object ID.
        %   force       % A force in Nm.
        % 
        
        
            r = obj.sim.simxSetJointForce(obj.clientID,handle,force,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end
        
        function setJointPosition(obj,handle,pos)
        % VREP.setJointPosition
        %
        % Sets the position of a joint. This method will snap the joint to
        % the specified position, to rotate a joint using the joint motor,
        % see VREP.setJointTargetPosition
        % May have no effect with certain joint modes.
        %
        % Arguments:
        %
        %   handle      % A VREP object ID
        %   pos         % An angular position in Tadians.
        %
        
        
            r = obj.sim.simxSetJointPosition(obj.clientID,handle,pos,obj.setter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
            
        function setJointTargetPosition(obj,handle,pos)
        % VREP.setJointTargetPosition
        %
        % Sets joint target position. Only works if joint is in
        % torque/force mode and if its motor and position control are
        % enabled. The maximum torque and/or velocity will be as set in the
        % joint's dynamic properties dialog box in VREP.
        %
        % Arguments:
        %
        %   handle      % A VREP object ID.
        %   pos         % An angular position in Radians.
        %
        
            r = obj.sim.simxSetJointTargetPosition(obj.clientID,handle,pos,obj.setter_mode);

            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        function setJointTargetVelocity(obj,handle,vel)
        % VREP.setJointTargetVelocity
        %
        % Sets a non-spherical joint's target velocity. Joint needs to be
        % in torque/force mode with dynamics and joint motor enabled, and
        % position control disabled.
        %
        % Arguments:
        %
        %   handle      % A VREP object ID
        %   vel         % An angular velocity in Radians/sec
        %
        
            r = obj.sim.simxSetJointTargetVelocity(obj.clientID,handle,vel,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end
        
%% Object Parameters

        function setObjIntParam(obj,handle,param,new)
        % VREP.setObjIntParam
        %
        % Sets a specified integer parameter for a given object.
        %
        % Arguments:
        %
        %   handle      % A VREP object ID
        %   param       % The parameter identifier (use the numerical
        %                 identifier, not the VREP API prototype)
        %   new         % The new value. Must be an integer.
        %
            
            r = obj.sim.simxSetObjectIntParameter(obj.clientID, handle, param, new, obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        function setObjFloatParam(obj,handle,param,new)
        % VREP.setObjFloatParam
        %
        % Sets a specified float parameter for a given object.
        %
        % Arguments:
        %
        %   handle      % A VREP object ID
        %   param       % The parameter identifier (using the numerical
        %                 identifier and not the VREP API prototype seems
        %                 to be more reliable)
        %   new         % The new value. Must be a float.
        %    
        
            r = obj.sim.simxSetObjectFloatParameter(obj.clientID,handle,param, new, obj.setter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
                    
        end
        
        function res = getObjIntParam(obj,handle,param)
        % VREP.getObjIntParam
        %
        % Gets a specified integer parameter from a given object.
        %
        % Arguments:
        %
        %   handle      % A VREP object ID
        %   param       % The parameter identifier (using the numerical
        %                 identifier and not the VREP API prototype seems
        %                 to be more reliable)
        %
        % Returns:
        %
        %   res        % The value of the parameter.
        %
            
            [r, res] = obj.sim.simxGetObjectIntParameter(obj.clientID,handle,param,obj.getter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        function param = getObjFloatParam(obj,handle,param)
        % VREP.getObjFloatParam
        %
        % Gets a specified integer parameter from a given object.
        %
        % Arguments:
        %
        %   handle      % A VREP object ID
        %   param       % The parameter identifier (using the numerical
        %                 identifier and not the VREP API prototype seems
        %                 to be more reliable)
        %
        % Returns:
        %
        %   res        % The value of the parameter.
        %
            
            [r, param] = obj.sim.simxGetObjectFloatParameter(obj.clientID,handle,param,obj.getter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
           end
            
        end  

%% Parameter Management

        function res = getIntegerParam(obj,param)
        % VREP.getIntegerParam
        %
        % Gets a specified global integer parameter.
        %
        % Arguments:
        %
        %   param       % The parameter identifier (using the numerical
        %                 identifier and not the VREP API prototype seems
        %                 to be more reliable)
        %
        % Returns:
        %
        %   res        % The value of the parameter.
        %
            
            [r,res] = obj.sim.simxGetIntegerParameter(obj.clientID,param,obj.getter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function res = getFloatParam(obj,param)
        % VREP.getFloatParam
        %
        % Gets a specified global float parameter.
        %
        % Arguments:
        %
        %   param       % The parameter identifier (using the numerical
        %                 identifier and not the VREP API prototype seems
        %                 to be more reliable)
        %
        % Returns:
        %
        %   res        % The value of the parameter.
        %
            
            [r,res] = obj.sim.simxGetFloatingParameter(obj.clientID,param,obj.getter_mode);
          
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function res = getBooleanParam(obj,param)
        % VREP.getBooleanParam
        %
        % Gets a specified global boolean parameter.
        %
        % Arguments:
        %
        %   param       % The parameter identifier (using the numerical
        %                 identifier and not the VREP API prototype seems
        %                 to be more reliable)
        %
        % Returns:
        %
        %   res        % The value of the parameter.
        %
            
            [r,res] = obj.sim.simxGetBooleanParameter(obj.clientID,param,obj.getter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function res = getStringParam(obj,param)
        % VREP.getStringParam
        %
        % Gets a specified global string parameter.
        %
        % Arguments:
        %
        %   param       % The parameter identifier (using the numerical
        %                 identifier and not the VREP API prototype seems
        %                 to be more reliable)
        %
        % Returns:
        %
        %   res        % The value of the parameter.
        %
            [r,res] = obj.sim.simxGetStringParameter(obj.clientID,param,obj.getter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function setIntegerParam(obj,param,new)
        % VREP.setIntegerParam
        %
        % Sets a specified global integer parameter.
        %
        % Arguments:
        %
        %   param       % The parameter identifier (use the numerical
        %                 identifier, not the VREP API prototype)
        %   new         % New value. Must be an integer.
        %
            
            r = obj.sim.simxGetIntegerParameter(obj.clientID,param,new,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function setFloatParam(obj, param, new)
        % VREP.setFloatParam
        %
        % Sets a specified global float parameter.
        %
        % Arguments:
        %
        %   param       % The parameter identifier (use the numerical
        %                 identifier, not the VREP API prototype)
        %   new         % New value. Must be a float.
        %
            
            r = obj.sim.simxGetFloatParameter(obj.clientID,param,new,obj.setter_mode);
          
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function setBooleanParam(obj, param, new)
        % VREP.setBooleanParam
        %
        % Sets a specified global boolean parameter.
        %
        % Arguments:
        %
        %   param       % The parameter identifier (use the numerical
        %                 identifier, not the VREP API prototype)
        %   new         % New value. Must be boolean/logical.
        %
            
            r = obj.sim.simxGetBooleanParameter(obj.clientID,param,new,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function setStringParam(obj, param, new)
        % VREP.setStringParam
        %
        % Sets a specified global string parameter.
        %
        % Arguments:
        %
        %   param       % The parameter identifier (use the numerical
        %                 identifier, not the VREP API prototype)
        %   new         % New value. Must be a string.
        %
            
            r = obj.sim.simxGetStringParameter(obj.clientID,param,new,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        
            
%% Signal management

 
        function res = getIntegerSignal(obj,signal)
        % VREP.getIntegerSignal
        %
        % Retrieves an integer signal.
        %
        % Arguments:
        %
        %   signal      % A string identifying the desired signal.
        %
        % Returns:
        %
        %   res         % The contents of the signal.
        %
            
            [r,res] = obj.sim.simxGetIntegerSignal(obj.clientID,signal,obj.getter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function sig = getFloatSignal(obj,signal)
        % VREP.getFloatSignal
        %
        % Retrieves a float signal.
        %
        % Arguments:
        %
        %   signal      % A string identifying the desired signal.
        %
        % Returns:
        %
        %   res         % The contents of the signal.
        %    
            
            [r,sig] = obj.sim.simxGetFloatSignal(obj.clientID,signal,obj.getter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
           
        end

        function sig = getBooleanSignal(obj,signal)
        % VREP.getBooleanSignal
        %
        % Retrieves a boolean signal.
        %
        % Arguments:
        %
        %   signal      % A string identifying the desired signal.
        %
        % Returns:
        %
        %   res         % The contents of the signal.
        %    
                
            [r,sig] = obj.sim.simxGetBooleanSignal(obj.clientID,signal,obj.getter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
           
        end

        function sig = getStringSignal(obj,signal)
        % VREP.getStringSignal
        %
        % Retrieves a string signal.
        %
        % Arguments:
        %
        %   signal      A string identifying the desired signal.
        %
        % Returns:
        %
        %   res         % The contents of the signal.
        %        
        
            [r,sig] = obj.sim.simxGetStringSignal(obj.clientID,signal,obj.getter_mode);
        
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end

        function setIntegerSignal(obj,signal,new)
        % VREP.setIntegerSignal
        %
        % Sets an Integer signal. Caution! If a signal matching the given 
        % name is not present an error will not be thrown, instead a new 
        % signal will be created. This is a quirk of the VREP Remote API.
        %
        % Arguments:
        %
        %   signal      % A string identifying the desired signal.
        %   new         % The new signal value, this must be an Integer.
        %    
            
            r = obj.sim.simxSetIntegerSignal(obj.clientID,signal,new,obj.getter_mode);
        
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function setFloatSignal(obj,signal,new)
        % VREP.setFloatSignal
        %
        % Sets a Float signal. Caution! If a signal matching the given 
        % name is not present an error will not be thrown, instead a new 
        % signal will be created. This is a quirk of the VREP Remote API.
        %
        % Arguments:
        %
        %   signal      % A string identifying the desired signal.
        %   new         % The new signal value, this must be a Float.
        %    
            
            r = obj.sim.simxSetFloatSignal(obj.clientID,signal,new,obj.getter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function setBooleanSignal(obj,signal,new)
        % VREP.setBooleanSignal
        %
        % Sets a Boolean signal. Caution! If a signal matching the given 
        % name is not present an error will not be thrown, instead a new 
        % signal will be created. This is a quirk of the VREP Remote API.
        %
        % Arguments:
        %
        %   signal      % A string identifying the desired signal.
        %   new         % The new signal value, this must be a Boolean.
        %      
        
            r = obj.sim.simxSetBooleanSignal(obj.clientID,signal,new,obj.getter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
           
        end

        function setStringSignal(obj,signal,new)
        % VREP.setStringSignal
        %
        % Sets a String signal. Caution! If a signal matching the given 
        % name is not present an error will not be thrown, instead a new 
        % signal will be created. This is a quirk of the VREP Remote API.
        %
        % Arguments:
        %
        %   signal      % A string identifying the desired signal.
        %   new         % The new signal value, this must be a String.
        % 
            
            r = obj.sim.simxSetStringSignal(obj.clientID,signal,new,obj.getter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
           
        end
        
        function clearIntegerSignal(obj,signal)
        % VREP.clearIntegerSignal
        %
        % Removes a named Integer signal. If an empty string is passed
        % instead of a signal name, all Integer signals will be cleared. 
        %
        % Arguments:
        %   
        %   signal      % A string identifying the desired signal.
        %
            
            r = obj.sim.simxClearIntegerSignal(obj.clientID,signal,obj.setter_mode);
           
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end

        function clearFloatSignal(obj,signalName)
        % VREP.clearFloatSignal
        %
        % Removes a named Float signal. If an empty string is passed
        % instead of a signal name, all Float signals will be cleared. 
        %
        % Arguments:
        %   
        %   signal      % A string identifying the desired signal.
        %
               
            r = obj.sim.simxClearFloatSignal(obj.clientID,signalName,obj.setter_mode);
        
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end

        function clearBooleanSignal(obj,signalName)
        % VREP.clearBooleanSignal
        %
        % Removes a named Boolean signal. If an empty string is passed
        % instead of a signal name, all Boolean signals will be cleared. 
        %
        % Arguments:
        %   
        %   signal      % A string identifying the desired signal.
        %
            
            r = obj.sim.simxClearBooleanSignal(obj.clientID,signalName,obj.setter_mode);
        
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end

        function clearStringSignal(obj,signalName)
        % VREP.clearStringSignal
        %
        % Removes a named String signal. If an empty string is passed
        % instead of a signal name, all String signals will be cleared. 
        %
        % Arguments:
        %   
        %   signal      % A string identifying the desired signal.
        %
            
            r = obj.sim.simxClearStringSignal(obj.clientID,signalName,obj.setter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        

        
%% Image Sensor Handling


%         sim_visionfloatparam_near_clipping (1000): float parameter : near clipping plane
%         sim_visionfloatparam_far_clipping (1001): float parameter : far clipping plane
%         sim_visionintparam_resolution_x (1002): int32 parameter : resolution x
%         sim_visionintparam_resolution_y (1003): int32 parameter : resolution y
%         sim_visionfloatparam_perspective_angle (1004): float parameter : perspective projection angle
%         [r,msg] = obj.sim.simxGetFloatParameter(onj.clientID,target,obj.mode);
%         [r,msg] = obj.sim.simxGetIntegerParameter(obj.clientID,target,obj.mode);
        
        

        function img = readVisionSensor(obj,handle,grey)
        % VREP.readVisionSensor
        %
        % Retrieves an image from a V-REP vision sensor object. This is 
        % returned as a width-by-height-by-3 matrix or a
        % width-by-height-by-1 matrix, depending on whether greyscale mode 
        % is selected. To take an rgb image specify pass false into grey, 
        % or ignore completely. To take a greyscale image make grey = true.
        %
        % Arguments:
        %
        %   handle          % The VREP object ID of the sensor.
        %   grey            % Boolean that that determines whether the 
        %                     images is greyscale (true) or rgb (false) 
        % 
        % Returns
        %
        %   img             % An image matrix. Will be h_res-by-v_res-by-3
        %                     for and rgb image, or h_res-by-v_res-by-1 
        %                     for a greyscale image.
        %

            if nargin < 3
                grey = false;
            end
            
            [r,~,img] = obj.sim.simxGetVisionSensorImage2(obj.clientID,handle,grey,obj.getter_mode);

           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        

        function [auxData, dataIndex] = readPointVisionSensor(obj,handle)
        % VREP.readPointVisionSensor
        %
        % Retrieves data packets from a vision sensor. Use this when
        % retreiving data from a vision sensor that has had filters applied
        % to it. 
        %
        % Arguments:
        %
        %   handle      % VREP Object ID of vision sensor
        %
        % Returns:
        %
        %   auxData         % The data packets returned from the sensor
        %   dataIndex       % The indicies of each data packet returned in
        %                     auxData.
        %
            
            [r, ~, auxData, dataIndex] = obj.sim.simxReadVisionSensor(obj.clientID, handle, obj.getter_mode);
        
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
        
        end
        

        function [img] = readVisionSensorDepth(obj,handle)
        % VREP.readVisionSensorDepth
        %
        % Retrieves a greyscale depth map image from a V-REP vision sensor
        %
        % Arguments:
        %
        %   handle      % VREP object ID of sensor
        %
        % Returns:
        %
        %   img         % An image matrix
        %
            
            [r,~,img] = obj.sim.simxGetVisionSensorDepthBuffer2(obj.clientID,handle,obj.getter_mode);
            
            if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
   
        

%% Other sensors



        function [state,torque,force] = readForceSensor(obj,handle)
        % VREP.readForceSensor
        %
        % Retrieve data from a V-REP force sensor. Returns the current
        % state (broken/unbroken) as well as the torque and force applied
        % to the sensor.
        %
        % Arguments:
        %
        %   handle      % V-REP object ID of force sensor.
        %
        % Returns:
        %
        %   state       % Logical value that represents the state (broken
        %                 or unbroken) of the sensor.
        %   torque      % Torque acting on the sensor in Newton Meters
        %   force       % Force acting on the sensor in Newtons.
        %
            
           [r,state,torque,force] = obj.sim.simxReadForceSensor(obj.clientID,handle,obj.getter_mode);
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        

        function [state,point] = readProximitySensor(obj,handle)
        % VREP.readProximitySensor
        % 
        % Reads a V-REP proximity sensor.  
        % 
        % Arguments:
        %
        %   handle      % A proximity sensor V-REP object ID 
        %
        % Returns:
        %
        %   state       % Logical, whether the proximity sensor has been
        %                 triggered.
        %   point       % The coordinates of the triggered point w.r.t. 
        %                 sensor.
        %
          
     
           [r,state,point,~,~] = obj.sim.simxReadProximitySensor(obj.clientID,handle,obj.getter_mode); % [r,state,point,found_obj,found_surface] 
            
           if r ~= 0 && r ~= 1
                throw(obj.errcheck(r))
            end
            
        end
        
        
%% Simulation Object Factories.

        function out = entity(obj,handle)
        % VREP.entity
        %
        % Generates an entity object from either an object name or object
        % ID.
        %
        % Arguments:
        %
        %   handle       % A string name or VREP object ID. 
        %
        % Returns:
        %
        %   out          % An entity object. 
        %
            
            out = sim_entity(obj,handle);

        end
        
        function out = joint(obj,handle)
        % VREP.joint
        %
        % Generates an joint object from either an object name or object
        % ID.
        %
        % Arguments:
        %
        %   handle       % A string name or VREP object ID. 
        %
        % Returns:
        %
        %   out          % A joint object. 
        %
            
            out = sim_joint(obj,handle);
            
        end
                
        function out = rgb_sensor(obj,handle)
        % VREP.rgb_sensor
        %
        % Generates a rgb sensor object from either an object name or object
        % ID.
        %
        % Arguments:
        %
        %   handle       % A string name or VREP object ID. 
        %
        % Returns:
        %
        %   out          % A rgb sensor object. 
        %
            
            out = sim_rgb_sensor(obj,handle);
            
        end
        
        function out = xyz_sensor(obj,handle)
        % VREP.xyz_sensor
        %
        % Generates a XYZ sensor object from either an object name or object
        % ID.
        %
        % Arguments:
        %
        %   handle       % A string name or VREP object ID. 
        %
        % Returns:
        %
        %   out          % A xyz sensor object. 
        %
            
            out = sim_xyz_sensor(obj,handle);
            
        end
        
        function out = xy_sensor(obj,handle)
        % VREP.xy_sensor
        % 
        % Generates a XY sensor object from either an object name or object
        % ID.
        %
        % Arguments:
        %
        %   handle       % A string name or VREP object ID. 
        %
        % Returns:
        %
        %   out          % A xy sensor object. 
        %
            out = sim_xy_sensor(obj,handle);
        
        end
        
        function out = rgbdCamera(obj,handle)
        % VREP.rgbdCamera
        %
        % Generates a RGBD camera object from either an object name or object
        % ID.
        %
        % Arguments:
        %
        %   handle       % A string name or VREP object ID. 
        %
        % Returns:
        %
        %   out          % A rgbd sensor object. 
        %
            
            out = sim_cameraRGBD(obj,handle);
        
        end
        
        function out = forceSensor(obj,handle)
        % VREP.forceSensor
        %
        % Generates a force sensor object from either an object name or 
        % object ID.
        %
        % Arguments:
        %
        %   handle       % A string name or VREP object ID. 
        %
        % Returns:
        %
        %   out          % A force sensor object. 
        %
            
           out = sim_force_sensor(obj,handle);
            
        end
        
        function out = hokuyo(obj,handle,ref)
        % VREP.hokuyo
        %    
        % Generates a Fast Hokuyo object from either an object name or 
        % object ID.
        %
        % Arguments:
        %
        %   handle       % A string name or VREP object ID. 
        %
        % Returns:
        %
        %   out          % A hokuyo object. 
        %
             
            if nargin < 3
                out = sim_fast_hokuyo(obj,handle);
            else
                out = sim_fast_hokuyo(obj,handle,ref);
            end
            
        end
        
        
        function out = arm(obj,base_handle,joint_handle,fmt)
        % VREP.arm
        %
        % Generates an Arm object from either an object name or 
        % object ID.
        %
        % Arguments:
        %
        %   handle       % A string name or VREP object ID. 
        %
        % Returns:
        %
        %   out          % An arm object. 
        %
            if nargin < 4
                [list,num] = obj.armhelper(joint_handle);
            else
                [list,num] = obj.armhelper(joint_handle,fmt);
            end
            
             out = sim_arm(obj, base_handle, list, num);
            
        end
        
        function out = youBot(obj,handle)
        % VREP.youBot
        %
        % Generates a youBot object from either an object name or 
        % object ID.
        %
        % Arguments:
        %
        %   handle       % A string name or VREP object ID. 
        %
        % Returns:
        %
        %   out          % An youBot object. 
        %
            out = sim_youBot(obj,handle);
            
        end
        
        function out = youBotTRS(obj,handle)
        % VREP.youBotTRS
        %
        % Generates a TRS modified youBot object from either an object name
        % or object ID.
        %
        % Arguments:
        %
        %   handle       % A string name or VREP object ID. 
        %
        % Returns:
        %
        %   out          % An arm object. 
        %

            out = sim_youBot_TRS(obj,handle);
            
        end
        
        function out = diffBot(obj,handle)
        % VREP.diffBot
        %
        % Generates a diffBot object from either an object name
        % or object ID.
        %
        % Arguments:
        %
        %   handle       % A string name or VREP object ID. 
        %
        % Returns:
        %
        %   out          % An arm object. 
        %
            
            out = sim_diffBot(obj,handle);
        
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
                % if h == 0
                %   break
                % end
                catch ME % I'm almost ashamed of this.
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
    
    %% Super secret private stuff. 
    
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
 
end
   


