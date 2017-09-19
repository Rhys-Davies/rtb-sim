%% Entity Superclass %%
%
% A superclass that describes a generic object in the simulation world.
% Subclass for specific object types, can be used without subclassing.
% Handles common elements between all world objects.
% Pose, orientation, position.
%
%
% Properties:
%   id                  The handle or ID assigned to the object by the simulator
%   name                The string name assigned to the object.
%   sim                 The simulator object, must be passed in as an initialization argument
%
%
% Methods:
%   pose                Gets the pose of an object relative to another
%   position            Gets the global position of the object
%   orientation         Gets the global orientation of the object
%   get_IntParam        Retrieves the value of object's named integer paramter.
%   get_FloatParam      Retrieves the value of object's named float paramter.
%
%
%   set_pose            Sets the pose of an object relative to another
%   set_position        Sets the global position of the object
%   set_orientation     Sets the global orientation of the object
%   set_IntParam        Sets object's named integer parameter to a given value.
%   set_FloatParam      Sets object's name float parameter to a given value.

classdef sim_entity < handle
    
    properties
        id;
        name;
        sim;
%         stream_pos; % Is position streamed?.
%         stream_orient; % Is orientatoin streamed?. 
%         stream_pos_rel2; % Is position streamed?.
%         stream_orient_rel2; % Is orientatoin streamed?.
    end
    
    methods
        
        function obj = sim_entity(sim,ident,varargin)
            
            % Add option to stream position and orientation
            
%              p = inputParser;
%              
%              stream_pos_default = 'None';
%              stream_orient_default = 'None';
%              
%              stream_pos_rel2_default = -1;
%              stream_pos_rel2_default = -1;
%              
%              addOptional(p,'Stream_Pos',stream_pos_default);
%              addOptional(p,'Stream_Orient',stream_orient_default);
%              
%              addOptional(p,'Stream_Pos_Relto',stream_pos_rel2_default);
%              addOptional(p,'Stream_Orient_Relto',stream_pos_rel2_default);
%              
%              parse(p,varargin(:));
%              
%              obj.stream_pos = p.Results.Stream_Pos;
%              obj.stream_orient = p.Results.Stream_Orient;
%              obj.stream_pos_rel2 = p.Results.Stream_Orient_Relto;
%              obj.stream_orient_rel2 = p.Results.Stream_Orient_Relto;
             
             
            obj.sim = sim;
            if isstr(ident)
                obj.id = obj.sim.getHandle(ident);
                obj.name = ident;
            else
                obj.id = ident;
                obj.name = obj.sim.getName(ident);
            end
            
        end
        
        function destroy(obj)
        %% sim_entity.destroy
        % Deletes the model from the scene
        
            obj.sim.deleteSimObject(obj.id)
            
        end
        
        function out = pose(obj,rel2)
        %% sim_entity.pose
        % Retrieves the current pose of the object.
        % If rel2 is a valid scene object, the pose will be returned with
        % respect to that object.
        
            if nargin < 2 
                rel2 = -1;
            elseif nargin == 2 && rel2 ~= -1 && isa(rel2,'sim_entity')
                rel2 = rel2.id;
            end
            
            pos = obj.sim.getPosition(obj.id,rel2);
            eul = obj.sim.getOrientation(obj.id,rel2);
            
            out = transl(pos) * eul2tr(eul, 'deg');
        
        end
        
        function out = position(obj,rel2) % position(obj,varargin)
        %% sim_entity.position
        % Returns [x,y,z] positions of the item 
        %
        

        
            if nargin < 2 
                rel2 = -1;
            elseif nargin == 2 && rel2 ~= -1 && isa(rel2,'sim_entity')
                rel2 = rel2.id;
            end
            
            out = obj.sim.getPosition(obj.id,rel2);
            
            
            
        end
        
        
        
        
          
%         function out = position(obj,rel2,varargin) % position(obj,varargin)
%         %% sim_entity.position
%         % Returns [x,y,z] positions of the item 
%         %        
%         % Optional Arguments:
%         %           
%         % 'stream'          % A boolean value. Stream this attribute to buffer. Use this
%         %                     where blocking for a return value is too slow. 
%         %          
% 
%              p = inputParser;
%              defaultStream = false;
%              addOptional(p,'stream',defaultStream,@islogical);
%              parse(p,varargin(:));
%              stream = p.Results.stream;
% 
%             if isa(rel2,'sim_entity')
%                   rel2 = rel2.id;
%             end
% 
%             if stream == true && obj.stream_pos == false
% 
%                   op = 'start_stream';
%                   [~] = obj.sim.getPosition(obj.id,rel2,op);
%                   obj.stream_pos = true;
%                   pause(0.2) % Make sure the buffer fills
%                   op = 'buffer';
% 
%             elseif stream == true && obj.stream_pos == true
% 
%                   op = 'buffer';
% 
%             elseif stream == false && obj.stream_pos == true
% 
%                   op = 'stop_stream';
%                   obj.stream_pos = false;
% 
%             else
% 
%                   op = 'default';
%             end
% 
% 
%             out = obj.sim.getPosition(obj.id,rel2,op); 
%             
%         end
        
        
        function out = orientation(obj,rel2)
            
            if nargin < 2
                rel2 = -1;
            elseif nargin == 2 && rel2 ~= -1 && isa(rel2,'sim_entity')
                rel2 = rel2.id;
            end
            
            out = obj.sim.getOrientation(obj.id,rel2);
            
        end
        
        function out = get_signal(obj,sig,dtype)
            
            out = obj.sim.getSignal(sig,dtype);
            
        end

        % Setters
        function set_pose(obj,new,rel2)
            
            if nargin < 3 
                rel2 = -1;
            elseif nargin == 2 && rel2 ~= -1 && isa(rel2,'sim_entity')
                rel2 = rel2.id;
            end
            
            pos = transl(new);
            eul = tr2eul(new, 'deg');
            
            obj.setPosition(handle,pos,rel2);
            obj.setOrientation(handle,eul,rel2);
            
        end
        
        
        function set_position(obj,new,rel2)
            
            if nargin < 3 
                rel2 = -1;
            elseif nargin == 2 && rel2 ~= -1 && isa(rel2,'sim_entity')
                rel2 = rel2.id;
            end
            
            obj.sim.setPosition(obj.id,new,rel2);
            
        end
        
        function set_orientation(obj,new,rel2)
            
            if nargin < 3 
                rel2 = -1;
            elseif nargin == 2 && rel2 ~= -1 && isa(rel2,'sim_entity')
                rel2 = rel2.id;
            end
            
            obj.sim.setOrientation(obj.id,new,rel2); 
            
        end
        
        function set_IntParam(obj,param,new)
   
            obj.sim.setObjIntParam(obj.id,param,new);
            
        end
        
        function set_FloatParam(obj,param,new)
            
            obj.sim.setObjFloatParam(obj.id,param,new);
            
        end
        
        function param = get_IntParam(obj,param)

            param = obj.sim.getObjIntParam(obj.id,param);
            
        end
        
        function param = get_FloatParam(obj,param)
            
            param = obj.sim.getObjFloatParam(obj.id,param,new);
            
        end


    end
    
end

