%% sim_entity %%
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
%   pose                Gets the pose of an object. A 4x4 matrix.
%   position            Gets the global position of the object in meters.
%   orientation         Gets the global orientation of the object in radians.
%   get_intParam        Retrieves the value of object's named integer paramter.
%   get_floatParam      Retrieves the value of object's named float paramter.
%
%
%   set_pose            Sets the pose of an object. A 4x4 matrix.
%   set_position        Sets the global position of the object in meters
%   set_orientation     Sets the global orientation of the object in radians.
%   set_intParam        Sets object's named integer parameter to a given value.
%   set_floatParam      Sets object's name float parameter to a given value.
%

classdef sim_entity < handle
    
    properties
        id;
        name;
        sim;
    end
    
    methods
        
        function obj = sim_entity(sim,ident)
            
                         
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
        % sim_entity.destroy
        % Deletes the model from the scene
        
            obj.sim.deleteSimObject(obj.id)
            
        end
        
        function out = pose(obj,rel2)
        % sim_entity.pose
        %
        % Retrieves the current pose of the object.
        % If rel2 is a valid scene object, the pose will be returned with
        % respect to that object.
        %
        % % NOTE: V-REP RETURNS AND EXPECTS EULER ANGLES IN 'XYZ' FORMAT!!
        %
        % Argument:
        % 
        %   rel2        % An object to use as a reference frame. Set to -1
        %                 or leave blank to use world reference frame.
        %
        % Returns:
        %
        %   out         % A homogeneous transform representing the objects
        %                 pose
        %
        
        
            if nargin < 2 
                rel2 = -1;
            elseif nargin == 2 && isa(rel2,'sim_entity')
                rel2 = rel2.id;
            end
            
            pos = obj.sim.getPosition(obj.id,rel2);
            eul = obj.sim.getOrientation(obj.id,rel2);
            
            out = transl(pos) * trotx(eul(1)) * troty(eul(2)) * trotz(eul(3));
            
        end
        
        function out = position(obj,rel2)
        % sim_entity.position
        %
        % Returns positions of the item in Meters.
        %
        % Arguments:
        %
        %   rel2        % An object to use as a reference frame. Set to -1
        %                 or leave blank to use world reference frame.
        % 
        % Returns:
        %
        %   out         % A 3-vector containing the position of the object
        %                 in the form [x,y,z]. Measurements are in meters. 
        %
        

        
            if nargin < 2 
                rel2 = -1;
            elseif nargin == 2 && rel2 ~= -1 && isa(rel2,'sim_entity')
                rel2 = rel2.id;
            end
            
            out = obj.sim.getPosition(obj.id,rel2);
            
            
            
        end
        
        
        function out = orientation(obj,rel2)
        % sim_entity.orientation
        % 
        % Gets the orientation of the object.
        %
        % NOTE: V-REP RETURNS AND EXPECTS EULER ANGLES IN 'XYZ' FORMAT!!
        %
        % Arguments:
        %   
        %   rel2        % An object to use as a reference frame. Set to -1
        %                 or leave blank to use world reference frame.
        %   
        % Returns:
        %
        %   out         % A 3-vector containing euler angles representing
        %                 the objects orientation in [X,Y,Z] form. All
        %                 angles are in radians.
        %
        
            if nargin < 2
                rel2 = -1;
            elseif nargin == 2 && rel2 ~= -1 && isa(rel2,'sim_entity')
                rel2 = rel2.id;
            end
            
            out = obj.sim.getOrientation(obj.id,rel2);
            
        end
        

        % Setters
        
        function set_pose(obj,new,rel2)
        % sim_entity.set_pose
        % 
        % Set the pose of an object using a homogeneous transform.
        %
        % Arguments
        %   
        %   new         % A 4x4 homogeneous transform.
        %   rel2        % An object to use as a reference frame. Set to -1
        %                 or leave blank to use world reference frame.   
        %
        
            if nargin < 3 
                rel2 = -1;
            elseif nargin == 2 && rel2 ~= -1 && isa(rel2,'sim_entity')
                rel2 = rel2.id;
            end
            
            % NOTE: V-REP RETURNS AND EXPECTS EULER ANGLES IN 'XYZ' FORMAT!
            
            pos = transl(new);
            eul = tr2rpy(new);
            
            obj.setPosition(handle,pos,rel2);
            obj.setOrientation(handle,eul,rel2);
            
        end
        
        
        function set_position(obj,new,rel2)
        % sim_entity.set_position
        %
        % Sets the position of the object.
        %
        % Arguments:
        %   
        %   new         % A 3-vector of coordinates in the form [x,y,z].
        %                 Coordinates are in meters.
        %   rel2        % An object to use as a reference frame. Set to -1
        %                 or leave blank to use world reference frame.
        %
        
            if nargin < 3 
                rel2 = -1;
            elseif nargin == 2 && rel2 ~= -1 && isa(rel2,'sim_entity')
                rel2 = rel2.id;
            end
            
            obj.sim.setPosition(obj.id,new,rel2);
            
        end
        
        function set_orientation(obj,new,rel2)
        % sim_entity.set_orientation
        %
        % Sets the orientation of the object. 
        %
        % NOTE: V-REP RETURNS AND EXPECTS EULER ANGLES IN 'XYZ' FORMAT!!
        %
        % Arguments:
        %  
        %   new         % A 3-vector of euler angles in the form [X,Y,Z].
        %                 Angles are in radians. 
        %   rel2        % An object to use a reference frame. -1 or leave
        %                 black to use world reference.
        %
            if nargin < 3 
                rel2 = -1;
            elseif nargin == 2 && rel2 ~= -1 && isa(rel2,'sim_entity')
                rel2 = rel2.id;
            end
            
            obj.sim.setOrientation(obj.id,new,rel2); 
            
        end
        
        function set_IntParam(obj,param,new)
        % sim_entity.set_IntParam
        %
        % Sets a specified parameter of datatype Integer.
        % See http://www.coppeliarobotics.com/helpFiles/en/objectParameterIDs.htm
        % for a complete list of all parameters avaliable in V-REP. Some
        % are specific to the type of object.
        %
        % Arguments:
        %
        %   param       % The numerical parameter identifier assigned by
        %                 V-REP.
        %   new         % The new value of the parameter.
        %   
        %
   
            obj.sim.setObjIntParam(obj.id,param,new);
            
        end
        
        function set_FloatParam(obj,param,new)
        % sim_entity.set_FloatParam
        %
        % Sets a specified parameter of datatype Float.
        % See http://www.coppeliarobotics.com/helpFiles/en/objectParameterIDs.htm
        % for a complete list of all parameters avaliable in V-REP. Some
        % are specific to the type of object.
        %
        % Arguments:
        %
        %   param       % The numerical parameter identifier assigned by
        %                 V-REP.
        %   new         % The new value of the parameter.
        %   
        %
            
            obj.sim.setObjFloatParam(obj.id,param,new);
            
        end
        
        function param = get_IntParam(obj,param)
        % sim_entity.get_IntParam
        %
        % Gets a specified parameter of datatype Integer.
        % See http://www.coppeliarobotics.com/helpFiles/en/objectParameterIDs.htm
        % for a complete list of all parameters avaliable in V-REP. Some
        % are specific to the type of object.
        %
        % Arguments:
        %
        %   param       % The numerical parameter identifier assigned by
        %                 V-REP.
        %
        % Returns
        %
        %   new         % The current value of the parameter.
        %   

            param = obj.sim.getObjIntParam(obj.id,param);
            
        end
        
        function param = get_FloatParam(obj,param)
        % sim_entity.get_FloatParam
        %
        % Gets a specified parameter of datatype Float.
        % See http://www.coppeliarobotics.com/helpFiles/en/objectParameterIDs.htm
        % for a complete list of all parameters avaliable in V-REP. Some
        % are specific to the type of object.
        %
        % Arguments:
        %
        %   param       % The numerical parameter identifier assigned by
        %                 V-REP.
        %
        % Returns
        %
        %   new         % The current value of the parameter.
        %   
            
            param = obj.sim.getObjFloatParam(obj.id,param);
            
        end


    end
    
end

