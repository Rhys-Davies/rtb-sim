%% Simulator Abstract Superclass %%
%
% Must be subclassed for each specific simulation environment that is
% desired.
%
% Functions that absolutely must exist in all simulator subclasses!
% Others can be added as needed to handle extra features or simulator
% specific methods required to implement these abstract methods.
% These methods will define the minimum feature set each simulator
% specific interface will be required to support.


classdef (Abstract) simulator < handle
    % An abstract superclass that defines the minimum feature set
    % for all simulation package interfaces.

    properties
        clientID
        IP
        PORT
        sim
    end

    methods (Abstract)
        
        %% Generic Simulation Management
        pauseComms(obj,status)
        pauseSim(obj)  
        startSim(obj) 
        stopSim(obj)
        loadScene(obj,path,opt)
        id = loadSimObject(obj,handle,opt)
        deleteSimObject(obj,handle)
        closeScene(obj)
        time = pingSim(obj,n)
        
        %% Generic object management
        [objid,name] = getObjects(obj,type)
        [handle] = getHandle(obj,name)
        name = getName(obj,objhandle)
        types = getTypes(obj,objhandle)
        children = getChildren(obj,handle)
        orient = getOrientation(obj,handle,rel2)             
        pos = getPosition(obj,handle,rel2)
        setPosition(obj,handle,new,rel2)        
        setOrientation(obj,handle,orient,rel2)
        
        %% Joint Specific Methods
        pos = getJointPosition(obj,handle)       
        matrix = getJointMatrix(obj,handle)
        force = getJointForce(obj,handle)
        setSphericalJointMatrix(obj,handle,matrix)
        setJointForce(obj,handle,force)
        setJointPosition(obj,handle,pos)
        setJointTargetPosition(obj,handle,pos)      
        setJointTargetVelocity(obj,handle,vel)
        
        %% Image Sensor Handling
        [res,img] = readVisionSensor(obj,target)
        [res,img] = readVisionSensorDepth(obj,target)
 
        %% Other Sesors
        [state,torque,force] = readForceSensor(obj,target)
        
        %% Simulation Objects        
        out = entity(sim,ident)
        out = joint(sim,ident,type)
        out = rgb_sensor(sim,ident)
        out = xyz_sensor(sim,ident)
        out = rgbdCamera(sim,ident)
        out = forceSensor(sim,ident)
        out = hokuyo(sim,ident)
        %out = robot(sim,ident)
        out = arm(sim,ident)
        out = youbot(sim,ident)
        
    end
    
    methods
        function sim = simulator(varargin)
        end
    end

end
