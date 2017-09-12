classdef Gazebo < simulator
    %GAZEBO Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        
        %% Generic Simulation Management
        function pauseComms(obj,status)
        end
        
        function pauseSim(obj)  
        end
        function startSim(obj)
        end
        
        function stopSim(obj)
        end
        
        function loadScene(obj,path,opt)
        end
        
        function id = loadSimObject(obj,handle,opt)
        end
        
        function deleteSimObject(obj,handle)
        end
        
        function closeScene(obj)
        end
        
        function time = pingSim(obj,n)
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
        
        function matrix = getJointMatrix(obj,handle)
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
        
      
        %% Signal Management
        function setSignal(obj,msg,target,dtype)
        end
        
        function sig = getSignal(obj,target,dtype)
        end
        
        function clearSignal(obj,target,dtype)
        end
        
        %% Parameter Management
        function setParam(obj,target,msg,dtype)
        end
        
        function msg = getParam(obj,target,dtype)
        end
        
        
        %% Image Sensor Handling
        function [res,img] = readVisionSensor(obj,target)
        end
        
        function [res,img] = readVisionSensorDepth(obj,target)
        end
        
 
        %% Other Sesors
        function [state,torque,force] = readForceSensor(obj,target)
        end
        
        %% Simulation Objects        
        function out = entity(sim,ident)
        end
        
        function out = joint(sim,ident,type)
        end
        
        function out = rgbCamera(sim,ident,type)
        end
        
        function out = xyzCamera(sim,ident,type)
        end
        
        function out = forceSensor(sim,ident)
        end
        
        function out = lidar(sim,ident)
        end
        
        function out = hokuyo(sim,ident)
        end
        
        function out = robot(sim,ident)
        end
        
        function out = arm(sim,ident)
        end
        
        function out = youbot(sim,ident)
        end
        
        
    end
    
end

