% Returns a list of wheel velocities to achieve the given forward, lateral
% and rotational velocities. The velocities are normalized to say
% within the bounds of the actuator capabilities.

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)

% Modified Rhys Davies 2017
% Converted into a class and modified to output a vector of velocities
% instead of directly actuating the the wheels.
% Output is: vel = frontleft, rearleft,rearright,frontright;

classdef yb_motion_controller < handle

    properties
       previousForwBackVel
       previousLeftRightVel
       previousRotVel
       
       pParam
       maxV
       pParamRot
       maxVRot
       accelF
       
    end
    
    methods
        
        function obj = yb_motion_controller(pParam,maxV,pParamRot,maxVRot,accelF)
        
            obj.previousForwBackVel = 0;
            obj.previousLeftRightVel = 0;
            obj.previousRotVel = 0;
            
            obj.pParam = pParam;
            obj.maxV = maxV;
            obj.pParamRot = pParamRot;
            obj.maxVRot = maxVRot;
            obj.accelF = accelF;
        
        end
        
        function out = update(obj, forwBackVel, leftRightVel, rotVel)
        
            forwBackVel=forwBackVel*obj.pParam;
            leftRightVel=leftRightVel*obj.pParam;
            v=sqrt(forwBackVel*forwBackVel+leftRightVel*leftRightVel);
            if v>obj.maxV,
                forwBackVel=forwBackVel*obj.maxV/v;
                leftRightVel=leftRightVel*obj.maxV/v;
            end;
            rotVel=rotVel*obj.pParamRot;
            if (abs(rotVel)>obj.maxVRot),
                rotVel=obj.maxVRot*rotVel/abs(rotVel);
            end;

            df=forwBackVel-obj.previousForwBackVel;
            ds=leftRightVel-obj.previousLeftRightVel;
            dr=rotVel-obj.previousRotVel;

            if (abs(df)>obj.maxV*obj.accelF),
                df=abs(df)*(obj.maxV*obj.accelF)/df;
            end;

            if (abs(ds)>obj.maxV*obj.accelF),
                ds=abs(ds)*(obj.maxV*obj.accelF)/ds;
            end;

            if (abs(dr)>obj.maxVRot*obj.accelF),
                dr=abs(dr)*(obj.maxVRot*obj.accelF)/dr;
            end;

            forwBackVel=obj.previousForwBackVel+df;
            leftRightVel=obj.previousLeftRightVel+ds;
            rotVel=obj.previousRotVel+dr;
            obj.previousForwBackVel=forwBackVel;
            obj.previousLeftRightVel=leftRightVel;
            obj.previousRotVel=rotVel;
            
            out = [forwBackVel,leftRightVel,rotVel];
            
        end
          
    end
    
end
