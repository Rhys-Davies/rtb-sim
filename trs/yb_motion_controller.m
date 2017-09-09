classdef yb_motion_controller < handle
    %YB_MOTION_CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
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
        
        function vel = update(obj, forwBackVel, leftRightVel, rotVel)
        
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

            vel1 = -forwBackVel-leftRightVel+rotVel;
            vel2 = -forwBackVel+leftRightVel+rotVel;
            vel3 = -forwBackVel-leftRightVel-rotVel;
            vel4 = -forwBackVel+leftRightVel-rotVel;
            
            vel = [vel1,vel2,vel3,vel4];
            
        end
          
    end
    
end
