classdef TelloKF < handle
    %TELLOEKF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x=[] % x,y,z,w,xp,yp,zp,wp
        P=[]
        H=eye(4,8)
    end
    
    methods
        function obj = TelloKF(x_0,P_0)
            obj.x=x_0;
            obj.P=P_0;
        end
        
        function update(obj,obs,noise)
            y = obs-obj.H*obj.x;
            y(4) = mod(y(4),2*pi);
            
            if(y(4)>pi)
                y(4)=y(4)-2*pi;
            end
            
            S = obj.H*obj.P*(obj.H.')+noise;
            K=obj.P*(obj.H.')/S;
            obj.x=obj.x+K*y;
            obj.P = (eye(8)-K*obj.H)*obj.P;
            
            obj.x(4) = mod(obj.x(4),2*pi);
            if(obj.x(4)>pi)
                obj.x(4)=obj.x(4)-2*pi;
            end
        end
        
        function predict(obj,ts,noise)
            F = [eye(4),eye(4)*ts;zeros(4),eye(4)];
            
            obj.x=F*obj.x;
            obj.P=F*obj.P*(F.') + noise;
            
            if(obj.x(4)>pi)
                obj.x(4)=obj.x(4)-2*pi;
            elseif(obj.x(4)<pi)
                obj.x(4)=obj.x(4)+2*pi;
            end
        end
    end
end

