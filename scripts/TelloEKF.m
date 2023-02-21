classdef TelloEKF < handle
    %TELLOEKF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x=[] % x,y,z,w,xp,yp,zp,wp
        P=[]
    end
    
    methods
        function obj = TelloEKF(x_0,P_0)
            obj.x=x_0;
            obj.P=P_0;
        end
        
        function update(obj,obs,noise)
            y = obs-TelloEKF.h(obj.x);
            y(4) = mod(y(4),2*pi);
            
            if(y(4)>pi)
                y(4)=y(4)-2*pi;
            end
            
            H_k=TelloEKF.H(obj.x);
            S = H_k*obj.P*(H_k.')+noise;
            K=obj.P*(H_k.')/S;
            obj.x=obj.x+K*y;
            obj.P = (eye(8)-K*H_k)*obj.P;
            
            obj.x(4) = mod(obj.x(4),2*pi);
            if(obj.x(4)>pi)
                obj.x(4)=obj.x(4)-2*pi;
            end
        end
        
        function predict(obj,u,noise)
            obj.x=TelloEKF.f(obj.x,u);            
            F_k = TelloEKF.F(obj.x,u);
            obj.P=F_k*obj.P*(F_k.') + noise;
            
            if(obj.x(4)>pi)
                obj.x(4)=obj.x(4)-2*pi;
            elseif(obj.x(4)<pi)
                obj.x(4)=obj.x(4)+2*pi;
            end
        end
    end
    methods(Static)
        
        function new_state = f(state,u)
            x=state(1);
            y=state(2);
            z=state(3);
            w=state(4);
            xp=state(5);
            yp=state(6);
            zp=state(7);
            wp=state(8);
            new_state= [
                x+u*xp;
                y+u*yp;
                z+u*zp;
                w+u*wp;
                xp;
                yp;
                zp;
                wp
                ];
        end
        
        function X = F(x,u)
            x1=x(1);
            x2=x(2);
            x3=x(3);
            x4=x(4);
            x5=x(5);
            x6=x(6);
            x7=x(7);
            x8=x(8);
            
            X=[ [1, 0, 0, 0, u, 0, 0, 0];
                [0, 1, 0, 0, 0, u, 0, 0];
                [0, 0, 1, 0, 0, 0, u, 0];
                [0, 0, 0, 1, 0, 0, 0, u];
                [0, 0, 0, 0, 1, 0, 0, 0];
                [0, 0, 0, 0, 0, 1, 0, 0];
                [0, 0, 0, 0, 0, 0, 1, 0];
                [0, 0, 0, 0, 0, 0, 0, 1]];
        end
        
        function y = h(x)
            y=x(1:4);
        end
        
        function Y = H(x)
            Y=[eye(4,4),zeros(4,4)];
        end
    end
end

