classdef equations
    properties(Constant)
        g = 9.81;  % m/s^2
        mu = 0.5;  % drag coefficient of sphere
        dt = 0.0005;
        e = 0.6;
    end
    
    methods(Static)
        
        function [y,v] = get_y(y0, v0)
            dt = equations.dt;
            mu = equations.mu;
            g = equations.g;
            
            % y'' = -g + (mu * y')
            
            % break into 2 first order
            % s1: y' = v
            % s2: v' = -g - (mu * v)
            
            s1 = @(v) v;
            s2 = @(v) -g - mu * v;
            
            k1y = s1(v0);
            k1v = s2(v0);
            
            k2y = s1(v0 + dt* k1v/2);
            k2v = s2(v0 + dt * k1v/2);
            
            k3y = s1(v0 + dt * k2v/2);
            k3v = s2(v0 + dt * k2v/2);
            
            k4y = s1(v0 + dt * k3v);
            k4v = s2(v0 + dt * k3v);
            
            y = y0 + dt/6 * (k1y + 2*k2y + 2*k3y + k4y);
            v = v0 + dt/6 * (k1v + 2*k2v + 2*k3v + k4v);
        end
        
        function [x,v] = get_x(x0, v0)
            dt = equations.dt;
            mu = equations.mu;
            
            % x'' = -(mu * x')
            
            % break into 2 first order
            % s1: x' = v
            % s2: v' = -(mu * v)
            
            s1 = @(v) v;
            s2 = @(v) -mu * v;
            
            k1y = s1(v0);
            k1v = s2(v0);
            
            k2y = s1(v0 + dt* k1v/2);
            k2v = s2(v0 + dt * k1v/2);
            
            k3y = s1(v0 + dt * k2v/2);
            k3v = s2(v0 + dt * k2v/2);
            
            k4y = s1(v0 + dt * k3v);
            k4v = s2(v0 + dt * k3v);
            
            x = x0 + dt/6 * (k1y + 2*k2y + 2*k3y + k4y);
            v = v0 + dt/6 * (k1v + 2*k2v + 2*k3v + k4v);
        end
        
        function [Vxs, Vys, Vxl, Vyl] = rebound(Vxs0, Vys0, Vxl0, Vyl0, x, y, ms, ml)
            e = equations.e;

            r = [x, y];
            v = [Vxs0, Vys0];
            
            vr = (dot(v,r) / norm(r)^2) * r;
            vt = e * (v - vr);
            vr = -e * vr;
            vrl = ((ms*sqrt(Vxs0^2 + Vys0^2)+ ml*sqrt(Vxl0^2 + Vyl0^2) - ms*norm(vr + vt)^2) / ml) * [Vxl0 Vyl0];  % Change in velocity of large ball
            Vxl = vrl(1);
            Vyl = vrl(2);
            Vxs = vt(1) + vr(1);
            Vys = vt(2) + vr(2);
        end
    end
end