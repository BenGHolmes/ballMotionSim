classdef equations
    properties(Constant)
        g = 9.81;  % m/s^2
        mu = 0.5;  % drag coefficient of sphere
        dt = 0.005;  % time step
        e = 0.95;  % coefficient of restitution
        run_time = 10;  % s
        m_l = 10;  % kg
        m_s = 1;  % kg
    end
    
    methods(Static)
        
        % Get new position and velocity in y direction using fourth order 
        % Runge-Kutta
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
        
        % Get new position and velocity in x direction using fourth order 
        % Runge-Kutta 
        function [x,v] = get_x(x0, v0)
            dt = equations.dt;
            mu = equations.mu;
            
            % x'' = -(mu * x')
            
            % break into 2 first order
            % s1: x' = v
            % s2: v' = -(mu * v)
            
            s1 = @(v) v;
            s2 = @(v) -mu * v;
            
            k1x = s1(v0);
            k1v = s2(v0);
            
            k2x = s1(v0 + dt * k1v/2);
            k2v = s2(v0 + dt * k1v/2);
            
            k3x = s1(v0 + dt * k2v/2);
            k3v = s2(v0 + dt * k2v/2);
            
            k4x = s1(v0 + dt * k3v);
            k4v = s2(v0 + dt * k3v);
            
            x = x0 + dt/6 * (k1x + 2*k2x + 2*k3x + k4x);
            v = v0 + dt/6 * (k1v + 2*k2v + 2*k3v + k4v);
        end
        
        % Calculate velocity in the x and y directions after rebounding
        % off the inside of the ball at a specific point (x,y) with some 
        % initial velocity (Vx0, Vy0). It is assumed that only the radial
        % velocity is reflected while the tangential velocity keeps its
        % direction. Both velocities decrease by a factor of 1 over their
        % appropriate coefficient of restitution (e for Vr and e_tan
        % for Vt)
        function [Vx, Vy] = rebound_fixed_outer(Vx0, Vy0, x, y)
            e = equations.e;

            r = [x, y];
            v = [Vx0, Vy0];
            
            vr = (dot(v,r) / norm(r)^2) * r;
            vt =  (v - vr);
            vr = -e * vr;
            Vx = vt(1) + vr(1);
            Vy = vt(2) + vr(2);
        end
        
        % Calculate velocity in the x and y directions after bouncing off
        % the ground or roof. Vy is reflected and scaled by 1/e while Vx keeps
        % its direction and magnitude
        function [Vx, Vy] = rebound_outer_y(Vx0, Vy0)
            e = equations.e;
            
            Vx = Vx0;
            Vy = -e * Vy0;
        end
        
        function [Vx, Vy] = rebound_outer_x(Vx0, Vy0)
            e = equations.e;
            
            Vx = -e * Vx0;
            Vy = Vy0;
        end
        
        function [Vxl, Vyl, Vxs, Vys] = rebound_free_outer(Vl, Vs, L, S)
            e = equations.e;
            m_l = equations.m_l;
            m_s = equations.m_s;
            
            % Relative r and v vectors
            r = S-L;
            v = Vs-Vl;
            
            % Tangent and normal components of relative velocity
            vrs = (dot(v,r) / norm(r)^2) * r;
            vts = (v - vrs);
            
            % Find final radial velocities of inner and outer ball given e 
            vrs_f = (m_s*vrs + m_l * e * -vrs) / (m_l + m_s);
            vrl_f = (m_s*vrs + m_s * e * vrs) / (m_l + m_s);
            
            Vxl = Vl(1) + vrl_f(1);
            Vyl = Vl(2) + vrl_f(2);
            Vxs = Vl(1) + vrs_f(1) + vts(1);
            Vys = Vl(2) + vrs_f(2) + vts(2);
            
        end
            
    end
end