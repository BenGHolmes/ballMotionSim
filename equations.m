classdef equations
    properties(Constant)
        g = 9.81;  % m/s^2
        mu = 0.5;  % drag coefficient of sphere
        dt = 0.0005;  % time step
        e = 0.7;  % coefficient of restitution
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
        % velocity is reflected while the tangential velocity remains the 
        % same. Both velocities are reduced after impact by a factor of 1/e
        function [Vx, Vy] = rebound_fixed_outer(Vx0, Vy0, x, y)
            e = equations.e;

            r = [x, y];
            v = [Vx0, Vy0];
            
            vr = (dot(v,r) / norm(r)^2) * r;
            vt = e * (v - vr);
            vr = -e * vr;
            Vx = vt(1) + vr(1);
            Vy = vt(2) + vr(2);
        end
    end
end