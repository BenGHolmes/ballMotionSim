% Inner ball initial conditions
x0 = 0.5; y0 = 0.4; Vx = 3; Vy = -6;

run_time = 10;  % Length of sim

% Data points
x = zeros(1,run_time/equations.dt);
y = zeros(1,run_time/equations.dt);

% Initial conditions
x(1) = x0; y(1) = y0;

% Plot outer ball
theta = 0:0.01:2*pi;
circle_x = cos(theta);
circle_y = sin(theta);

for i = 2:numel(y)
    % Get new position and velocity from Runge-Kutta methods
    [y(i), Vy] = equations.get_y(y(i-1), Vy);
    [x(i), Vx] = equations.get_x(x(i-1), Vx);
   
    if x(i)^2 + y(i)^2 > 1
        [Vx, Vy] = equations.rebound_fixed_outer(Vx, Vy, x(i), y(i));
        x(i) = x(i-1) + (Vx*equations.dt);
        y(i) = y(i-1) + (Vy*equations.dt);
    end
    
    
end

hold on
plot(x,y)
plot(circle_x, circle_y)
ylim([-1.5 1.5]);
xlim([-1.5 1.5]);
pbaspect([1 1 1]);
hold off