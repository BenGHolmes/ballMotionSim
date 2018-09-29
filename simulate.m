% Inner ball initial conditions
xs0 = -1;
Vxs0 = 5;
ys0 = 0;
Vys0 = 0;
ms = 2;

% Outer ball initial conditions
xl0 = 0;
Vxl0 = 0;
yl0 = 0;
Vyl0 = 0;
ml = 5;

run_time = 10;  % Length of sim

% Data points
xs = zeros(1,run_time/equations.dt);
ys = zeros(1,run_time/equations.dt);
xl = zeros(1,run_time/equations.dt);
yl = zeros(1,run_time/equations.dt);
contact_x = zeros(1,run_time/equations.dt);
contact_y = zeros(1,run_time/equations.dt);

xs(1) = xs0;
ys(1) = ys0;
xl(1) = xl0;
yl(1) = yl0;

theta = 0:0.01:2*pi;
circle_x = cos(theta);
circle_y = sin(theta);

for i = 2:numel(ys)
    [ys0, Vys0] = equations.get_y(ys0, Vys0);
    [xs0, Vxs0] = equations.get_x(xs0, Vxs0);
    [xl0, Vxl0] = equations.get_x(xl0, Vxl0);
    [yl0, Vyl0] = equations.get_y(yl0, Vyl0);
    
    xs(i) = xs(i-1) + (Vxs0*equations.dt);
    ys(i) = ys(i-1) + (Vys0*equations.dt);
    xl(i) = xl(i-1) + (Vxl0*equations.dt);
    yl(i) = yl(i-1) + (Vyl0*equations.dt);
    
    if xs(i)^2 + ys(i)^2 > 1
        [Vxs0, Vys0, Vxl0, Vyl0] = equations.rebound(Vxs0, Vys0, Vxl0, Vyl0, xs(i), ys(i), ms, ml);
        xs(i) = xs(i-1) + (Vxs0*equations.dt);
        ys(i) = ys(i-1) + (Vys0*equations.dt);
        xl(i) = xl(i-1) + (Vxl0*equations.dt);
        yl(i) = yl(i-1) + (Vyl0*equations.dt);
    end
    
    
end

hold on
plot(xs,ys)
plot(circle_x, circle_y)
ylim([-1.5 1.5]);
xlim([-1.5 1.5]);
pbaspect([1 1 1]);
hold off

plot(xl,yl)
