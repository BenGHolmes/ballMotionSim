run_time = equations.run_time;

% Data points
xs = zeros(1,run_time/equations.dt);
ys = zeros(1,run_time/equations.dt);
xl = zeros(1,run_time/equations.dt);
yl = zeros(1,run_time/equations.dt);
contact_x = [];
contact_y = [];

% Initial conditions
xs(1)=2; ys(1)=10; Vxs=5; Vys=0;
xl(1)=2; yl(1)=10; Vxl=0; Vyl=0;

for i=2:numel(xs)
    % Update outer ball position and velocity
    [xl(i), Vxl] = equations.get_x(xl(i-1), Vxl);
    [yl(i), Vyl] = equations.get_y(yl(i-1), Vyl);
    
    % Update inner ball position and velocity
    [xs(i), Vxs] = equations.get_x(xs(i-1), Vxs);
    [ys(i), Vys] = equations.get_y(ys(i-1), Vys);
    
    % Rebound large ball if it goes below the ground
    if yl(i) < 1
        [Vxl, Vyl] = equations.rebound_outer_y(Vxl, Vyl);
        xl(i) = xl(i-1) + Vxl*equations.dt;
        yl(i) = yl(i-1) + Vyl*equations.dt;
    end
    
    if (xs(i)-xl(i))^2 + (ys(i)-yl(i))^2 > 1
        contact_x = [contact_x xl(i)];
        contact_y = [contact_y yl(i)];
        [Vxl, Vyl, Vxs, Vys] = equations.rebound_free_outer([Vxl Vyl], [Vxs Vys], [xl(i) yl(i)], [xs(i) ys(i)]);
        xl(i) = xl(i-1) + Vxl*equations.dt;
        yl(i) = yl(i-1) + Vyl*equations.dt;
        xs(i) = xs(i-1) + Vxs*equations.dt;
        ys(i) = ys(i-1) + Vys*equations.dt;
    end
end

clf;
hold on
plot(xl, yl)
plot(contact_x, contact_y, 'o')
ylim([0 15]);
xlim([0 10]);
hold off

hold on
plot(xs, ys)
plot(contact_x, contact_y, 'o')
hold off



