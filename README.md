# Overview
Found the paper "Modeling The Motion Of A Ball Bouncing Within A Ball" by T. Morrison (https://web.math.pmf.unizg.hr/nastava/nmf/Lopta.pdf) and decided to try implementing it myself. All code was written in Matlab. Feel free to modify or add to any of the files. It's fun to change initial conditions and see how the system responds

# Files
- `ball_in_free_outer_ball.m`: Simulates the entire interaction for `equations.run_time` seconds and plots the final trajectory of the outer ball as a line, with impact points highlighted by dots
- `ball_in_stationary_outer_ball.m`: Simulates the entire interaction of a small ball bouncing around inside a fixed outer ball for `equations.run_time` seconds and plots the trajectory of the outre ball
- `live_plot.m`: Simulates the smaller ball bouncing around inside of a larger ball and plots the position of both balls in real(ish) time. 
- `live_plot_ball_in_ball_in_box.m`: Same as `live_plot.m` but the entire system is confined to a 10x10 box
- `equations.m`: Holds all constants and functions for solving the ODEs that govern this system and reflecting the balls when impacts occur

# Use
If you want to run any of these files, start by setting the initial conditions in the file that you want to run. Initial position and velocity for the balls is at the top of each file. Keep in mind that the outer ball is assumed to be radius 1, and you are responsible for ensuring that the initial conditions have the inner ball within the outer ball, and that both balls are within the box (if applicable). Set `run_time`, `gravity`, and `e` (the coefficient of restitution) in `equations.m` if you want to change any of those. Then hit run and check out the plots!

# Assumptions and Future Improvements
Similar to Morrison, I assumed that all impacts reflected only the velocity normal to the impact, and that all tangent velocities remained the same. I also assumed that there was no friction or rolling between the two balls, or between the ball and the ground. In the future, it would be interesting to add more detail and try to account for these factors in the model.
