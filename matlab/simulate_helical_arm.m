%% Parameterize the arm
rho = 1 * 0.0254; % M, Radius
muscle_angle = deg2rad(15);
color_blue = [100 143 255] / 255;
color_red = [220 38 127] / 255;
color_yellow = [255 176 0] / 255;
color_green = [0, 191, 159] / 255;

%% Construct pose matrices
% Transformations from world frame to each muscle
g_default_muscle = SE3.hat(eul2rotm([0, pi/2, 0], 'xyz'), [0 0 0]); % Frame corresponding to a muscle at the origin, pointing down

% Posese of muscle in world frame
% Muscles are arranged in equilateral triangle with radius rho
g_muscles = Arm3D.generate_g_muscles(rho, g_default_muscle, 3, muscle_angle);
g_muscle_shift=eye(4);

% Then left-apply g_shift to all transformations
g_o = g_muscle_shift * g_default_muscle;
g_muscles = cellfun(@(g_muscle) g_muscle_shift * g_muscle, g_muscles, 'uniformoutput', false);

mat_K = diag([1, 100, 100, 0, 0.000098, 0.000098]);

f_contraction_fit = @contraction_fit_summer;
arm_obj = Arm3D(g_o, g_muscles, f_contraction_fit(0), "mat_K", mat_K, 'plot_unstrained', false);
arm_obj.n_spacers = 8;
arm_obj.muscles(1).color = color_red;
arm_obj.muscles(2).color = color_green;
arm_obj.muscles(3).color = color_blue;

%% Create Figure
ax = axes(figure());
set(gcf, "Position", [1300, 500, 400, 800])

% Set plotting parameters
line_options_muscles = struct("LineWidth", 3);
line_options_spacers = struct("LineWidth", 2);
arm_obj.initialize_plotting(ax, "line_options_muscles", line_options_muscles, "line_options_spacers", line_options_spacers);
arm_obj.update_arm(f_contraction_fit([60; 0; 0]));

grid on
axis equal
%v = [16.7252  -13.1063];
v = [-7.9210   30.6034];

initial_xlim = ax.XLim;
initial_ylim = ax.YLim;
initial_zlim = [-f_contraction_fit(0) * 1.02 0];

%% Create pressure function
p_0 = 60;   % psi
p_end = 0;  % psi
t_0 = 0;    % seconds
t_end = 20; % seconds
fps = 24;

% Derived values
slope = (p_end - p_0) / (t_end - t_0);
n_frames = fps * (t_end - t_0);
tspan = linspace(t_0, t_end, n_frames);
pressure = slope * tspan + p_0; % y = ax + b

%%
animator = Animator(gcf, length(pressure));

for i = 1 : length(pressure)
    arm_obj.update_arm(f_contraction_fit([pressure(i); 0; 0]));
    xlim(ax, initial_xlim);
    ylim(ax, initial_ylim);
    zlim(ax, initial_zlim);
    set(gcf, "Position", [1300, 500, 400, 800])
    animator.get_frame();
    drawnow
end