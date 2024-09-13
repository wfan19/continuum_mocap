project_dir_name = "apr_2022/spatial_3muscle_2in_7_7_level";
color_blue = [100 143 255] / 255;
color_red = [220 38 127] / 255;
color_yellow = [255 176 0] / 255;
color_green = [0, 191, 159] / 255;

%% Parameterize tag locations
% ids of tags along the arm (and the order that we will store them in)
experiment_tags = [10 11 30 31]; % Tags 6 to 9 
base_tag_id = 10;

% the muscles that these muscles are attached to (in the same order) 
tag_muscle_ids = [1 1 1 1]; % List of muscle ids that the tags are attached to, in order

% Define t_tags: Percentage of tag position along total length of arm
% Tag in the middle of arm: t = 0.5
muscle_length_cm = contraction_fit_summer(0) * 100;
posns_tags_left = [13.5, 26.5, 39.5]; % centimeters
posns_tags_right = [0, posns_tags_left];
t_tags = ([0, posns_tags_left]) / muscle_length_cm;

% Pressures corresponding to each experiment bag file in order
experiment_pressures = [0 10 15 20 25 30 35 40 45 50 55 60];

%% Parameterize the arm
rho = 1 * 0.0254; % M, Radius
muscle_angle = 0;

% Transformations from world frame to each muscle
g_default_muscle = SE3.hat(eul2rotm([pi/6, 0, 0], 'xyz'), [0 0 0]); % Frame corresponding to a muscle at the origin, pointing down

% Posese of muscle in world frame
% Muscles are arranged in equilateral triangle with radius rho
g_muscles = Arm3D.generate_g_muscles(rho, g_default_muscle, 3, muscle_angle);

% Set position of first muscle to actually be at the origin
% We want to shift *everything* uniformly regardless of its pose, therefore
% it'll be a left-action:
% g_shift * g_1 = g_neutral
% Therefore g_shift = g_neutral * inv(g_1)
% Additionally, we rotate by a global 180deg so that all 3 muscles are
% aligned correctly
g_muscle_shift = g_default_muscle * inv(g_muscles{1});

% Then left-apply g_shift to all transformations
g_o = g_muscle_shift * g_default_muscle;
g_muscles = cellfun(@(g_muscle) g_muscle_shift * g_muscle, g_muscles, 'uniformoutput', false);

mat_K = diag([1, 100, 100, 0, 0.000128, 0.000128]);

arm_obj = Arm3D(g_o, g_muscles, contraction_fit(0), "mat_K", mat_K, 'plot_unstrained', false);
arm_obj.n_spacers = 8;
arm_obj.muscles(1).color = color_red;
arm_obj.muscles(2).color = color_green;
arm_obj.muscles(3).color = color_blue;

tags = Tag(experiment_tags, repmat({eye(4)}, size(experiment_tags)), tag_muscle_ids, t_tags);
base_tag = tags([tags.id] == base_tag_id);

g_tag_offset = SE3.hat(eul2rotm([0, 0, 0], 'zyx'), [0; -0.461; 1.675] * 0.0254);
g_global_offset = SE3.hat(eul2rotm([0, deg2rad(0), 0], 'zyx'), zeros(3, 1));

dataset_params = DatasetParams("Spatial 3 muscle", SE3, arm_obj, base_tag, tags, g_tag_offset);
dataset_params.f_parse_bag = @(bag_name) DatasetParams.parse_bag_generic(bag_name, 3, 3);
dataset_params.f_contraction_model = @contraction_fit_summer;
dataset_params.g_global_offset = g_global_offset;

dataset_2in = Dataset(project_dir_name, dataset_params);

%%
rho = 2 * 0.0254; % M, Radius
muscle_angle = 0;

% Transformations from world frame to each muscle
g_default_muscle = SE3.hat(eul2rotm([pi/6, 0, 0], 'xyz'), [0 0 0]); % Frame corresponding to a muscle at the origin, pointing down

% Posese of muscle in world frame
% Muscles are arranged in equilateral triangle with radius rho
g_muscles = Arm3D.generate_g_muscles(rho, g_default_muscle, 3, muscle_angle);

% Set position of first muscle to actually be at the origin
% We want to shift *everything* uniformly regardless of its pose, therefore
% it'll be a left-action:
% g_shift * g_1 = g_neutral
% Therefore g_shift = g_neutral * inv(g_1)
% Additionally, we rotate by a global 180deg so that all 3 muscles are
% aligned correctly
g_muscle_shift = g_default_muscle * inv(g_muscles{1});

% Then left-apply g_shift to all transformations
g_o = g_muscle_shift * g_default_muscle;
g_muscles = cellfun(@(g_muscle) g_muscle_shift * g_muscle, g_muscles, 'uniformoutput', false);

mat_K = diag([1, 100, 100, 0, 0.000128, 0.000128]);

arm_obj = Arm3D(g_o, g_muscles, contraction_fit(0), "mat_K", mat_K, 'plot_unstrained', false);
arm_obj.n_spacers = 8;

project_dir_name = "apr_2022/spatial_3muscle_4in_7_11_level";
rho = 2 * 0.0254; % M, Radius
muscle_angle = 0;

arm_obj = Arm3D(g_o, g_muscles, contraction_fit(0), "mat_K", mat_K, 'plot_unstrained', false);
arm_obj.n_spacers = 8;
arm_obj.muscles(1).color = color_red;
arm_obj.muscles(2).color = color_green;
arm_obj.muscles(3).color = color_blue;

dataset_params_4in = DatasetParams("Spatial 3 muscle", SE3, arm_obj, base_tag, tags, g_tag_offset);
dataset_params_4in.f_parse_bag = @(bag_name) DatasetParams.parse_bag_generic(bag_name, 3, 3);
dataset_params_4in.f_contraction_model = @contraction_fit_summer;
dataset_params_4in.g_global_offset = g_global_offset;
dataset_4in = Dataset(project_dir_name, dataset_params_4in);

%% 2inch spatial arm scene
fig_scene = figure("Position", [500, 500, 400, 600]);
ax = axes(fig_scene, "FontSize", 22, "FontName", "Times");
dataset_2in.measurements(end).plot_measurement(ax, false);

for j = 1:length(ax.Children)
    line_j = ax.Children(j);
    if line_j.LineStyle == "-"
        line_j.LineWidth = 3;
    else
        line_j.LineWidth = 2.5;
    end
end

title("")
xlabel(ax, "");
ylabel(ax, "");
zlabel(ax, "");
ax.XTick = [ax.XTick(1), ax.XTick(end)];
ax.YTick = [ax.YTick(1), ax.YTick(end)];
ax.ZTick = [];
view([-90, 75])

xlim(ax, [-0.15, 0.38])
ylim(ax, [-0.05, 0.30])

xtick_posns = 0.35;
ytick_posns = [-0.0, 0.3];

xticks(xtick_posns);
yticks(ytick_posns);
ax.XTickLabel = string(num2str(xtick_posns' - ax.XLim(1))) + " (m)";
ax.YTickLabel = string(num2str(abs(ytick_posns' - ax.YLim(2))));
ax.YTickLabel{1} = ax.YTickLabel{1} + " (m)";
grid off
view(-90.1, 75)

ax.Units = "pixels";
ax.Position = [37.003662109375,22.697784423828125,325.9617450921854,568.2533264160156];
ax.InnerPosition = [53,67,307.9654072015604,489];
ax.OuterPosition = [1,1,400,600];

xtickangle(ax, 90);
set(gcf, "Renderer", "Painters")

%% Create final figure with all arms consolidated together

fig_combined = figure("position", [500, 500, 560, 350]);
ax = axes(fig_combined);

color_2in = [100 143 255] / 255;
color_3in = [220 38 127] / 255;
color_4in = [255 176 0] / 255;
dataset_2in.plot_dataset_curvature(ax, "color", color_2in, "linespec", "o", "metric", true);
dataset_4in.plot_dataset_curvature(ax, "color", color_4in, "linespec", "diamond", "metric", true);

text_posns = [
  216    1.6;
  305    1.15;   
];

arm_sizes = [2, 4];
for i = 1:length(arm_sizes)
    text(ax, text_posns(i, 1), text_posns(i, 2), sprintf("%.2fmm", arm_sizes(i) * 25.4), "FontSize", 12, "FontName", "DejaVu Sans");
end

ylim([0, 1.8])

%title("Spatial Arm Model Accuracies")
ax.FontSize = 14;
ax.FontName = "DejaVu Sans";
title("")

%% Create tip position error plot but with scatters and per dataset
arm_datasets = [dataset_2in, dataset_4in];
fig = figure();
set(fig, "position", [0, 0, 1600, 350])
for i = 1 : length(arm_datasets)
    subplot(1, length(arm_datasets), i);
    arm_datasets(i).plot_tip_accuracy(gca, "color", color_2in, "metric", true, "median", false);
end

%%
fig_combined = figure("position", [500, 500, 560, 350]);
ax = axes(fig_combined);
hold on

color_2in = [100 143 255] / 255;
color_4in = [255 176 0] / 255;
dataset_2in.plot_tip_accuracy(ax, "color", color_2in, "linespec", "o:", "metric", true);
dataset_4in.plot_tip_accuracy(ax, "color", color_4in, "linespec", "diamond:", "metric", true);

legend(ax, ["50.8mm" "101.6mm"],'location', 'southeast')
title("Spatial Arm Tip Position Error")
ax.FontSize = 14;
ax.FontName = "Times";
title("")
ylim([0, 12])