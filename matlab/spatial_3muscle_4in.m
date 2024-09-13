project_dir_name = "apr_2022/spatial_3muscle_4in_7_11_level";
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
rho = 2 * 0.0254; % M, Radius
muscle_angle = 0;

%% Construct pose matrices
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

mat_K = diag([1, 5, 5, 0, 0.00015, 0.00015]);

arm_obj = Arm3D(g_o, g_muscles, contraction_fit(0), "mat_K", mat_K, 'plot_unstrained', false);
arm_obj.n_spacers = 8;
tags = Tag(experiment_tags, repmat({eye(4)}, size(experiment_tags)), tag_muscle_ids, t_tags);
base_tag = tags([tags.id] == base_tag_id);

g_tag_offset = SE3.hat(eul2rotm([0, 0, 0], 'zyx'), [0; -0.461; 1.675] * 0.0254);
g_global_offset = SE3.hat(eul2rotm([0, deg2rad(0), 0], 'zyx'), zeros(3, 1));

dataset_params = DatasetParams("Spatial 3 muscle", SE3, arm_obj, base_tag, tags, g_tag_offset);
dataset_params.f_parse_bag = @(bag_name) DatasetParams.parse_bag_generic(bag_name, 3, 3);
dataset_params.f_contraction_model = @contraction_fit_summer;
dataset_params.g_global_offset = g_global_offset;
%% 
dataset_obj = Dataset(project_dir_name, dataset_params);