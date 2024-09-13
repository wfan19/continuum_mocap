project_dir_name = "apr_2022/spatial_3muscle_2in_5_6";
%% Parameterize tag locations
% ids of tags along the arm (and the order that we will store them in)
experiment_tags = [30 31 32 33 34 35 36]; % Tags 6 to 9 
base_tag_id = 30;

% the muscles that these muscles are attached to (in the same order) 
tag_muscle_ids = [3 3 3 3 1 1 1]; % List of muscle ids that the tags are attached to, in order

% Define t_tags: Percentage of tag position along total length of arm
% Tag in the middle of arm: t = 0.5
muscle_length_cm = 18 * 2.54;
posns_tags_left = [12.5, 25.5, 38.5]; % centimeters
posns_tags_right = [0, posns_tags_left];
t_tags = ([posns_tags_left, posns_tags_right]) / muscle_length_cm;

% Pressures corresponding to each experiment bag file in order
experiment_pressures = [0 10 15 20 25 30 35 40 45 50 55 60];

%% Parameterize the arm
rho = 1 * 0.0254; % M, Radius
muscle_angle = 0;

%% Construct pose matrices
% Transformations from world frame to each muscle
g_default_muscle = SE3.hat(eul2rotm([0, pi/2, 0], 'xyz'), [0 0 0]); % Frame corresponding to a muscle at the origin, pointing down

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

arm_obj = Arm3D(g_o, g_muscles, contraction_fit(0), 'plot_unstrained', false);
arm_obj.n_spacers = 8;
tags = Tag(experiment_tags, repmat({eye(4)}, size(experiment_tags)), tag_muscle_ids, t_tags);
base_tag = tags([tags.id] == base_tag_id);

g_tag_offset = SE3.hat(eul2rotm([0, 0, 0], 'zyx'), [0; -0.461; 1.675] * 0.0254);
g_global_offset = SE3.hat(eul2rotm([0, deg2rad(0), 0], 'zyx'), zeros(3, 1));

dataset_params = DatasetParams("Spatial 3 muscle", SE3, arm_obj, base_tag, tags, g_tag_offset);
dataset_params.f_parse_bag = @(bag_name) DatasetParams.parse_bag_generic(bag_name, 3, 1);
dataset_params.g_global_offset = g_global_offset;
%% 
dataset_obj = Dataset(project_dir_name, dataset_params);