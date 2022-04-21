bags = Dataset.get_dataset_bags("feb_2022/planar_arm_2in");
test_bag = bags(6);
test_bag_path = fullfile(test_bag.folder, test_bag.name);
fprintf("Testing with bag %s\nIn %s\n", test_bag.name, test_bag.folder)
tag_ids = [6 7 8 9];

g_tags = repmat({eye(3)}, 1, 4);

% the muscles that these muscles are attached to (in the same order) 
muscle_ids = [1 1 1 1]; % List of muscle ids that the tags are attached to, in order

% Define t_tags: Percentage of tag position along total length of arm
% Tag in the middle of arm: t = 0.5
muscle_length_cm = 18 * 2.54;
muscle_spacing_cm = 13;
base_offset_cm = 2;
t_tags = ([0, 1, 2, 3] * muscle_spacing_cm + base_offset_cm) / muscle_length_cm;

tags = Tag(tag_ids, {}, muscle_ids, t_tags, "group", SE2());

% Create arm object
g_1 = SE2.hat([0, -base_offset_cm / 100, pi/2]);

rho = 1 * 0.0254;
g_o_1 = SE2.hat([0, rho, 0]); % Muscle 1
g_o_2 = SE2.hat([0, -rho, 0]); % Muscle 2
g_o = g_1 * inv(g_o_1); % Central muscle
g_o_muscles = {g_1; g_o * g_o_2};
l_0 = contraction_fit_poly(0);
arm_obj = Arm2D(g_o, g_o_muscles, l_0, 'plot_unstrained', false);

g_tag_offset = SE2.hat([0 0 -pi/2]);

dataset_params = DatasetParams("planar_2muscle", SE2, arm_obj, tags(1), tags, g_tag_offset);

% Create Measurement object
obj_measurement = Measurement(test_bag_path, [0, 15], dataset_params);
fig = figure();
obj_measurement.plot_measurement(axes);
title("Model (Dashed) vs Measured (Solid)");