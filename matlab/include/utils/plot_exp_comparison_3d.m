function [h_o_model, h_o_fit] = plot_exp_comparison_3d(...
    bag, psi, experiment_tags, tag_muscle_ids, t_tags, base_tag_id, model_arm, muscle_pressurized)

if ~exist('muscle_pressurized', 'var')
    muscle_pressurized = 1;
end

%% Define Experiment Parameters
filename = strsplit(bag.FilePath, '/');
filename = filename{end};
fprintf("Testing with bag: %s\n", filename);

%% Extract and Plot Transformations
% Retrieve transformations
g_tags_meas = cell(size(experiment_tags));
for i = 1 : length(experiment_tags)
    id_tag = experiment_tags(i);
    
    % Make sure that the tag we're looking for has an existing
    % transformation in the TF tree
    if ~any(bag.AvailableFrames == sprintf("tag_%d", id_tag))
        error("Tag with id %d not found in list of available frames", id_tag)
    end
    
    tform_i = getTransform(bag, "tag_" + base_tag_id, "tag_" + id_tag);
    R = quat2rotm(tform_i.Transform.Rotation.readQuaternion());
    % Apply extra 90 deg ccw rotation to correct for misplaced tags
    R = R * eul2rotm([pi/2, 0, 0], 'zyx');
    
    T_obj = tform_i.Transform.Translation;
    T = [T_obj.X; T_obj.Y; T_obj.Z];
    g_tags_meas{i} = SE3(R, T);
end

% First: I accidentally left the tags upside down - let's flip them around
%g_tags_meas = cellfun(@(g_tag) g_tag * SE3(eul2rotm([0, 0, pi], 'xyz'), [0;0;0]), g_tags_meas, ...
%    'UniformOutput', false);

% Base muscle pose: pointing down, located at 0,0,0:
% The neutral pose is where the main contracting muscle will be. This is
% our "origin"
g_default_muscle_original = SE3(eul2rotm([0, pi/2, 0], 'xyz'), [0 0 0]);
g_default_muscle = SE3(eul2rotm([0, pi/2 + deg2rad(4), 0], 'xyz'), [0 0 0]); % Frame corresponding to a muscle at the origin, pointing down

% The default tag - the tag at the base of the default muscle - will
% actually be offset from the default muscle pose by a
% right-multiplication, corresponding to being offset in the tag-body Z
% direction.
g_tag_offset_right = SE3(eye(3), [0; 0; 0.788 * 0.0254]);
offsets = repelem({g_tag_offset_right}, length(experiment_tags));
g_default_tag = g_default_muscle * g_tag_offset_right;

% Transform all tag measurements so that the base muscle is at 0,0,0, and
% the base tag is offset by a right-transformation from that base muscle
% position. These offsets and poses are defined above.
g_base_tag = g_tags_meas{experiment_tags == base_tag_id};
g_tag_shift = g_default_tag * inv(g_base_tag);

% Apply the offset to each pose in the cell array
g_tags_meas = cellfun(@(g_tag) g_tag_shift * g_tag, g_tags_meas, ...
    'UniformOutput', false);

%% Create Arm Objects
fit_arm = copy(model_arm);
model_arm = copy(model_arm);

% Initialzie model arm:
v_pressures = zeros(length(model_arm.muscles), 1);
v_pressures(muscle_pressurized) = psi;
v_l = contraction_fit(v_pressures);

h_o_model = model_arm.mat_N * v_l;

%% Fit curvature
warning('off', 'MATLAB:logm:nonPosRealEig') % Surpress matrix log warning
% TODO: Implement analytic SE3 logm?
[h_o_fit, g_tags_fit] = ...
    curve_fit_3d(model_arm, g_tags_meas, t_tags, tag_muscle_ids, ...
        init_val=h_o_model * 1.5, ... % Multiply by 1.5 so we don't just fit to the model solution
        offset=offsets);

%% Initialize Plotting
% Create figure
ax = gca;
grid on
axis equal

% Set plotting parameters
line_options_muscles = struct("LineWidth", 3);

f_arm_initialize_plotting = @(arm) arm.initialize_plotting(ax, "line_options_muscles", line_options_muscles);

f_arm_initialize_plotting(model_arm);
f_arm_initialize_plotting(fit_arm);

% Darken plotting colors in model arm
for i = 1 : length(model_arm.muscles)
    color_current = model_arm.muscles(i).color;
    model_arm.muscles(i).color = hsv2rgb(rgb2hsv(color_current) .* [1, 1, 0.75]);
    model_arm.muscles(i).lh.LineStyle = ":";
end

% Bring solid line (Fitted muscle curves) to the back
for i = 1 : length(fit_arm.muscles)
    uistack(fit_arm.muscles(i).lh, "bottom");
end

%% Final plotting
model_arm.update_arm(v_l, h_o_model);
%fit_arm.update_arm(v_l, h_o_fit .* [1 0 0 1 1 1]'); % Ignore shearing
fit_arm.update_arm(v_l, h_o_fit);


plot_options = struct();
plot_options.framesize = 0.03;
plot_options.MeshFilePath = "tf_frame.stl";
%{
plot_options.MeshColor = "#089000";

plotTransformsSE_n(g_tags_meas, ax, plot_options);
%}

plotTransformsSE_n(g_tags_meas, ax, plot_options);
plot_options.MeshColor = "green";
plotTransformsSE_n(g_tags_fit, ax, plot_options);

view([20.1549 18.2776]);
end