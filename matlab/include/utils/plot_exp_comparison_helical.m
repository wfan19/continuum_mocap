function [h_o_model, h_o_fit] = plot_exp_comparison_helical(...
    bag, psi, experiment_obj)

%% Define Experiment Parameters
filename = strsplit(bag.FilePath, '/');
filename = filename{end};
fprintf("Testing with bag: %s\n", filename);

%% Create Arm Objects
fit_arm = copy(experiment_obj.arm_obj);
model_arm = copy(experiment_obj.arm_obj);

% Initialzie model arm:
v_pressures = zeros(length(model_arm.muscles), 1);
v_pressures(experiment_obj.muscle_pressurized) = psi;
v_l = contraction_fit(v_pressures);

h_o_model = model_arm.mat_N * v_l;

%% Extract and Plot Transformations
g_tags_meas = cell(size(experiment_obj.tags));
for i = 1 : length(experiment_obj.tags)
    id_tag = experiment_obj.tags(i);
    
    % Make sure that the tag we're looking for has an existing
    % transformation in the TF tree
    if ~any(bag.AvailableFrames == sprintf("tag_%d", id_tag))
        error("Tag with id %d not found in list of available frames", id_tag)
    end
    
    tform_i = getTransform(bag, "lab", "tag_" + id_tag);
    R = quat2rotm(tform_i.Transform.Rotation.readQuaternion());
    % Apply extra 90 deg ccw rotation to correct for misplaced tags
    R = R * eul2rotm([-pi/2, 0, 0], 'zyx');
    
    T_obj = tform_i.Transform.Translation;
    T = [T_obj.X; T_obj.Y; T_obj.Z];
    g_tags_meas{i} = SE3(R, T);
end

% First: I accidentally left the tags upside down - let's flip them around
g_tags_meas = cellfun(@(g_tag) g_tag * SE3(eul2rotm([0, 0, pi], 'xyz'), [0;0;0]), g_tags_meas, ...
    'UniformOutput', false);

% The default tag - the tag at the base of the default muscle - will
% actually be offset from the default muscle pose by a
% right-multiplication, corresponding to being offset in the tag-body Z
% direction.
g_tag_offset_right = SE3(eye(3), [0; 0; 0.4 * 0.0254]);
%g_tag_offset_right = eye(4);
offsets = repelem({g_tag_offset_right}, length(experiment_obj.tags));
%offsets = repelem({eye(4)}, length(experiment_tags));

g_origin = SE3(eul2rotm([0, pi/2, 0], 'xyz'), [0 0 0]);
g_default_muscle_upright = g_origin * SE3(eul2rotm([-pi/2, 0, 0], 'xyz'), [0, 0, 0]);
%g_default_tag = g_default_muscle_upright * g_tag_offset_right;

g_default_tag = model_arm.muscles(1).g_0 * g_tag_offset_right; % * SE3(eye(3), [0; 0; 0.005]);

% Transform all tag measurements so that the base muscle is at 0,0,0, and
% the base tag is offset by a right-transformation from that base muscle
% position. These offsets and poses are defined above.
g_base_tag = g_tags_meas{experiment_obj.tags == experiment_obj.base_tag_id};
g_tag_shift = g_default_tag * inv(g_base_tag);

% Apply the offset to each pose in the cell array
g_tags_meas = cellfun(@(g_tag) g_tag_shift * g_tag, g_tags_meas, ...
    'UniformOutput', false);

%% Fit curvature
warning('off', 'MATLAB:logm:nonPosRealEig') % Surpress matrix log warning
% TODO: Implement analytic SE3 logm?
[h_o_fit, g_tags_fit] = ...
    curve_fit_3d(model_arm, g_tags_meas, experiment_obj.t_tags, experiment_obj.tag_muscle_ids, ...
        init_val=h_o_model + [0;0;0;0;-0.2;0], ... % Multiply by 1.5 so we don't just fit to the model solution
        offset=offsets);

plot_experiment(g_tags_meas, g_tags_fit, v_l, model_arm, fit_arm, h_o_model, h_o_fit);

view([20.1549 18.2776]);
end