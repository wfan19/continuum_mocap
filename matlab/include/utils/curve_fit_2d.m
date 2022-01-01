function h_optim = curve_fit_2d(planar_arm_obj, g_tags_meas, t_tags, muscle_ids, options)

arguments
    planar_arm_obj
    g_tags_meas
    t_tags
    muscle_ids
    options.init_val (3, 1) = [1; 0; -0.5];
    options.offsets cell = repelem({eye(3)}, length(g_tags_meas));
end

tag_num = length(t_tags);

f_error_vs = cell(1, tag_num);
f_costs = cell(1, tag_num + 1); % Plus one for the shear-regulating term

for i = 1 : tag_num
    % Lie algebra distance between curve fit and measured position
    % Right subtraction
    g_o_Xi = inv(planar_arm_obj.muscle_o.g_0) * planar_arm_obj.muscles(muscle_ids(i)).g_0;
    g_o_Xi = g_o_Xi * options.offsets{i}; % Apply any offsets that may be needed

    f_error_vector = @(h_o) vee_se2(logm(inv(g_tags_meas{i}) * planar_arm_obj.muscle_o.g_0 * expm_se2(h_o * t_tags(i)) * g_o_Xi));
    
    % Define quadratic cost
    K = diag([1, 1, 0]);
    f_error_vs{i} = f_error_vector;
    f_costs{i} = @(h_o) f_error_vector(h_o)' * K * f_error_vector(h_o);
end

% Add a an additional shear-regulating cost to try to minimize shearing
f_costs{end} = @(h_o) h_o' * diag([0, 1, 0]) * h_o;

f_cost_total = @(h_o) sum(cellfun(@(func) func(h_o(:)), f_costs));

h_optim = fminsearch(f_cost_total, options.init_val);

end

