function [gh_tform, t_tforms, quat_tforms] = plotTransformsSE_n(g_tforms, ax, plot_options)
%PLOTTRANSFORMSSE3 Applys plotTransform() TF frame plotting to a cell-array
%of SE3 pose matrices
% Args:
% - g_tforms: Cell array of SE3 or SE2 pose matrices

arguments
    g_tforms cell
    ax = gca()
    plot_options = struct()
end

if length(g_tforms{1}) == 3
    % SE2
    f_cellfun_t = @(mat) [mat(1:2, 3); 0];
    f_cellfun_quat = @(mat) transpose(rotm2quat(blkdiag(mat(1:2, 1:2), 1)));
elseif length(g_tforms{1}) == 4
    % SE3
    f_cellfun_t = @(mat) mat(1:3, 4);
    f_cellfun_quat = @(mat) transpose(rotm2quat(mat(1:3, 1:3)));
else
    error("Unsupported matrix dimension detected")
end

f_extract_with_cellfun = @(f_cellfun) ...
    cell2mat(cellfun(f_cellfun, g_tforms, 'uniformoutput', false));

% Apply "extraction" functions to the cell array
t_tforms = transpose(f_extract_with_cellfun(f_cellfun_t));
quat_tforms = transpose(f_extract_with_cellfun(f_cellfun_quat));

% Set plotTransforms Parent options to the specified Axis
plot_options.parent = ax;

plotTransforms(t_tforms, quat_tforms, plot_options);

% Newest axis children (corresponding to the plotTransform) is pre-pended
% to Axis.Children so we can grab the Group Handle (gh).
gh_tform = ax.Children(1);

end

