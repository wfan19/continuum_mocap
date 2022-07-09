function plot_experiment(g_tags_meas, g_tags_fit, v_l, model_arm, fit_arm, h_o_model, h_o_fit, options)

arguments
    g_tags_meas
    g_tags_fit
    v_l
    model_arm
    fit_arm
    h_o_model
    h_o_fit
    options.line_options_muscles = struct("LineWidth", 3);
end

%% Initialize Plotting
% Create figure
ax = gca;
grid on
axis equal

% Set plotting parameters

f_arm_initialize_plotting = @(arm) arm.initialize_plotting(ax, ...
    "line_options_muscles", options.line_options_muscles, "resolution", 40);

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

plot_options.MeshColor = "green";
plotTransformsSE_n(g_tags_meas, ax, plot_options);
plot_options.MeshColor = "red";
plotTransformsSE_n(g_tags_fit, ax, plot_options);

end

