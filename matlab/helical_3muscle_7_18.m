dataset_obj = load_helical_3_muscle_7_18;
psi_to_kpa = 6.89476;
color_blue = [100 143 255] / 255;
color_red = [220 38 127] / 255;
color_yellow = [255 176 0] / 255;
color_green = [0, 191, 159] / 255;

%% Single image at 60psi
v = [24 13.1063];
ax = axes(figure("Position", [500, 500, 400, 600]), "FontSize", 22, "FontName", "Times");
dataset_obj.measurements(45).plot_measurement(ax, false)
view(v)
title("")
xlabel("");
ylabel("");
zlabel("");
xlim(ax, [-0.03, 0.1]);
ylim(ax, [-0.05,0.05])
zlim(ax, [-0.4,0]);

xtick_posns = [-0.03, 0.1];
ytick_posns = 0.05;
ztick_posns = -0.0;

xticks(xtick_posns);
yticks(ytick_posns);
zticks(ztick_posns);
ax.XTickLabel = string(num2str(xtick_posns' - ax.XLim(1)));
ax.YTickLabel = string(num2str(ytick_posns' - ax.YLim(1)));
ax.ZTickLabel = string(num2str(ztick_posns' - ax.ZLim(1)));

ax.XTickLabel{2} = ax.XTickLabel{2} + " (m)";
ax.YTickLabel = ax.YTickLabel + " (m)";
ax.ZTickLabel = ax.ZTickLabel + " (m)";

xtickangle(0);
ytickangle(0);
ztickangle(90);
ax.FontSize = 22; 

ax.Units = "pixels";
ax.Position = [-66.55611419677734,60.99044036865257,475.4021835327146,491.0095596313508];
set(gcf, "Renderer", "Painters")

%% 20, 40, 60psi scenes
%i_scenes = [15, 35, 49];


%% ??, 35, 50psi scenes?
%i_scenes = [14, 28, 35, 40, 45, 49];
i_scenes = [14, 28, 45];

%figure()
%set(gcf, "position", [0, 0, 1600, 900]);
%v = [16.7252  -13.1063];
%v = [-7.9210   30.6034];
v = [23.7252  13.1063];
for i = 1:length(i_scenes)
    %subplot(1, length(i_scenes), i);
    %ax = gca;
    ax = axes(figure("Position", [500, 500, 400, 800]), "FontSize", 22, "FontName", "Times");

    disp(i_scenes(i))
    measurement = dataset_obj.measurements(i_scenes(i));
    measurement.plot_measurement(ax, false)
    view(v)
    %title(sprintf("%.f kPa", measurement.v_pressure(1) * psi_to_kpa));
    title("")

    for j = 1:length(ax.Children)
        line_j = ax.Children(j);
        if line_j.LineStyle == "-"
            line_j.LineWidth = 3;
        else
            line_j.LineWidth = 2.5;
        end
    end

    xlabel("");
    ylabel("");
    zlabel("");
    xlim(ax, [-0.03, 0.1]);
    ylim(ax, [-0.05,0.05])
    zlim(ax, [-0.45,0]);
    
    xtick_posns = [-0.03, 0.1];
    ytick_posns = 0.05;
    ztick_posns = -0.0;
    
    xticks(xtick_posns);
    yticks(ytick_posns);
    zticks(ztick_posns);
    ax.XTickLabel = string(num2str(xtick_posns' - ax.XLim(1)));
    ax.YTickLabel = string(num2str(ytick_posns' - ax.YLim(1)));
    ax.ZTickLabel = string(num2str(ztick_posns' - ax.ZLim(1)));
    
    ax.XTickLabel{2} = ax.XTickLabel{2} + " (m)";
    ax.YTickLabel = ax.YTickLabel + " (m)";
    ax.ZTickLabel = ax.ZTickLabel + " (m)";
    xtickangle(ax, 0);
    ytickangle(ax, 0);
    ztickangle(ax, 90);

    ax.OuterPosition = [-0.107802763711126,-0.035663590156198,1.17009866867323,1.049079754601227];
    ax.InnerPosition = [0.040731707317073,0.079735182849937,0.885493961898881,0.855];
    ax.Position = [-0.039268292682927,0.078485182849937,0.885493961898881,0.855];
    %fig.Position = [500, 500, 400, 800];
end

%%
posn_error = zeros(1, length(dataset_obj.tab_measurements.v_pressure));
for i = 1:length(dataset_obj.tab_measurements.v_pressure)
    displacement = SE3.vee(inv(se3.expm(se3.hat(dataset_obj.tab_measurements.h_o_fit(i, :)))) * se3.expm(se3.hat(dataset_obj.tab_measurements.h_o_model(i, :))));
    posn_error(i) = norm(displacement(1:3));
end

posn_error = posn_error / contraction_fit_summer(0) * 100;

outlier_datasets = posn_error > 10 & 1:length(posn_error) > 10;
    
v_pressure = dataset_obj.tab_measurements.v_pressure(~outlier_datasets, 1);
posn_errors = posn_error(~outlier_datasets);

[groups, pressures] = findgroups(v_pressure);
posn_error_med = splitapply(@median, posn_errors(:), groups(:));

ax = axes(figure("position", [500, 500, 560, 350]));
%plot(dataset_obj.tab_measurements.v_pressure(~outlier_datasets, 1), posn_error(~outlier_datasets))
%plot(pressures * psi_to_kpa, posn_error_med, 'b:o', "linewidth", 3, "color", color_blue, "markersize", 13, "markerfacecolor", "w")
plot(pressures * psi_to_kpa, posn_error_med, 'b:o', "linewidth", 3, "color", color_blue, "markerfacecolor", color_blue, "markersize", 10)
xlim([min(pressures*6.89476), max(pressures*6.89476)])
xlabel("Pressure (kpa)")
ylabel("Tip position error")
ytickformat("percentage")
ylim([0, 12])
%title("Helical Arm End Position Error")
grid on

ax.FontSize = 14;
ax.FontName = "Times";

%%

h_o_fit_med = splitapply(@(data) median(data, 1), dataset_obj.tab_measurements.h_o_fit(~outlier_datasets, :), groups);
h_o_model_med = splitapply(@(data) median(data, 1), dataset_obj.tab_measurements.h_o_model(~outlier_datasets, :), groups);

ax = axes(figure());
hold on
plot(h_o_fit_med(:, 4), h_o_fit_med(:, 6), 'bx')
plot(h_o_model_med(:, 4), h_o_model_med(:, 6), 'ro:')

xlabel("X-Curvature")
ylabel("Z-Curvature")
grid on
ax.FontSize = 16;

%%
fig = figure("position", [500, 500, 560, 350]); 
ax = axes(fig);

hold on
plot(pressures*psi_to_kpa, h_o_fit_med(:, 4), 'o', 'linewidth', 3, "markersize", 10, "color", color_blue);
plot(pressures*psi_to_kpa, h_o_model_med(:, 4), ':', 'linewidth', 5, "color", color_blue);

plot(pressures*psi_to_kpa, h_o_fit_med(:, 6), 'square', 'linewidth', 3, "markersize", 10, "color", color_red);
plot(pressures*psi_to_kpa, h_o_model_med(:, 6), ':', 'linewidth', 5, "color", color_red);

xlim([min(pressures*psi_to_kpa), max(pressures*psi_to_kpa)]);
legend(["X curvature", "", "Z curvature", ""], "location", "southeast")
xlabel("Pressure (kPa)")
ylabel("Scaled-Curvature (rad)")

%title("Helical dataset curatures")
grid on

ax.FontSize = 14;
ax.FontName = "DejaVu Sans";
fig.Children(1).Position = [0.141569217329836,0.811143506000189,0.275584793598828,0.141399415698361];

%%
figure()

colororder({'b', 'r'})

hold on
yyaxis left
plot(pressures*psi_to_kpa, h_o_fit_med(:, 4), 'bx', 'linewidth', 3);
plot(pressures*psi_to_kpa, h_o_model_med(:, 4), 'b:', 'linewidth', 3);

yyaxis right
plot(pressures*psi_to_kpa, h_o_fit_med(:, 6), 'rx', 'linewidth', 3);
plot(pressures*psi_to_kpa, h_o_model_med(:, 6), 'r:', 'linewidth', 3);

xlim([min(pressures*psi_to_kpa), max(pressures*psi_to_kpa)]);
legend(["", "X curvature", "", "Z curvature"], "location", "northwest")

title("Helical dataset curatures")

grid on

%% Compute error metrics in terms of winding radius
num_meas = size(dataset_obj.tab_measurements, 1);
winding_axis_fit = zeros(num_meas, 3);
r_o_fit = zeros(num_meas, 3);
pitch_fit = zeros(num_meas, 1);

winding_axis_model = zeros(num_meas, 3);
r_o_model = zeros(num_meas, 3);
pitch_model = zeros(num_meas, 1);

for i = 1 : num_meas
    measurement = dataset_obj.tab_measurements(i, :);
    [winding_axis_fit(i, :), r_o_fit(i, :), pitch_fit(i, :)] = compute_helix(measurement.h_o_fit);
    [winding_axis_model(i, :), r_o_model(i, :), pitch_model(i, :)] = compute_helix(measurement.h_o_model);
end

r_error = (vecnorm(r_o_fit, 2, 2) - vecnorm(r_o_model, 2, 2))./vecnorm(r_o_model, 2, 2);
r_error = r_error(~outlier_datasets);

pitch_error=(pitch_fit - pitch_model)./ pitch_model;
pitch_error = pitch_error(~outlier_datasets);

%%
function [winding_axis_o, r_o, pitch] = compute_helix(h_o_tilde)
    v = h_o_tilde(1:3);     % Linear velocity
    omega = h_o_tilde(4:6); % Angular velocity
    
    % Calculate helix parameters in the base-curve frame
    winding_axis_o = omega / norm(omega) * sign(omega(1));
    r_o = cross(omega, v) / norm(omega)^2; % Radius vector
    
    v_x_prime = omega(1) * v(1) / norm(omega); % Helix linear velocity in winding axis direction
    pitch = v_x_prime / norm(omega); % Pitch = length / turns
end
