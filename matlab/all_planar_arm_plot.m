color_blue = [100 143 255] / 255;
color_red = [220 38 127] / 255;
color_yellow = [255 176 0] / 255;

%%
tag_ids = [6 7 8 9 10 11 14];

% the muscles that these muscles are attached to (in the same order) 
muscle_ids = [1 1 1 2 2 2 2]; % List of muscle ids that the tags are attached to, in order

% Define t_tags: Percentage of tag position along total length of arm
% Tag in the middle of arm: t = 0.5
muscle_length_cm = 18 * 2.54;
base_offset_cm = 3;

posns_tags_left = [13, 25.5, 39]; % centimeters
posns_tags_right = [0, posns_tags_left];
t_tags = ([posns_tags_left, posns_tags_right] + base_offset_cm) / muscle_length_cm;

tags = Tag(tag_ids, {}, muscle_ids, t_tags, "group", SE2());
base_tag = tags([tags.id] == 9);

arm_obj_2in = create_arm_obj(2);
arm_obj_2in.muscles(1).color=color_blue;
arm_obj_2in.muscles(2).color=color_red;

g_tag_offset = SE2.hat([0 0 0]);

dataset_params_apr = DatasetParams("2 inch", SE2, arm_obj_2in, base_tag, tags, g_tag_offset);
dataset_params_apr.f_parse_bag = @(bag_name) DatasetParams.parse_bag_generic(bag_name, 2, 1);

arm_2in = Dataset("apr_2022/planar_arm_2in_4_25", dataset_params_apr);
%arm_2in.add_dataset("apr_2022/planar_arm_2in_4_23", dataset_params_apr);

%%
dataset_params_3in = copy(dataset_params_apr);
dataset_params_3in.arm_obj = create_arm_obj(3);
dataset_params_3in.arm_obj.muscles(1).color=color_blue;
dataset_params_3in.arm_obj.muscles(2).color=color_red;

dataset_params_apr.dataset_name = "3 inch";
arm_3in = Dataset("apr_2022/planar_arm_3in_4_30", dataset_params_3in);
%arm_3in.add_dataset("apr_2022/planar_arm_3in_5_2", dataset_params_3in);

%%
dataset_params_4in = copy(dataset_params_3in); % For some reason using dataset_params_apr breaks this.
dataset_params_4in.arm_obj = create_arm_obj(4);
dataset_params_4in.arm_obj.muscles(1).color=color_blue;
dataset_params_4in.arm_obj.muscles(2).color=color_red;

dataset_params_4in.dataset_name = "4 inch";
arm_4in = Dataset("apr_2022/planar_arm_4in_4_30", dataset_params_4in);
arm_4in.add_dataset("apr_2022/planar_arm_4in_4_30_2", dataset_params_4in);

%% 4inch planar arm scene
fig_scene = figure("Position", [500, 500, 400, 600]);
ax = axes(fig_scene, "FontSize", 22);
arm_4in.measurements(end).plot_measurement(ax, false);

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
xlim(ax, [-0.15, 0.15])
ylim(ax, [-0.05, 0.40])

xtick_posns = [-0.15, 0.1];
ytick_posns = 0.35;

xticks(xtick_posns);
yticks(ytick_posns);
ax.XTickLabel = string(num2str(xtick_posns' - ax.XLim(1)));
ax.XTickLabel{2} = ax.XTickLabel(2) + " (m)";
ax.YTickLabel = num2str(ytick_posns - ax.YLim(1)) + " (m)";
ytickangle(90)
view(0, 90)
grid off
ax.FontName = "Times";

%% Create final figure with all arms consolidated together

fig_combined = figure("position", [500, 500, 560, 350]);
ax = axes(fig_combined);

plot_median = true;
arm_2in.plot_dataset_curvature(ax, "color", color_blue, "linespec", "o", "metric", true, "median", plot_median);
arm_3in.plot_dataset_curvature(ax, "color", color_red, "linespec", "square", "metric", true, "median", plot_median);
arm_4in.plot_dataset_curvature(ax, "color", color_yellow, "linespec", "diamond", "metric", true, "median", plot_median);

text_posns = [
  203    1.35;
  324    1.17;
  326    0.65;   
];

arm_sizes = [2, 3, 4];
for i = 1:length(arm_sizes)
    text(ax, text_posns(i, 1), text_posns(i, 2), sprintf("%.2fmm", arm_sizes(i) * 25.4), "fontsize", 12, "fontname", "DejaVu Sans");
end

ylim([0, 1.8])
ax.FontSize = 14;
ax.FontName = "DejaVu Sans";

%title("Planar Arm Model Accuracies")
title("")

%% Create final figure with all arms' tip position errors
fig_combined = figure("position", [500, 500, 560, 350]);
ax = axes(fig_combined);
hold on

color_2in = [100 143 255] / 255;
color_3in = [220 38 127] / 255;
color_yellow = [255 176 0] / 255;
arm_2in.plot_tip_accuracy(ax, "color", color_2in, "linespec", "o:", "metric", true);
arm_3in.plot_tip_accuracy(ax, "color", color_3in, "linespec", "square:", "metric", true);
arm_4in.plot_tip_accuracy(ax, "color", color_yellow, "linespec", "diamond:", "metric", true);

legend(ax, ["50.8mm" "76.2mm" "101.6mm"],'location', 'northeast')
ax.FontSize = 14;
ax.FontName = "Times";
%title("Planar Arm Tip Position Error")
ylim([0, 12])

title("")

%% Create tip position error plot but with scatters and per dataset
arm_datasets = [arm_2in, arm_3in, arm_4in];
fig = figure();
set(fig, "position", [0, 0, 1600, 400])
for i = 1 : length(arm_datasets)
    subplot(1, 3, i);
    arm_datasets(i).plot_tip_accuracy(gca, "color", color_2in, "metric", true, "median", false);
end

%%  
function arm_obj = create_arm_obj(width, base_offset_cm)
    arguments
        width
        base_offset_cm = 3;
    end

    % Create arm object
    g_1 = SE2.hat([0, -base_offset_cm / 100, pi/2]);
    
    rho = width/2 * 0.0254;
    g_o_1 = SE2.hat([0, rho, 0]); % Muscle 1
    g_o_2 = SE2.hat([0, -rho, 0]); % Muscle 2
    g_o = g_1 * inv(g_o_1); % Central muscle
    g_o_muscles = {g_1; g_o * g_o_2};
    l_0 = contraction_fit_poly(0);

    mat_K = diag([1, 0, 0.000098*2]);

    arm_obj = Arm2D(g_o, g_o_muscles, l_0, mat_K, 'plot_unstrained', false);
end