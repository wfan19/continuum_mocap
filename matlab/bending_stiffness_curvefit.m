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

g_tag_offset = SE2.hat([0 0 0]);

arm_width = 3; % Inches
arm_obj_3in = create_arm_obj(arm_width);

dataset_params = DatasetParams("3 inch arm", SE2, arm_obj_3in, base_tag, tags, g_tag_offset);
dataset_params.f_contraction_model = @contraction_fit_poly;
dataset_params.f_parse_bag = @(bag_name) DatasetParams.parse_bag_generic(bag_name, 2, 1);

arm_3in = Dataset("apr_2022/planar_arm_3in_4_30", dataset_params);

%% Create expected displacement vs curvature plot
[pressure_group, pressures] = findgroups(arm_3in.tab_measurements.v_pressure(:, 1));

expected_displacements = dataset_params.f_contraction_model(pressures);

h_o_fit_medians = splitapply(@median, arm_3in.tab_measurements.h_o_fit, pressure_group);
h_o_fit_stds = splitapply(@std, arm_3in.tab_measurements.h_o_fit, pressure_group);

measured_curvatures = h_o_fit_medians(:, 3);
curvefit = fit(expected_displacements(:), measured_curvatures(:), 'poly1');

d = arm_width * 0.0254/2;
bending_stiffness_meas = ((-d / curvefit.p1) - 2*d^2) / 2;
t_expected_displacements = linspace(min(expected_displacements) * 0.975, max(expected_displacements) * 1.025, 100);
curvatures_fit = curvefit(t_expected_displacements);

figure()
hold on
%errorbar(expected_displacements, measured_curvatures, h_o_fit_stds(:, 3), 'bo', "linewidth", 2)
plot(expected_displacements, measured_curvatures, "bo", "linewidth", 2)
plot(t_expected_displacements, curvatures_fit, 'r--', "linewidth", 2)

xlabel("Muscle neutral length (m)")
ylabel("Scaled Curvature (unitless)")
grid on

%% Get end position error versus pressure
posn_error = zeros(length(arm_3in.measurements), 1);
for i = 1:length(arm_3in.measurements)
    displacement = SE2.vee(inv(se2.expm(se2.hat(arm_3in.measurements(i).h_o_fit))) * se2.expm(se2.hat(arm_3in.measurements(i).h_o_model)));
    posn_error(i) = norm(displacement(1:2));
end
posn_error_normalized = posn_error / dataset_params.f_contraction_model(0) * 100;

posn_error_medians = splitapply(@median, posn_error_normalized, pressure_group);
posn_error_stds = splitapply(@std, posn_error_normalized, pressure_group);

figure()
errorbar(pressures, posn_error_medians, posn_error_stds, 'b', "linewidth", 2);
xlabel("Pressure (psi)")
ylabel("Tip position error (% of body length)")
title("3 Inch Arm End Position Error")
grid on

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

    mat_K = diag([1, 0, 0.000098]); % Remember to use 20

    arm_obj = Arm2D(g_o, g_o_muscles, l_0, mat_K, 'plot_unstrained', false);
end