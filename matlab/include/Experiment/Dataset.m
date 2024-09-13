classdef Dataset < handle & matlab.mixin.Copyable
    % A "Dataset" is a single experiment datapoint, typically captured
    % within a rosbag
    
    properties(Constant)
       default_project_root = "~/tagslam_ws/tagslam_root/src/continuum_mocap";
    end

    properties
        project_root
        
        group
        
        measurements
        tab_measurements

        dataset_params
    end
    
    methods
        function obj = Dataset(dataset_path, dataset_params, options)
            arguments
                dataset_path
                dataset_params
                options.project_root = Dataset.default_project_root;
            end
            obj.project_root = options.project_root;
            obj.group = dataset_params.group;
            obj.tab_measurements = table();
            obj.dataset_params = copy(dataset_params);

            obj = obj.add_dataset(dataset_path, copy(dataset_params));
        end

        function obj = add_dataset(obj, dataset_path, params)
            arguments
                obj
                dataset_path string
                params DatasetParams
            end
            %%% Retrieve Rosbag files of each Measurement in the Dataset
            % <Read rosbag here - Resource Acquisition is Initialization>
            % - Find all bag files in bags dir corresponding to the desired experiment
            % that is specified by `experiment_name`
            dataset_bags = obj.get_dataset_bags(dataset_path, "project_root", obj.project_root);
            
            dataset_measurements = Measurement.empty(0, length(dataset_bags));

            % For each retrieved bag file object, create a Measurement
            % object and initialize them
            for i = 1 : length(dataset_bags)
                    fprintf("Loading bag %s\n", dataset_bags(i).name);
    
                    % Create measurement object
                    measurement_bag_path = fullfile(dataset_bags(i).folder, dataset_bags(i).name);
                    
                    % Determine muscle pressures and measurement "label" based
                    % on the bag file's name
                    [v_pressure, measurement_label] = params.f_parse_bag(dataset_bags(i).name);
                try
                    dataset_measurements(i) = Measurement( ...
                        measurement_bag_path, ...
                        v_pressure, ...
                        params ...
                    );
                    dataset_measurements(i).label = measurement_label;
                catch ME
                    if strcmp(ME.identifier, "Measurement:MissingBaseTag")
                        warning("Measurement %s missing base tag", dataset_bags(i).name)
                    else
                        rethrow(ME)
                    end
                end
            end
            obj.measurements = [obj.measurements, dataset_measurements];

            % Create table of measurement data from this dataset
            table_variable_names = {'label', 'v_pressure', 'h_o_model', 'h_o_fit'};
            dataset_tab_measurements = table( ...
                [dataset_measurements.label]', ...
                [dataset_measurements.v_pressure]', ...
                [dataset_measurements.h_o_model]', ...
                [dataset_measurements.h_o_fit]', ...
                'VariableNames',table_variable_names ...
            );
            obj.tab_measurements = [obj.tab_measurements; dataset_tab_measurements];
        end
    
        % TODO: Implement allowing color choices and default blue/red color
        % scheme
        function obj = plot_dataset_curvature(obj, ax, muscle_pressurized, options)
            arguments
                obj Dataset
                ax = axes(figure())
                muscle_pressurized = 0
                options.color = ""
                options.linespec = "x"
                options.true_curvature = false
                options.median = true
                options.errorbar = false
                options.metric = false
            end

            function out = calc_curvature(mat_h)
                if isa(obj.group, "SE2")
                    out = abs(mat_h(:, 3));
                elseif isa(obj.group(), "SE3")
                    out = vecnorm(mat_h(:, 4:6)')';
                else
                    error("Dataset object of unrecognized group")
                end

                % Normalize by the arclenght if we are plotting the true
                % curvature
                if options.true_curvature
                    out = out ./ mat_h(:, 1);
                end
            end
            if muscle_pressurized == 0 % Find default muscle to pressurize based on tab_meas v_pressure
                [~, muscle_pressurized] = max(std(obj.tab_measurements.v_pressure));
            end

            pressure_min = obj.tab_measurements.v_pressure(1, muscle_pressurized);
            pressure_max = obj.tab_measurements.v_pressure(end, muscle_pressurized);
            t_pressure = linspace(pressure_min, pressure_max, 100);

            % Calculate the continuous version of the model expected values
            % line.
            muscle_pressures_zero = zeros(size(obj.tab_measurements.v_pressure, 2) , 1);
            v_l = obj.dataset_params.f_contraction_model(muscle_pressures_zero);
            mat_h_o_model = zeros(length(t_pressure), size(obj.tab_measurements.h_o_fit, 2));
            for i = 1:length(t_pressure)
                v_l(muscle_pressurized) = obj.dataset_params.f_contraction_model(t_pressure(i));
                mat_h_o_model(i, :) = (obj.dataset_params.arm_obj.mat_N * v_l)';
            end

            pressures_meas = obj.tab_measurements.v_pressure(:, muscle_pressurized);
            h_o_fit = obj.tab_measurements.h_o_fit;
            % If we want to just plot the medians, find groups by pressure
            % and find the medians for each of those groups.
            if options.median
                [groups, pressures_meas] = findgroups(pressures_meas);
                h_o_fit = splitapply(@(vals) median(vals, 1), h_o_fit, groups);
            end

            % If given a color, apply the color and use a darker version
            % for the model line.
            if isa(options.color, "string") && options.color == ""
                meas_color = "b";
                model_color = "r";
            elseif isa(options.color, "double")
                meas_color = options.color;
                model_color = hsv2rgb([1, 1, 0.75] .* rgb2hsv(meas_color));
            end

            % Convert psi to kpa if plotting in metric.
            if options.metric
                pressures_meas = pressures_meas * 6.89476;
                t_pressure = t_pressure * 6.89476;
            end
            
            hold(ax, 'on')
            plot(t_pressure, calc_curvature(mat_h_o_model),":", "Linewidth", 5, color=model_color)
            plot(pressures_meas, calc_curvature(h_o_fit), options.linespec, "Linewidth", 3, "MarkerSize", 10, color=meas_color)

            hold(ax, 'off')
            legend(["Model", "Experiment"], 'location', 'southeast')
            title(obj.dataset_params.dataset_name + ": Pressure vs Curvature")

            if options.metric
                xlabel("Pressure (kPa)")
                xlim([min(pressures_meas), max(pressures_meas)])
            else
                xlabel("Pressure (psi)")
            end
            
            if options.true_curvature
                ylabel("True Curvature (1/m)")
            else
                ylabel("Scaled Curvature (rad)")
            end
            grid on
        end
    
        function plot_tip_accuracy(obj, ax, options)
            arguments
                obj Dataset
                ax = axes(figure())
                options.color = "b"
                options.linespec = "o:"
                options.median = true
                options.normalize = true
                options.muscle_inflated = -1
                options.metric = false
            end
            if options.muscle_inflated == -1 % Find default muscle to pressurize based on tab_meas v_pressure
                [~, options.muscle_inflated] = max(std(obj.tab_measurements.v_pressure));
            end

            numels = size(obj.tab_measurements, 1);

            % Shorthand names for referencing
            group = obj.group;
            algebra = obj.group.algebra;

            posn_error = zeros(1, numels);
            for i = 1:numels
                fit_tip_posn = algebra.expm(algebra.hat(obj.tab_measurements.h_o_fit(i, :)));
                model_tip_posn = algebra.expm(algebra.hat(obj.tab_measurements.h_o_model(i, :)));

                % Instead of a proper box-minus (with a log map of the
                % difference) here we just keep things in the Lie group.
                %
                % It's just the norm of the translation so who cares,
                % right?
                v_displacement = algebra.vee(inv(fit_tip_posn) * model_tip_posn);

                if length(v_displacement) == 3
                    posn_error(i) = norm(v_displacement(1:2));
                elseif length(v_displacement) == 6
                    posn_error(i) = norm(v_displacement(1:3));
                end
            end
            if options.normalize
                posn_error = posn_error / obj.dataset_params.f_contraction_model(0) * 100;
            end
            
            outlier_datasets = posn_error > 10 & 1:length(posn_error) > 10;
            
            pressures = obj.tab_measurements.v_pressure(~outlier_datasets, options.muscle_inflated);
            posn_errors = posn_error(~outlier_datasets);

            if options.median
                [groups, pressures] = findgroups(pressures);
                posn_errors = splitapply(@median, posn_errors(:), groups(:));
            end

            if options.metric
                psi_to_kpa = 6.89476;
                pressures = pressures * psi_to_kpa;
            end
            
            if options.median
                plot(ax, pressures, posn_errors, options.linespec, "markersize", 10, "color", options.color, "linewidth", 3, "markerfacecolor", options.color)
            else
                plot(ax, pressures, posn_errors, 'x', "color", options.color, "linewidth", 5)
            end

            if options.metric
                xlim(ax, [min(pressures), max(pressures)])
                xlabel(ax, "Pressure (kPa)")
            else
                xlabel(ax, "Pressure (psi)")
            end

            if options.normalize
                ylabel(ax, "Tip position error")
                ytickformat("percentage")
            else
                ylabel(ax, "Tip position error (m)")
            end
            title(ax, "Tip Position Error")
            grid(ax, "on")
        end
    end

    methods(Static)
        function dataset_bags = get_dataset_bags(dataset_name, search_string, options)
            arguments
                dataset_name string
                search_string = '**/*.*' % By default fetches all files and folders in *subfolders* of dataset folder
                options.project_root = Dataset.default_project_root;
            end
            bags_dir = fullfile(options.project_root, "bags");

            bag_files = dir(fullfile(bags_dir, dataset_name, search_string)); 
            bag_files = bag_files(~[bag_files.isdir]);  % Remove folders from list

            %%%  Search for localized(tagslammed) bags
            % Create and fill selection mask
            selection_pattern = "localized"; 
            dataset_bags = bag_files(contains({bag_files.name}, selection_pattern));
        end
    end
end

