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
        function obj = plot_dataset_curvature(obj, ax, muscle_pressurized, color, options)
            arguments
                obj Dataset
                ax = axes(figure())
                muscle_pressurized = 0
                color = ""
                options.true_curvature = false
            end

            if isa(obj.group, "SE2")
                if options.true_curvature
                    f_curvature = @(mat_h_o) abs(mat_h_o(:, 3) ./ mat_h_o(:, 1));
                else
                    f_curvature = @(mat_h_o) abs(mat_h_o(:, 3));
                end
            elseif isa(obj.group, "SE3")
                if options.true_curvature
                    f_curvature = @(mat_h_o) vecnorm(mat_h_o(:, 4:6)')' ./ mat_h_o(:, 1);
                else
                    f_curvature = @(mat_h_o) vecnorm(mat_h_o(:, 4:6)')';
                end
            end

            if muscle_pressurized == 0 % Find default muscle to pressurize based on tab_meas v_pressure
                [~, muscle_pressurized] = max(std(obj.tab_measurements.v_pressure));
            end
            
            hold(ax, 'on')
            plot( ...
                obj.tab_measurements.v_pressure(:, muscle_pressurized), ...
                f_curvature(obj.tab_measurements.h_o_fit), ...
                "bx", "Linewidth", 3 ...
            )
            
            plot( ...
                obj.tab_measurements.v_pressure(:, muscle_pressurized), ...
                f_curvature(obj.tab_measurements.h_o_model), ...
                "r:", "Linewidth", 3 ...
            )

            hold(ax, 'off')
            legend(["Experiment", "Model"], 'location', 'southeast')
            title(obj.dataset_params.dataset_name + ": Pressure vs Curvature")
            xlabel("Pressure (psi)")
            ylabel("Curvature (1/m)")
            grid on
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

