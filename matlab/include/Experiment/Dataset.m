classdef Dataset < handle & matlab.mixin.Copyable
    % A "Dataset" is a single experiment datapoint, typically captured
    % within a rosbag
    
    properties
        group

        project_root = "~/tagslam_ws/tagslam_root/src/continuum_mocap";
        
        measurements
        tab_measurements
    end
    
    methods
        % TODO: Refactor such that parameters are passed with a
        % DatasetParams object
        function obj = Dataset(dataset_name, base_tag, tags, arm_obj, f_parse_bag, g_tag_offset, options)
            arguments
                dataset_name string
                base_tag
                tags
                arm_obj
                f_parse_bag = @() deal(zeros(size(arm_obj.muscles)), "default")
                g_tag_offset = []
                options.project_root = "~/tagslam_ws/tagslam_root/src/continuum_mocap";
                options.group = SE3()
            end
            obj.project_root = options.project_root;
            obj.group = options.group;
            obj.tab_measurements = table();

            obj = obj.add_dataset(dataset_name, base_tag, tags, arm_obj, f_parse_bag, g_tag_offset);
        end

        function obj = add_dataset(obj, dataset_name, base_tag, tags, arm_obj, f_parse_bag, g_tag_offset)
            arguments
                obj
                dataset_name string
                base_tag
                tags
                arm_obj
                f_parse_bag = @() deal(zeros(size(arm_obj.muscles)), "default")
                g_tag_offset = []
            end
            %%% Retrieve Rosbag files of each Measurement in the Dataset
            % <Read rosbag here - Resource Acquisition is Initialization>
            % - Find all bag files in bags dir corresponding to the desired experiment
            % that is specified by `experiment_name`
            dataset_bags = obj.get_dataset_bags(dataset_name, "project_root", obj.project_root);
            
            dataset_measurements = Measurement.empty(0, length(dataset_bags));

            % For each retrieved bag file object, create a Measurement
            % object and initialize them
            for i = 1 : length(dataset_bags)
                % Create measurement object
                measurement_bag_path = fullfile(dataset_bags(i).folder, dataset_bags(i).name);

                % Determine muscle pressures and measurement "label" based
                % on the bag file's name
                [v_pressure, measurement_label] = f_parse_bag(dataset_bags(i).name);
                dataset_measurements(i) = Measurement( ...
                    measurement_bag_path, ...
                    v_pressure, ...
                    base_tag, ...
                    tags, ...
                    arm_obj, ...
                    g_tag_offset, ...
                    "group", obj.group ...
                );
                dataset_measurements(i).label = measurement_label;
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
    end

    methods(Static)
        function dataset_bags = get_dataset_bags(dataset_name, options)
            arguments
                dataset_name string
                options.project_root = "~/tagslam_ws/tagslam_root/src/continuum_mocap";
                options.search_string = '**/*.*' % By default fetches all files and folders in *subfolders* of dataset folder
            end
            bags_dir = fullfile(options.project_root, "bags");

            bag_files = dir(fullfile(bags_dir, dataset_name, options.search_string)); 
            bag_files = bag_files(~[bag_files.isdir]);  % Remove folders from list

            %%%  Search for localized(tagslammed) bags
            % Create and fill selection mask
            selection_pattern = "localized"; 
            dataset_bags = bag_files(contains({bag_files.name}, selection_pattern));
        end
    end
end

