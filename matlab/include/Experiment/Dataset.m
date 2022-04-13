classdef Dataset
    % A "Dataset" is a single experiment datapoint, typically captured
    % within a rosbag
    
    properties
        project_root = "~/tagslam_ws/tagslam_root/src/continuum_mocap";
        experiment_dir
        experiment_name = "default_experiment_name"
        tags
        
        measurements
    end
    
    methods
        function obj = Dataset(experiment_name, options)
            arguments
                experiment_name string
                options.project_root = "~/tagslam_ws/tagslam_root/src/continuum_mocap";
            end
            obj.experiment_name = experiment_name;
            obj.project_root = options.project_root;
            
            dataset_bags = obj.get_dataset_bags(experiment_name, options);
            
            measurements = cell(size(dataset_bags)); % TODO: Create it in the Measurement type
            % For each retrieved bag file object, create a Measurement
            % object and initialize them
            for i = 1 : length(dataset_bags)
                % Create measurement object
                measurement_bag_path = fullfile(bag.folder, bag.name);
                measurements(i) = Measurement(measurement_bag_path);
            end
        end
        
        function outputArg = plot_experiment(obj,inputArg)
        end
    end

    methods(Static)
        function dataset_bags = get_dataset_bags(experiment_name, options)
            arguments
                experiment_name string
                options.project_root = "~/tagslam_ws/tagslam_root/src/continuum_mocap";
            end
            %%% Retrieve Rosbag files of each Measurement in the Dataset
            % <Read rosbag here - Resource Acquisition is Initialization>
            % - Find all bag files in bags dir corresponding to the desired experiment
            % that is specified by `experiment_name`
            bags_dir = fullfile(options.project_root, "bags");

            bag_files = dir(fullfile(bags_dir, experiment_name,'**/*.*'));  % Get list of files and folders in any subfolder
            bag_files = bag_files(~[bag_files.isdir]);  % Remove folders from list

            %%%  Search for localized(tagslammed) bags
            % Create and fill selection mask
            selection_pattern = "localized"; 
            dataset_bags = bag_files(contains({bag_files.name}, selection_pattern));
        end
    end
end

