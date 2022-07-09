function [bag_obj, bag_name] = get_tagslam_bag(experiment_name, pressure, bag_choice)

    project_root = "~/tagslam_ws/tagslam_root/src/continuum_mocap";

    %% Find all bag files in bags dir corresponding to the desired experiment
    %  that is specified by `experiment_name`
    bags_dir = fullfile(project_root, "bags");

    bag_files = dir(fullfile(bags_dir, experiment_name,'**/*.*'));  % Get list of files and folders in any subfolder
    bag_files = bag_files(~[bag_files.isdir]);  % Remove folders from list

    %% Search for localized(tagslammed) bags
    % Create and fill selection mask
    tagslam_bags = search_bag_list_by_name(bag_files, "localized");

    % Let's just test with the first bag file
    bags_at_pressure = search_bag_list_by_name(tagslam_bags, "-" + pressure + "psi");
    
    bag = bags_at_pressure(bag_choice);
    bag_obj = rosbag(fullfile(bag.folder, bag.name));
    
    bag_name = bag.name;
    
    function bags_search = search_bag_list_by_name(bag_list, search_str)
        selection_mask = zeros(size(bag_list));
        for i = 1 : length(bag_list)
            if contains(bag_list(i).name, search_str)
                selection_mask(i) = 1;
            end
        end
        
        bags_search = bag_list(logical(selection_mask));
    end

end

