function dataset_obj = load_helical_3_muscle_7_18
    try
        load("helical_dataset.mat", "dataset_obj");
    catch
        project_dir_name = "apr_2022/helical_3muscle_2in_7_17";
        color_blue = [100 143 255] / 255;
        color_red = [220 38 127] / 255;
        color_yellow = [255 176 0] / 255;
        color_green = [0, 191, 159] / 255;
        
        %% Parameterize tag locations
        % ids of tags along the arm (and the order that we will store them in)
        experiment_tags = [10 11 30 31 6 7 8 9]; % Tags 6 to 9 
        base_tag_id = 6;
        
        % the muscles that these muscles are attached to (in the same order) 
        tag_muscle_ids = [2 2 2 2 3 3 3 3]; % List of muscle ids that the tags are attached to, in order
        
        % Define t_tags: Percentage of tag position along total length of arm
        % Tag in the middle of arm: t = 0.5
        muscle_length_cm = contraction_fit_summer(0) * 100;
        posns_tags_left = [0, 11.5, 24.5, 36.5]; % inches
        posns_tags_right = posns_tags_left;
        t_tags = ([posns_tags_left, posns_tags_right]) / muscle_length_cm;
        
        tags = Tag(experiment_tags, repmat({eye(4)}, size(experiment_tags)), tag_muscle_ids, t_tags);
        base_tag = tags([tags.id] == base_tag_id);
    
        %% Parameterize the arm
        rho = 1 * 0.0254; % M, Radius
        muscle_angle = deg2rad(15);
        
        %% Construct pose matrices
        % Transformations from world frame to each muscle
        g_default_muscle = SE3.hat(eul2rotm([0, pi/2, 0], 'xyz'), [0 0 0]); % Frame corresponding to a muscle at the origin, pointing down
        
        % Posese of muscle in world frame
        % Muscles are arranged in equilateral triangle with radius rho
        g_muscles = Arm3D.generate_g_muscles(rho, g_default_muscle, 3, muscle_angle);
        
        % Set position of first muscle to actually be at the origin
        % We want to shift *everything* uniformly regardless of its pose, therefore
        % it'll be a left-action:
        % g_shift * g_1 = g_neutral
        % Therefore g_shift = g_neutral * inv(g_1)
        % Additionally, we rotate by a global 180deg so that all 3 muscles are
        % aligned correctly
        %g_muscle_shift = g_default_muscle * inv(g_muscles{1});
        g_muscle_shift=eye(4);
        
        % Then left-apply g_shift to all transformations
        g_o = g_muscle_shift * g_default_muscle;
        g_muscles = cellfun(@(g_muscle) g_muscle_shift * g_muscle, g_muscles, 'uniformoutput', false);
        
        mat_K = diag([1, 100, 100, 0, 0.000098, 0.000098]);
        
        arm_obj = Arm3D(g_o, g_muscles, contraction_fit(0), "mat_K", mat_K, 'plot_unstrained', false);
        arm_obj.n_spacers = 8;
        arm_obj.muscles(1).color = color_red;
        arm_obj.muscles(2).color = color_green;
        arm_obj.muscles(3).color = color_blue;
        
        g_tag_offset = SE3.hat(eul2rotm([0, 0, 0], 'zyx'), [0; -0.461; 1.675] * 0.0254);
        g_global_offset = SE3.hat(eul2rotm([0, deg2rad(0), 0], 'zyx'), zeros(3, 1));
        
        dataset_params = DatasetParams("Spatial 3 muscle", SE3, arm_obj, base_tag, tags, g_tag_offset);
        dataset_params.f_parse_bag = @(bag_name) DatasetParams.parse_bag_generic(bag_name, 3, 1);
        dataset_params.g_global_offset = g_global_offset;
        dataset_params.f_contraction_model = @contraction_fit_summer;
        dataset_obj = Dataset(project_dir_name, dataset_params);
        
        save("helical_dataset.mat", "dataset_obj")
    end
end

