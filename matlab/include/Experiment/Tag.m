classdef Tag < handle & matlab.mixin.Copyable
    %TAG Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        group = SE3()

        id
        g_pose
        muscle_id % id of the muscle the tag is attached to
        t % Percent position along the arm
    end
    
    methods
        function obj = Tag(v_id, cell_g_poses, v_muscle_ids, v_t, options)
            arguments
                v_id = 0;
                cell_g_poses = {} % Signals its a default value to be initialized later
                v_muscle_ids = zeros(size(v_id));
                v_t = zeros(size(v_id));
                options.group = SE3();
            end

            if isempty(cell_g_poses)
                cell_g_poses = repmat({eye(options.group.mat_size)}, size(v_id));
            end

            % TODO: Validate all input sizes agree
            [m, n] = size(v_id);

            obj(m, n) = copy(obj); % Preallocate array of objects
            for i = 1 : m
                for j = 1 : n
                    obj(i, j).id = v_id(i, j);
                    obj(i, j).g_pose = cell_g_poses{i, j};
                    obj(i, j).muscle_id = v_muscle_ids(i, j);
                    obj(i, j).t = v_t(i, j);
                    obj(i, j).group = options.group;
                end
            end
        end

        function obj = apply_transform(obj, f_transform)
            [m, n] = size(obj);
            for i = 1 : m
                for j = 1 : n
                    obj(i, j).g_pose = f_transform(obj(i, j).g_pose);
                end
            end
        end

        function obj = set_pose_all(obj, cell_g_poses)
            % TODO: Validate all input sizes agree
            [m, n] = size(obj);
            for i = 1 : m
                for j = 1 : n
                    obj(i, j).g_pose = cell_g_poses{i, j};
                end
            end
        end

        function [obj, gh_tforms] = plot_tags(obj, ax, color, size, mesh_file)
            % Applies plotTransform() TF frame plotting to a cell-array
            %of SE3 pose matrices
            
            arguments
                obj Tag
                ax
                color = "red";
                size = 0.03;
                mesh_file = "tf_frame.stl";
            end

            if isa(ax, "matlab.ui.Figure")
                ax = axes(ax);
            end
            
            if isa(obj(1).group, "SE2")
                % SE2
                f_cellfun_t = @(mat) [mat(1:2, 3); 0];
                f_cellfun_quat = @(mat) transpose(rotm2quat(blkdiag(mat(1:2, 1:2), 1)));
            elseif isa(obj(1).group, "SE3")
                % SE3
                f_cellfun_t = @(mat) mat(1:3, 4);
                f_cellfun_quat = @(mat) transpose(rotm2quat(mat(1:3, 1:3)));
            else
                error("Unrecognized group value for Tag object")
            end

            g_tforms = {obj.g_pose};

            f_extract_with_cellfun = @(f_cellfun) ...
                cell2mat(cellfun(f_cellfun, g_tforms, 'uniformoutput', false));
            
            % Apply "extraction" functions to the cell array
            t_tforms = transpose(f_extract_with_cellfun(f_cellfun_t));
            quat_tforms = transpose(f_extract_with_cellfun(f_cellfun_quat));
            
            % Set plotTransforms Parent options to the specified Axis
            plot_options = struct();
            plot_options.MeshFilePath = mesh_file;
            plot_options.framesize = size;
            plot_options.MeshColor = color;
            plot_options.parent = ax;
            
            plotTransforms(t_tforms, quat_tforms, plot_options);
            
            % Newest axis children (corresponding to the plotTransform) is pre-pended
            % to Axis.Children so we can grab the Group Handle (gh).
            gh_tforms = ax.Children(1);
            
            if isa(obj(1).group, "SE2")
                view(ax, [0, 90]);
            end

            grid on
            axis equal
        end

        % Getter and setter functions, so that the group and pose matrix
        % size always match
        function set.group(obj, group)
            obj.group = group;

            if all(size(obj.g_pose) ~= group.mat_size)
                if isa(group, "SE3")
                    % Elseif the pose matrix is currently in SE2, bring it up
                    % to SE3.
                    v_SE2 = SE2.vee(obj.g_pose);
                    R_SE3 = eul2rotm('zyx', [v_SE2(3), 0, 0]);
                    t_SE3 = [v_SE2(:); 0];
                    
                    obj.g_pose = SE3.hat(R_SE3, t_SE3);
                elseif isa(group, "SE2")
                    % If the pose matrix is currently in SE3, flatten it to SE2
                    t_xy = obj.g_pose(4, 1:2);
                    eul = rotm2eul(R(1:3, 1:3), "zyx");
                    theta = eul(1);
                    
                    obj.g_pose = SE2.hat([t_xy(:); theta]);
                else
                    warning("Tag: Possibly mismatched group dimensions and pose matrix size")
                end
            end
        end

        function set.g_pose(obj, g_pose)
            obj.g_pose = g_pose;
            if all(obj.group.mat_size ~= size(g_pose))
                if all(size(g_pose) == SE2.mat_size)
                    obj.group = SE2;
                elseif all(size(g_pose) == SE3.mat_size)
                    obj.group = SE3;
                end
            end
        end
    end
end

