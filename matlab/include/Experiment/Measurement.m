classdef Measurement
    %CAPTURE A "Measurement" is a single "experiment measurement" - a capturing
    %of the poses of Apriltags attached to a manipulator inflated to a
    %certain pressure.
    
    properties
        bag_path % Store the path to the rosbag and retrieve as needed

        % Experiment Setup Information?
        v_pressure

        tags_meas
        tags_fit

        arm_obj
        f_contraction_model = @contraction_fit;

        % ??
        h_o_fit
        h_o_model
        group
    end
    
    methods
        function obj = Measurement(bag_path, v_pressure, base_tag, tags, arm_obj, options)
            arguments
                bag_path string
                v_pressure double
                base_tag Tag
                tags Tag
                arm_obj Arm
                options.group = SE3()
            end
            % Create Measurement object and extract tag poses
            obj.bag_path = bag_path;
            obj.v_pressure = v_pressure(:);
            obj.tags_meas = tags;
            obj.arm_obj = arm_obj;
            obj.group = options.group;

            bag_obj = rosbag(bag_path);
            
            % Read tags poses from bag object here
            for i = 1 : length(tags)
                id_tag = tags(i).id;

                % Make sure that the tag we're looking for has an existing
                % transformation in the TF tree
                if ~any(bag_obj.AvailableFrames == sprintf("tag_%d", id_tag))
                    error("Tag with id %d not found in list of available frames", id_tag)
                    % TODO: Maybe just warn instead of error out
                end

                % Get transformation from base tag to current tag
                name_base_tag = "tag_" + base_tag.id;
                name_id_tag = "tag_" + id_tag;
                tform_i = getTransform(bag_obj, name_base_tag, name_id_tag); % tag_6 as planar arm base tag for now
                
                R = quat2rotm(tform_i.Transform.Rotation.readQuaternion());
                % Apply extra 90 deg ccw rotation to correct for misplaced tags
                % TODO: Re-evaluate?
                R = R * eul2rotm([pi/2, 0, 0], 'zyx');

                T_obj = tform_i.Transform.Translation;
                T = [T_obj.X; T_obj.Y; T_obj.Z];

                if obj.group.dof == 6
                    % We are in 3D: create SE3 elements
                    obj.tags_meas(i).g_pose = SE3.hat(R, T);
                elseif obj.group.dof == 3
                    % We are in 2D / planar arm
                    % Extract yaw values and create new SE2 g_tags
                    eul_i = rotm2eul(R(1:3, 1:3), "zyx");
                    theta = eul_i(1);
    
                    x_y = T(1:2);
    
                    obj.tags_meas(i).group = SE2();
                    obj.tags_meas(i).g_pose = SE2.hat([x_y(1), x_y(2), theta]);
                end
            end

            [obj, obj.tags_fit] = obj.fit_curve();
        end
        
        function [obj, tags_fit] = fit_curve(obj, options)
            arguments
                obj
                options.offsets = repmat({eye(obj.group.mat_size)}, 1, length(obj.tags_meas));
            end
            tag_num = length(obj.tags_meas);
            tags_fit = obj.tags_meas;
            
            % Build cost function for fitting the curve
            function [cost, cell_g_tags] = f_cost(h_o)
                cost = 0;
                if obj.group.dof == 3
                    K = diag([1, 1, 0]);
                elseif obj.group.dof == 6
                    K = diag([1, 1, 1, 0, 0, 0]);
                end
                
                % Don't store tag poses if called by fminsearch
                if nargout > 1
                    cell_g_tags = cell(1, tag_num);
                end
                
                for i = 1 : tag_num
                    % Lie algebra distance between curve fit and measured position
                    % Right subtraction
                    g_o_Xi = inv(obj.arm_obj.muscle_o.g_0) * obj.arm_obj.muscles(obj.tags_meas(i).muscle_id).g_0;
                    % TODO: handle offsets
                    g_o_tag = g_o_Xi * options.offsets{i}; % Apply any offsets that may be needed
                    
                    % g_meas * g_error = g_tag(h_o)
                    % g_error = inv(g_meas) * g_tag(h_o)
                    g_tag = obj.arm_obj.muscle_o.g_0 * obj.group.algebra.expm(h_o * obj.tags_meas(i).t) * g_o_tag;
                    SE3_error = inv(obj.tags_meas(i).g_pose) * g_tag;
                    
                    % Lie-algebraic (R^6) form of the error vector.
                    v_se3_error = obj.group.algebra.vee(logm(SE3_error));
                    
                    cost_i = v_se3_error' * K * v_se3_error;
                    cost = cost + cost_i;
                    
                    % Don't store tag_poses if called by fminsearch
                    if nargout > 1
                        cell_g_tags{i} = g_tag;
                    end
                end
                
                % Add shear-regulating term
                if obj.group.dof == 3
                    mat_shear_cost = diag([0, 1, 0]);
                elseif obj.group.dof == 6
                    mat_shear_cost = diag([0, 1, 1, 0, 0, 0]);
                end
                cost = cost + h_o' * mat_shear_cost * h_o;
            end
        
            % Calculate initial value: The h_o vector predicted by the
            % model
            v_l = obj.f_contraction_model(obj.v_pressure);
            obj.h_o_model = obj.arm_obj.mat_N * v_l;

            % Perform optimization
            [obj.h_o_fit, ~] = fminsearch(@f_cost, obj.h_o_model);
            
            [~, cell_g_tags_fit] = f_cost(obj.h_o_fit);
            tags_fit = tags_fit.set_pose_all(cell_g_tags_fit);
            obj.tags_fit = tags_fit;
        end
    end
end

