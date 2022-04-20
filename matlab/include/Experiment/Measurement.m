classdef Measurement < handle & matlab.mixin.Copyable
    %CAPTURE A "Measurement" is a single "experiment measurement" - a capturing
    %of the poses of Apriltags attached to a manipulator inflated to a
    %certain pressure.
    
    properties
        group

        bag_path % Store the path to the rosbag and retrieve as needed

        % Experiment Setup Information?
        v_pressure
        label

        tags_meas
        tags_fit
        g_tag_offset
        f_transform_tags

        arm_obj
        f_contraction_model = @contraction_fit;

        % ??
        h_o_fit
        h_o_model

        % Plotting parameters
        gh_tags_meas
        gh_tags_fit

    end
    
    methods
        function obj = Measurement(bag_path, v_pressure, base_tag, tags, arm_obj, g_tag_offset, options)
            arguments
                bag_path string
                v_pressure double
                base_tag Tag
                tags Tag
                arm_obj Arm
                g_tag_offset = []
                options.group = SE3()
            end
            % Create Measurement object and extract tag poses
            obj.bag_path = bag_path;
            obj.v_pressure = v_pressure(:);
            obj.tags_meas = copy(tags);
            obj.arm_obj = arm_obj;
            obj.group = options.group;
            if isempty(g_tag_offset)
                obj.g_tag_offset = eye(obj.group.mat_size);
            else
                obj.g_tag_offset = g_tag_offset;
            end

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
                % TODO: Moving this to a more universal approach
                % R = R * eul2rotm([pi/2, 0, 0], 'zyx');

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

            % We want to "align/attach" the measured tags to the oriented 
            % arm object. Specifically, the base_tag needs to be on the
            % base muscle. We want to find the *left* transformation that
            % does this, so we can transform *all* of the tags
            % 
            % g_transform * g_base_tag = g_0_i * g_offset
            % => g_transform = g_0_i * g_offset * inv(g_base_tag)
            % where i is the base_tag's id. This aligns the base_tag to be
            % "attached to the muscle" in the world frame.
            g_alignment_transform = ...
                arm_obj.muscles(base_tag.muscle_id).g_0 * obj.g_tag_offset * inv(base_tag.g_pose);

            obj.tags_meas.apply_transform(@(g_tag) g_alignment_transform * g_tag);

            [obj, obj.tags_fit] = obj.fit_curve();
        end
        
        function [obj, tags_fit] = fit_curve(obj)
            arguments
                obj
            end
            tag_num = length(obj.tags_meas);
            tags_fit = copy(obj.tags_meas);
            
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

                    % First let's get the transformation from the
                    % base-curve to the muscle the tag belongs to
                    g_o_Xi = inv(obj.arm_obj.muscle_o.g_0) * obj.arm_obj.muscles(obj.tags_meas(i).muscle_id).g_0;
                    g_o_tag = g_o_Xi * obj.g_tag_offset; % Apply additional tag-frame constant offset (such as mounting height)
                    
                    % g_tag = g_0 * expm(h_o * t) * g_o_tag: Transform to
                    % arm base, flow along base curve for given amount of
                    % distance, and then transform from base curve to tag
                    % pose
                    g_tag = obj.arm_obj.muscle_o.g_0 * obj.group.algebra.expm(h_o * obj.tags_meas(i).t) * g_o_tag;

                    % Calculate error vector
                    SEn_error = inv(obj.tags_meas(i).g_pose) * g_tag;
                    
                    % Lie-algebraic (R^6) form of the error vector.
                    v_se3_error = obj.group.algebra.vee(logm(SEn_error));
                    
                    % Compute quardatic cost
                    cost_i = v_se3_error' * K * v_se3_error;
                    cost = cost + cost_i; % Running sum of cost
                    
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
%                 cost = cost + h_o' * mat_shear_cost * h_o;
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

        function plot_measurement(obj, ax, plot_tags)
            arguments
                obj
                ax
                plot_tags = true;
            end

            grid on
            axis equal

            %%% Plot the arms
            % Initialize plotting
            model_arm = copy(obj.arm_obj);
            fit_arm = copy(obj.arm_obj);
            
            % Set plotting parameters
            line_options_muscles = struct("LineWidth", 3);
            line_options_spacers = struct("LineWidth", 2);
            
            f_initialize_plotting = @(obj) ...
                obj.initialize_plotting(...
                    ax, ...
                    "line_options_muscles", line_options_muscles, ...
                    "line_options_spacers", line_options_spacers);

            f_initialize_plotting(model_arm);
            f_initialize_plotting(fit_arm);
            
            % Darken plotting colors in model arm
            for i = 1 : length(model_arm.muscles)
                color_current = model_arm.muscles(i).color;
                model_arm.muscles(i).color = hsv2rgb(rgb2hsv(color_current) .* [1, 1, 0.75]);
                model_arm.muscles(i).lh.LineStyle = ":";
            end

            % Make model arm spacers also dashed
            for i = 1 : length(model_arm.v_lh_spacers)
                model_arm.v_lh_spacers(i).LineStyle = ":";
            end
            
            % Bring solid line (Fitted muscle curves) to the back
            for i = 1 : length(fit_arm.muscles)
                uistack(fit_arm.muscles(i).lh, "bottom");
            end

            % Actually plot the arms now
            v_l = contraction_fit_poly(obj.v_pressure);
            model_arm.update_arm(v_l, obj.h_o_model);
            %fit_arm.update_arm(v_l, h_o_fit .* [1 0 0 1 1 1]'); % Ignore shearing
            fit_arm.update_arm(v_l, obj.h_o_fit);

            % Plot the tags
            if plot_tags
                obj.gh_tags_meas = obj.tags_meas.plot_tags(ax);
                obj.gh_tags_fit = obj.tags_fit.plot_tags(ax, "green");
            end
        end
    end
end

