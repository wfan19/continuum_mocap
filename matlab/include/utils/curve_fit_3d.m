function [h_optim, g_tags_final] = curve_fit_3d(arm_obj, g_tags_meas, t_tags, muscle_ids, options)
    % Find the curve that minimizes the cost function: the distance 
    % between where the tags *should* be given an h_o, and where we
    % have *actually* measured where the tags are.
    
    arguments
        arm_obj
        g_tags_meas
        t_tags
        muscle_ids
        options.init_val (6, 1) = [1; 0; 0; 0; 0; 0];
        options.offsets cell = repelem({eye(4)}, length(g_tags_meas));
    end

    tag_num = length(t_tags);
    
    % Build cost function for fitting the curve
    function [cost, g_tags] = f_cost(h_o)
        cost = 0;
        K = diag([1, 1, 1, 0, 0, 0]);
        
        % Don't store tag poses if called by fminsearch
        if nargout > 1
            g_tags = cell(1, tag_num);
        end
        
        for i = 1 : tag_num
            % Lie algebra distance between curve fit and measured position
            % Right subtraction
            g_o_Xi = inv(arm_obj.muscle_o.g_0) * arm_obj.muscles(muscle_ids(i)).g_0;
            g_o_tag = g_o_Xi * options.offsets{i}; % Apply any offsets that may be needed
            
            % g_meas * g_error = g_tag(h_o)
            % g_error = inv(g_meas) * g_tag(h_o)
            g_tag = arm_obj.muscle_o.g_0 * expm_se3(h_o * t_tags(i)) * g_o_tag;
            SE3_error = inv(g_tags_meas{i}) * g_tag;
            
            % Lie-algebraic (R^6) form of the error vector.
            v_se3_error = vee_se3(logm(SE3_error));
            
            cost_i = v_se3_error' * K * v_se3_error;
            cost = cost + cost_i;
            
            % Don't store tag_poses if called by fminsearch
            if nargout > 1
                g_tags{i} = g_tag;
            end
        end
        
        % Add shear-regulating term
        cost = cost + h_o' * diag([0, 1, 1, 0, 0, 0]) * h_o;
    end

    % Perform optimization
    h_optim = fminsearch(@f_cost, options.init_val);
    
    [~, g_tags_final] = f_cost(h_optim);
end