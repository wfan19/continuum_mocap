classdef DatasetParams < matlab.mixin.Copyable
    properties
        group
        arm_obj

        dataset_name

        base_tag
        tags
        g_tag_offset
        g_global_offset

        f_parse_bag = @() deal(zeros(size(arm_obj.muscles)), "default");
    end
    
    methods
        function obj = DatasetParams(dataset_name, group, arm_obj, base_tag, tags, g_tag_offset)
            arguments
                dataset_name
                group
                arm_obj
                base_tag = Tag()
                tags = Tag()
                g_tag_offset = []
            end

            if isempty(g_tag_offset)
                g_tag_offset = zeros(group.mat_size);
            end

            obj.dataset_name = dataset_name;
            obj.group = group;
            obj.arm_obj = arm_obj;
            obj.base_tag = base_tag;
            obj.tags = tags;
            obj.g_tag_offset = g_tag_offset;

            obj.g_global_offset = eye(obj.group.mat_size);
        end
    end

    methods(Static)
        % Generic for parsing bags
        % Meant to be curryed to be used as an f_parse_bag function
        % IE: f_parse_bag = @(bag_name) DatasetParams.parse_bag_n_muscles(bag_name, 3, 1)
        function [v_muscle_pressures, label] = parse_bag_generic(bag_name, num_muscles, muscle_inflated)
            pressure = double(string(extractBetween(bag_name, "-", "psi")));
            
            v_muscle_pressures = zeros(num_muscles, 1);
            v_muscle_pressures(muscle_inflated) = 1;
            v_muscle_pressures = v_muscle_pressures * pressure;

            label = sprintf("%dpsi", pressure);
        end
    end

    % Copy constructor
    methods (Access = protected)
        % Copy constructor
        % Inherited from matlab.mixin.Copyable
        function cp = copyElement(obj)
            % Regular copy of all elements
            cp = copyElement@matlab.mixin.Copyable(obj);
            
            % Apply copy cstor for all handle children objects
            cp.arm_obj = copy(obj.arm_obj);
            cp.tags = copy(obj.tags);
        end
    end
end

