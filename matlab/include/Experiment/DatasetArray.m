classdef DatasetArray
    %DATASETARRAY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        datasets
        measurements
        tab_measurements
    end
    
    methods
        function obj = DatasetArray(v_input)
            arguments
                v_input
            end
            
            obj.datasets = Dataset.empty(0, length(v_input));

            for i = 1 : length(v_input)
                obj = obj.add_dataset(v_input(i));
            end
        end

        function obj = add_dataset(obj, dataset)
            if strcmp(class(dataset), "DatasetParams")

            elseif strcmp(class(dataset), "Dataset")
                
            else
                error("Invalid input type to DatasetArray cstor")
            end
        end
    end
end

