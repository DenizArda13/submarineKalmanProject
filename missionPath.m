classdef missionPath
    properties
        L
        WP
        R_acc
    end

    methods
        function obj = missionPath(L,shape, R_acc)
            obj.Property1 = inputArg1 + inputArg2;
        end

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end