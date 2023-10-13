classdef MatList<handle
   

    properties
        datacell,
        N
    end

    methods
        function obj = MatList()
            obj.datacell = {};
            obj.N = 0;
        end

        function addOne(obj,data)
            if obj.N >= length(obj.datacell)
                obj.datacell = [obj.datacell,cell(1,length(obj.datacell))];
            end
            obj.N = obj.N + 1;
            obj.datacell{obj.N} = data;
        end

        function r = toMat(obj)
            r = cell2mat(obj.datacell);
        end
    end
end