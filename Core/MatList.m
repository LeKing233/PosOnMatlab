classdef MatList<handle
    properties(Access = private)
        dataCell,%数据元胞
        N %最末尾数组位置
        extendScale = 2;%单次拓展倍数
    end

    methods
        function obj = MatList()
            obj.dataCell = {};
            obj.N = 0;
        end

        function addOne(obj,data)
            %指数扩容
            if obj.N >= length(obj.dataCell)
                obj.dataCell = [obj.dataCell;cell((obj.extendScale-1)*length(obj.dataCell),1)];
            end
            %添加数据
            obj.N = obj.N + 1;
            obj.dataCell{obj.N} = data;
        end

        function r = toMat(obj)
            r = cell2mat(obj.dataCell);
        end
    end
end