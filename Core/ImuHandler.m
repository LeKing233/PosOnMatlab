classdef ImuHandler <handle
    %IMUHANDLER Imu事务管理类
    %   封装Imu数据导入导出、计算结果绘图
    
    properties
        mFilePath;%文件地址
        mRawData;%原始数据
    end
    
    methods
        function obj = ImuHandler(filePath)
          obj.mFilePath = filePath;
          obj.extractData();
        end
        
        %提取数据
        function extractData(obj)
            obj.mRawData = readtable(obj.mFilePath);
        end
        


        %绘图函数
        
    end
end

