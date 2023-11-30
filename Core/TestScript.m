volt = 3200
voltage = volt/1000;%伏特
            R = (3630/voltage -100)/1000;%kohms千欧姆
            pho = 1./R;%电导率
            %调用拟合公式进行拟合
            a = 4.241;
            b = 3.211;
            
            force = a*exp(b*pho)