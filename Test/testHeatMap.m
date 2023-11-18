clc;clear;close all;
% 定义点(x,y,z)
x			= randn(50,1);
xmax 	= max(x);
xmin 	= min(x);
y			= randn(50,1);
ymax 	= max(y);
ymin 	= min(y);
z = exp(cos(x.^2)) + exp(cos(y.^2));
N = 500; % 每个维度的数据点数
% 网格化x,y二维空间
[X,Y] = meshgrid(linspace(xmin,xmax,N),linspace(ymin,ymax,N));
% 采用插值法扩展数据，可用方法有'linear'(default)|'nearest'|'natural'|'cubic'|'v4'|
Z = griddata(x,y,z,X,Y,'v4');

%% 等高线法
figure('NumberTitle','off','Name','等高线法','Color','w','MenuBar','none','ToolBar','none');
contourf(X,Y,Z,N, 'LineColor','none');
colormap('jet');
colorbar;
axis off;

%% 投影图法
figure('NumberTitle','off','Name','投影图法','Color','w','MenuBar','none','ToolBar','none');
surf(X,Y,Z,'LineStyle','none');
xlim([min(X(:)) max(X(:))]);
ylim([min(Y(:)) max(Y(:))]);
axis off;
colormap('jet');
colorbar;
shading interp;
view(0,90);

%% imagesc法
figure('NumberTitle','off','Name','imagesc法','Color','w','MenuBar','none','ToolBar','none');
% 因为图像坐标和笛卡尔坐标起始位置不一样，需要上下翻转
imagesc(flipud(Z));
colormap('jet');
colorbar;
axis off;

%% pcolor法
figure('NumberTitle','off','Name','pcolor法','Color','w','MenuBar','none','ToolBar','none');
pcolor(X,Y,Z);
colormap('jet');
colorbar;
shading interp;
axis off;
