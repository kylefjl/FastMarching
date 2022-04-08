
%%
% 快速行进平方法
% 此处对快速行进法进行大量简化，以及改进。若学习原理请参照官网的快速行进算法工具箱
%可改进之处 ：
%1.增加安全距离（安全阈值）参数，
% 只限学习交流
% 作者：2016011301 陈阔 B站分享学习心得 账号：废鹅吃柿子 微信15776629544
%%
clear all; clc;
close all;
n = 500;
addpath('fun','img');
name = 'map';
[M_static,W_static] = load_potential_map('map', n);%这个先不用在意 ？
%%
map_edge=bwperim((M_static));%查找边界
[row,column]=find(map_edge==1);%获取边界
start_points=[row';column'];%转置
%%
%start_pointstaic = start_pointstaics;
[D_staic,S] = perform_fast_marching(W_static, start_points);% 
U=rescale(D_staic);%归一化
% figure;
% subplot(2,1,1);
% imshow(D_staic);
% subplot(2,1,2);
% imshow(U);
%%
% display
U(W_static==0.001)=0;
U=rescale(U);
A = convert_distance_color(U);

%%
figure
clf; hold on; 
imageplot(A); axis image; axis off;

start_points2=[10;490];
[D_staic2,S] = perform_fast_marching(U, start_points2);
end_points = [490,10];
end_points = end_points';
paths = {};
i=1;
% figure;
% D_staic2(D_staic2==Inf) = max(max(D_staic2(D_staic2~=Inf)));
% imshow(rescale(D_staic2));title("D_staic2");

paths{i} = compute_geodesic(D_staic2,end_points(:,i));
if length(paths{i}(:))==2
    paths{i} = paths{i-1};
end
ms = 30; lw = 3;
A = convert_distance_color(D_staic2);

figure(2)
clf; hold on;%清除图窗 
imageplot(A); axis image; axis off;
h = plot( paths{i}(2,:), paths{i}(1,:), 'k' );
set(h, 'LineWidth', lw);
path{1}=paths;

saveas(gcf, [ 'result\',name '-geodesics.png'], 'png');
[x,y]=meshgrid(1:n,1:n);
figure(3);
Z=D_staic2;
%Z(W_static==0.001)=0;
mesh(x,y,Z);
%% 加载图片
function [M,W] = load_potential_map(name, n, options)
options.null = 0;
 file_name = [name, '.png'];
 M = rescale( double( im2bw(imread(file_name),0.9 ) ) );
 W = rescale(M)+0.001;
end
