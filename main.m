
%%
% 快速行进平方法
% 此处对快速行进法进行大量简化，以及改进。若学习原理请参照官网的快速行进算法工具箱
%可改进之处 ：
%1.增加安全距离（安全阈值）参数，
% 只限学习交流
%%
clear all; clc;
close all;
n = 500;
addpath('img');
name = 'map';
[M_static,W_static] = load_potential_map('map', n);%这个先不用在意 ？
%%
figure
imshow(M_static);
title("map");

map_edge=bwperim((M_static));%查找边界
[row,column]=find(map_edge==1);%获取边界
start_points=[row';column'];%转置
%%
%start_pointstaic = start_pointstaics;
[D_staic,S] = perform_fast_marching(W_static, start_points);% 

U=rescale(D_staic);%归一化
%%
% display
U(W_static==0.001)=0;
U=rescale(U);
A = convert_distance_color(U);


start_points2=[10;490];
[D_staic2,S] = perform_fast_marching(U, start_points2);
end_points = [490,10];
end_points = end_points';
paths = {};
i=1;



paths{i} = compute_geodesic(D_staic2,end_points(:,i));
if length(paths{i}(:))==2
    paths{i} = paths{i-1};
end
A = convert_distance_color(D_staic2);

figure
clf; %清除图窗 
imageplot(A); title("distance_color");
axis image; axis off;
hold on;
h = plot( paths{i}(2,:), paths{i}(1,:), 'k' );
set(h, 'LineWidth', 3);
path{1}=paths;

saveas(gcf, [ 'result\',name '-geodesics.png'], 'png');
[x,y]=meshgrid(1:n,1:n);
figure
Z=D_staic2;
%Z(W_static==0.001)=0;
mesh(x,y,A(:,:,1)');
view(90,80);
%% 加载图片
function [M,W] = load_potential_map(name, n, options)
options.null = 0;
 file_name = [name, '.png'];
 M = rescale( double( im2bw(imread(file_name),0.9 ) ) );
 W = rescale(M)+0.001;
end
