
%%
% �����н�ƽ����
% �˴��Կ����н������д����򻯣��Լ��Ľ�����ѧϰԭ������չ����Ŀ����н��㷨������
%�ɸĽ�֮�� ��
%1.���Ӱ�ȫ���루��ȫ��ֵ��������
% ֻ��ѧϰ����
%%
clear all; clc;
close all;
n = 500;
addpath('img');
name = 'map';
[M_static,W_static] = load_potential_map('map', n);%����Ȳ������� ��
%%
figure
imshow(M_static);
title("map");

map_edge=bwperim((M_static));%���ұ߽�
[row,column]=find(map_edge==1);%��ȡ�߽�
start_points=[row';column'];%ת��
%%
%start_pointstaic = start_pointstaics;
[D_staic,S] = perform_fast_marching(W_static, start_points);% 

U=rescale(D_staic);%��һ��
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
clf; %���ͼ�� 
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
%% ����ͼƬ
function [M,W] = load_potential_map(name, n, options)
options.null = 0;
 file_name = [name, '.png'];
 M = rescale( double( im2bw(imread(file_name),0.9 ) ) );
 W = rescale(M)+0.001;
end
