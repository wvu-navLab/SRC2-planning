
load('map_src2.mat');
%hist(slope(:));
fileID = fopen('cost2goal.txt','r');
formatspec = '%d';
sizeA = [606,606];
A = fscanf(fileID,formatspec,sizeA);
figure();
imagesc(A);
hold on;
[i,j] = ind2sub([606,606],352651);
plot(j,i,'ws');
start = [200,300];
goal = [j,i];
current = start;
path = [];

            


% x = 1:1:606;
% y = x';
% [px,py] = gradient(A);
% q = quiver(x,y,px,py);
% q.Color = 'white';

figure();
imagesc(slope);






