load('map_src2.mat')

[x,y] = meshgrid(X,Y);

origin = [302,302];

goal = [500,50];

map = true(606,606);
map(2:end-1,2:end-1) = false(604,604);

route = DijkstraTorus (map, dfdx, dfdy, origin, goal);

figure('Color','w');
mapshow(X,Y,Z,'DisplayType','surface');view(3);shg;
hold on 
plot3(x(route),y(route),Z(route), 'r', 'LineWidth',2.0);