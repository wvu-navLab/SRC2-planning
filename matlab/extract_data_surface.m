bagselect2 = rosbag('2020-04-26-23-17-45.bag')
bSel = select(bagselect2,'Topic','/world/octomap');
bagmsg2 = readMessages(bSel);
map2 = readOccupancyMap3D(bagmsg2{2});

%%
X = -70.5:0.2:70.5;
Y = -60.5:0.2:60.5;
Z = [];
for a=X
    z = [];
    for b=Y
        z_ = [];    
        for i=1:45
            if(map2.checkOccupancy([a,b,-4.1+i*0.2]))
                z_ = [z_,-4.1+i*0.2];
            end
        end
        z = [z; max(z_)];
    end
    Z = [Z, z];
end

%%
[dfdx,dfdy] = gradient(Z,0.2);
slope = hypot(dfdx, dfdy);

save('map_src2_updated.mat')


fun = @(block_struct) mean(block_struct.data(:));
SLOPE = blockproc(slope, [12 12], fun);

X_REDUCED = blockproc(X, [12 12], fun);
Y_REDUCED = blockproc(Y, [12 12], fun);

fid = fopen('X_REDUCED.txt','wt');
for ii = 1:size(X_REDUCED,1)
    fprintf(fid,'%g\t',X_REDUCED(ii,:));
    fprintf(fid,'\n');
end
fclose(fid);