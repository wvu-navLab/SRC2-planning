bagselect2 = rosbag('2020-04-26-23-17-45.bag')
bSel = select(bagselect2,'Topic','/world/octomap');
bagmsg2 = readMessages(bSel);
map2 = readOccupancyMap3D(bagmsg2{2});

%%
X = -60.5:2:60.5;
Y = -60.5:2:60.5;
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
