% Show data in 'Data'

%% load data 
fn = 'test_data/planes_05.L3D';
[rData, angleV, angleH, timestamp, datumSize, ...
    La, Lx, Dpsi, Dtheta, Dgamma] = read_L3D(fn); 

dist = rData(:,1:datumSize:end);
intensity = rData(:,2:datumSize:end);

%% show raw data
ps = dist2points(rData, angleV, angleH, La, Lx, Dpsi, Dtheta, Dgamma);

figure(1); 
scatter3(ps(:,1),ps(:,2),ps(:,3),1);
