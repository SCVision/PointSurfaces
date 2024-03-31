% load GEO and SEG file

%% file name without suffix
fn = 'test_data/lab1'; 

%% load GEO data
[geo_data, colAngles, rowAngles, datumSize] = read_GEO([fn '.GEO']);
if datumSize < 27
    disp 'invalid GEO data'
    return;
end
distance = geo_data(:,1:datumSize:end);
intensity = geo_data(:,2:datumSize:end);
brightness = geo_data(:,3:datumSize:end);
p_x = geo_data(:,4:datumSize:end);
p_y = geo_data(:,5:datumSize:end);
p_z = geo_data(:,6:datumSize:end);
density = geo_data(:,7:datumSize:end);
slope = geo_data(:,8:datumSize:end);
n_x = geo_data(:,9:datumSize:end);
n_y = geo_data(:,10:datumSize:end);
n_z = geo_data(:,11:datumSize:end);
pc1 = geo_data(:,12:datumSize:end);
pc2 = geo_data(:,13:datumSize:end);
pc_x = geo_data(:,14:datumSize:end);
pc_y = geo_data(:,15:datumSize:end);
pc_z = geo_data(:,16:datumSize:end);
belief = geo_data(:,17:datumSize:end);
dx_col = geo_data(:,18:datumSize:end);
dy_col = geo_data(:,19:datumSize:end);
dz_col = geo_data(:,20:datumSize:end);
dx_row = geo_data(:,21:datumSize:end);
dy_row = geo_data(:,22:datumSize:end);
dz_row = geo_data(:,23:datumSize:end);
dVol = geo_data(:,24:datumSize:end);
h11 = geo_data(:,25:datumSize:end);
h12 = geo_data(:,26:datumSize:end);
h22 = geo_data(:,27:datumSize:end);

%% load SEG data
[seg_map, seg_feature] = read_SEG([fn '.SEG']);
[rowTotal, colTotal] = size(seg_map);
if colTotal ~= length(colAngles) || rowTotal ~= length(rowAngles) 
    disp 'GEO and SEG mismatch'
    return;
end
[nSeg, lenFeature] = size(seg_feature);
if lenFeature < 12 
    disp 'invalid SEG data'
    return;
end
idSeg = seg_feature(:,1);
nPnts = seg_feature(:,2);
c_x = seg_feature(:,3);
c_y = seg_feature(:,4);
c_z = seg_feature(:,5);
n_x = seg_feature(:,6);
n_y = seg_feature(:,7);
n_z = seg_feature(:,8);
nBar_x = seg_feature(:,9);
nBar_y = seg_feature(:,10);
nBar_z = seg_feature(:,11);
area = seg_feature(:,12);

%% show segment and its normal
n = 2; % selected id

% segment
idx = (seg_map == n);
figure(1); 
scatter3(p_x(idx),p_y(idx),p_z(idx),1);

% normal
xn1 = [c_x(n) c_x(n)+n_x(n)];
yn1 = [c_y(n) c_y(n)+n_y(n)];
zn1 = [c_z(n) c_z(n)+n_z(n)];
xn2 = [c_x(n) c_x(n)+nBar_x(n)];
yn2 = [c_y(n) c_y(n)+nBar_y(n)];
zn2 = [c_z(n) c_z(n)+nBar_z(n)];
figure(1); hold on
plot3(xn1,yn1,zn1,'r',xn2,yn2,zn2,'b')
hold off
axis equal % the same units along each axis.
