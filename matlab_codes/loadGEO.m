% load geo file

%% load data
fn = '../data_L3D/planes_05.GEO';
[geo_data, colAngles, rowAngles, datumSize] = read_GEO(fn);

if datumSize < 8
    disp 'invalid data'
    return;
elseif datumSize < 26   % no geometric information
    distance = geo_data(:,1:datumSize:end);
    intensity = geo_data(:,2:datumSize:end);
    brightness = geo_data(:,3:datumSize:end);
    p_x = geo_data(:,4:datumSize:end);
    p_y = geo_data(:,5:datumSize:end);
    p_z = geo_data(:,6:datumSize:end);
    density = geo_data(:,7:datumSize:end);
    slope = geo_data(:,8:datumSize:end);
    % fill with zeros
    n_x = zeros(length(rowAngles), length(colAngles));
    n_y = zeros(length(rowAngles), length(colAngles));
    n_z = zeros(length(rowAngles), length(colAngles));
    pc1 = zeros(length(rowAngles), length(colAngles));
    pc2 = zeros(length(rowAngles), length(colAngles));
    pc_x = zeros(length(rowAngles), length(colAngles));
    pc_y = zeros(length(rowAngles), length(colAngles));
    pc_z = zeros(length(rowAngles), length(colAngles));
    h11 = zeros(length(rowAngles), length(colAngles));
    h12 = zeros(length(rowAngles), length(colAngles));
    h22 = zeros(length(rowAngles), length(colAngles));
    belief = zeros(length(rowAngles), length(colAngles));
    dx_col = zeros(length(rowAngles), length(colAngles));
    dy_col = zeros(length(rowAngles), length(colAngles));
    dz_col = zeros(length(rowAngles), length(colAngles));
    dx_row = zeros(length(rowAngles), length(colAngles));
    dy_row = zeros(length(rowAngles), length(colAngles));
    dz_row = zeros(length(rowAngles), length(colAngles));
else % datumSize >= 26
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
    h11 = geo_data(:,17:datumSize:end);
    h12 = geo_data(:,18:datumSize:end);
    h22 = geo_data(:,19:datumSize:end);
    belief = geo_data(:,20:datumSize:end);
    dx_col = geo_data(:,21:datumSize:end);
    dy_col = geo_data(:,22:datumSize:end);
    dz_col = geo_data(:,23:datumSize:end);
    dx_row = geo_data(:,24:datumSize:end);
    dy_row = geo_data(:,25:datumSize:end);
    dz_row = geo_data(:,26:datumSize:end);
end

%% show 3D
figure(2); 
scatter3(p_x(:),p_y(:),p_z(:),1);

