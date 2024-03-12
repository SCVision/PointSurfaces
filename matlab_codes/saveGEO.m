% save geo file

%% load data
fn = '../data_L3D/planes_05.GEO';
[geo_data, colAngles, rowAngles, datumSize] = read_GEO(fn);

if datumSize < 8
    disp 'invalid data'
    return;
end

%% save data
write_GEO("test.geo", geo_data, colAngles, rowAngles);
