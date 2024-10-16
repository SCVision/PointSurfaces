% save geo file

%% load data
fn = 'planes_05.GEO';
[geo_data, colAngles, rowAngles, datumSize] = read_GEO(fn);

if datumSize < 8
    disp 'invalid data'
    return;
end

%% save data
precision = 9;
write_GEO("test.geo", geo_data, colAngles, rowAngles, precision);

