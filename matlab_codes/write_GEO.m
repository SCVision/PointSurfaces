function write_GEO(fn, geo_data, colAngles, rowAngles) 
% Function: write GEO file.
% Input:
%     fn - data file name (GEO format).
%     geo_data - geometric data (m*n*datumSize). 
%     colAngles - vertical angles theta (n*1).
%     rowAngles - horizontal angles phi (m*1). 
%     datumSize - size of one point
% 
% Writen by LIN, Jingyu (linjy02@hotmail.com), 20240312
%

% data info
colTotal = length(colAngles);
rowTotal = length(rowAngles);
datumSize = size(geo_data,2)/colTotal;

% construct data
M_title = zeros(1, colTotal * datumSize + 1);
M_title(1) = datumSize;
M_title(2) = colTotal;
M_title(3) = rowTotal;
M_title(3+(1:colTotal)) = colAngles;

M = [M_title; rowAngles geo_data];
dlmwrite(fn, M, 'precision', 9);
