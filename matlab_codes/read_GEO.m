function [geo_data, colAngles, rowAngles, datumSize] = read_GEO(fn) 
% Function: load GEO file.
% Input:
%     fn - data file name (GEO format).
% Output:
%     geo_data - geometric data (m*n*datumSize). 
%     colAngles - vertical angles theta (n*1).
%     rowAngles - horizontal angles phi (m*1). 
%     datumSize - size of one point
% 
% Writen by LIN, Jingyu (linjy02@hotmail.com), 20240312
%

rawdata = importdata(fn);

% resolve title row
datumSize = rawdata(1,1);
colTotal = rawdata(1,2);
rowTotal = rawdata(1,3);
colAngles = rawdata(1,3+(1:colTotal));
rowAngles = rawdata(2:end,1);

% resolve data rows
geo_data = rawdata(2:end,2:end);

if length(rowAngles) ~= rowTotal
    disp 'rows error'
end
