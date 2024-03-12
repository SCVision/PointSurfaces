function [rData, angleV, angleH, timestamp, datumSize, ...
   La, Lx, Dpsi, Dtheta, Dgamma] = read_L3D(fn) 
% Function: load L3D file.
% Input:
%     fn - data file name (L3D format).
% Output:
%     rData - range data (H*V*datumSize). 
%     angleV - vertical angles theta (V*1).
%     angleH - horizontal angles phi (H*1). 
%     timestamp - timestamp of rows (H*1)
%     datumSize - size of one point
%     La, Lx, Dpsi, Dtheta, Dgamma - 3D scanner parameters
% 
% Writen by LIN, Jingyu (linjy02@hotmail.com), 20200127
% Revised by LIN, Jingyu (linjy02@hotmail.com), 20230429
% Revised by LIN, Jingyu (linjy02@hotmail.com), 20230507
%

rawdata = importdata(fn);
cols = size(rawdata, 2); % total columns

% resolve title row
datumSize = rawdata(1,1);
V = rawdata(1,2);
H = rawdata(1,3);
angleV = rawdata(1, 3+(1:V))';
La = 0; Lx = 0; Dpsi=0; Dtheta=0; Dgamma=0;
if cols >= 3+V+5
    La = rawdata(1,3+V+1);
    Lx = rawdata(1,3+V+2);
    Dpsi = rawdata(1,3+V+3);
    Dtheta = rawdata(1,3+V+4);
    Dgamma = rawdata(1,3+V+5);
end

% resolve data rows
timestamp = rawdata(2:end, 2);
angleH = rawdata(2:end, 3);
rData = rawdata(2:end, 3+(1:datumSize*V));

if length(angleH) ~= H
    disp 'rows error'
end
