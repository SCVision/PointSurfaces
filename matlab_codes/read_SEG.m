function [seg_map, seg_feature] = read_SEG(fn) 
% Function: read SEG file related to a GEO file.
% Input:
%     fn - data file name (SEG format).
% Output:
%     seg_map - segment map (rowTotal*colTotal). 
%     seg_feature - segment features (lenFeature*nSeg).
% Note: a SEG file should be used with its GEO file.
%
% Writen by LIN, Jingyu (linjy02@hotmail.com), 2024.3
%

rawdata = importdata(fn);

% resolve title row
colTotal = rawdata(1,1);
rowTotal = rawdata(1,2);
nSeg = rawdata(1,3);
lenFeature = rawdata(1,4);
colFeature = floor(size(rawdata,2)/lenFeature) * lenFeature;

% resolve map rows
seg_map = rawdata(2:(rowTotal+1),1:colTotal);

% resolve features rows
% features = rawdata((rowTotal+2):end,1:(colFeature))';
% features = features(1:lenFeature*nSeg);
% seg_feature = reshape(features, lenFeature, nSeg)';
features = rawdata((rowTotal+2):end,:)';
features = features(1:lenFeature*nSeg);
seg_feature = reshape(features, lenFeature, nSeg)';