function pntcloud = dist2points(rData, angleV, angleH, ...
    La, Lx, Dpsi, Dtheta, Dgamma) 
% Function: convert distance array to point sets.
% Input:
%     rData - LiDAR data (H*V*szData). 
%     angleV - vertical angles theta (V*1).
%     angleH - horizontal angles phi (H*1). 
%     La, Lx, Dpsi, Dtheta, Dgamma - 3D LiDAR parameters
% Output:
%     pntcloud - x, y, z and intensity of points ((H*V) * 4)
% 
% Writen by LIN, Jingyu (linjy02@hotmail.com), 20210428
% Revised by LIN, Jingyu (linjy02@hotmail.com), 20230429
% Revised by LIN, Jingyu (linjy02@hotmail.com), 20230507
%

% preprocessing
angleV = angleV(:)' + Dtheta; % first rotation
SV = sind(angleV); % row vector
CV = cosd(angleV); % row vector
% Yrot = [cosd(Dphi) 0 sind(Dphi); 0 1 0; -sind(Dphi) 0 cosd(Dphi)];
% Zrot = [cosd(Dpsi) -sind(Dpsi) 0; sind(Dpsi) cosd(Dpsi) 0; 0 0 1];
% Rot = Yrot*Zrot; % rotation from deviation angle
% Zrot = [cosd(Dphi) -sind(Dphi) 0; sind(Dphi) cosd(Dphi) 0; 0 0 1];
% Xrot = [1 0 0; 0 cosd(Dpsi) -sind(Dpsi); 0 sind(Dpsi) cosd(Dpsi)];
% Rot = Zrot*Xrot; % rotation from deviation angle
Rot = [1 0 0; 0 cosd(Dpsi) -sind(Dpsi); 0 sind(Dpsi) cosd(Dpsi)];
Cgamma = cosd(Dgamma); Sgamma = sind(Dgamma);

% prepare for transform
% [H,V] = size(rData);
H = length(angleH);
V = length(angleV);
datumSize = size(rData,2)/V;
x = zeros(H,V);
y = zeros(H,V);
z = zeros(H,V);
SH = sind(angleH); 
CH = cosd(angleH); 
for i = 1:H % for each scanning plane
    % polar coordinates to Cartesian coordinates
    dist = rData(i,1:datumSize:end);
    rc = dist*Cgamma;
    yL_hat = -dist*Sgamma;
    xL_hat = rc.*SV;
    zL_hat = rc.*CV;
    
    % scanning plane to LIDAR coordinates
    x_tilde = Rot*[xL_hat;yL_hat;zL_hat]; % rotation from deviation angle
    x_tilde(1,:) = x_tilde(1,:) + Lx; % translation
    x_tilde(2,:) = x_tilde(2,:) + La;  % translation
    x(i,:) =  x_tilde(1,:)*CH(i)+ x_tilde(2,:)*SH(i); % Zrot(-phi)
    y(i,:) = -x_tilde(1,:)*SH(i)+ x_tilde(2,:)*CH(i);
    z(i,:) =  x_tilde(3,:);
end
x=x';y=y';z=z';
if datumSize > 1
    c = rData(:,2:datumSize:end);
else
    c = ones(H,V);
end
pntcloud = [x(:), y(:), z(:), c(:)];
