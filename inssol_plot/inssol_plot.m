close all
clear all
clc
%%
d2r = pi/180;
r2d = 180/pi;

data = load('..\inssol_pp.txt');

ins_t = data(:,1);
gps_t = data(:,2);

roll = data(:,3);
pitch = data(:,4);
yaw = data(:,5);

lat_ins = data(:,6);
lon_ins = data(:,7);
hgt_ins = data(:,8);

lat_gps = data(:,9);
lon_gps = data(:,10);
hgt_gps = data(:,11);
hd_gps = data(:,12);
vel = data(:,13);

flag = data(:,14);
%%
figure,plot(lat_ins,lon_ins,'.')
hold on
plot(lat_gps,lon_gps,'ro')
grid on
legend('ins','gps');

%---------------------------------------------
% gedectic llh to TM
%---------------------------------------------
[x_gps, y_gps] = Trans_TM(lat_gps, lon_gps);
[x_ins, y_ins] = Trans_TM(lat_ins, lon_ins);
S = shaperead('contour_tm.shp');
figure,
for i=1:length(S)
    plot(S(i).X(1,:),S(i).Y(1,:),'k.-')
    hold on
end
plot(y_gps, x_gps,'ro-')
plot(y_ins, x_ins,'b.-')
title('Trajectory');

figure,
plot(y_ins-y_ins(1),x_ins-x_ins(1),'b.')
hold on
plot(y_gps-y_ins(1),x_gps-x_ins(1),'ro')
title('Trajectory_-Zero base');

figure,plot(diff(x_ins),'b.-')
hold on
plot(diff(y_ins),'ro-')
title('Diff(POS)');

figure,plot(ins_t,yaw,'b.-')
hold on
plot(gps_t,hd_gps,'ro-')
title('Heading')