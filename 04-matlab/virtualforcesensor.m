clc;
clear all;
close all;
data_file = "data_file_04-18-19_21-29-11"  

path = "/Users/elenakern/university/masters/sai-2/apps/inertial_parameter_application/data_collection/simulation/stepresponse/"

HeaderLines = 3;
T = readtable(path+data_file, 'HeaderLines',3);
data_val = cell2mat(table2cell(T(:, 1:13)));
timestamp = data_val(:,1:1);
q = data_val(:,2:7);
dq = data_val(:,8:13);
figure
ax1 = subplot(2,2,1); % top subplot
plot(ax1,timestamp,q(:,1:3))
title(ax1,'Virtual prismatic joints')
ylabel(ax1,'prismatic joints displacements in m')

ax2 = subplot(2,2,2); % top subplot
plot(ax2,timestamp,q(:,4:6))
title(ax2,'Virtual revolute joints')
ylabel(ax2,'revolute joints displacements in rad')

ax3 = subplot(2,2,3); % top subplot
plot(ax3,timestamp,dq(:,1:3))
title(ax3,'Virtual prismatic joints')
ylabel(ax3,'prismatic joints velocities in m/s')

ax4 = subplot(2,2,4); % top subplot
plot(ax4,timestamp,dq(:,4:6))
title(ax4,'Virtual revolute joints')
ylabel(ax4,'revolute joints velocities in rad/s')
% plot(timestamp , q(:,1:3))
% plot(timestamp , q(:,4:7))
% plot(timestamp , dq(:,1:3))
% plot(timestamp , dq(:,4:7))


t_x = q(:,1);
t_y = q(:,2);
t_z = q(:,3);
q_x = q(:,4);
q_y = q(:,5);
q_z = q(:,6);

% in_1 = [zeros(3,1) ;ones(size(t_x),1)]
input = ones(size(t_x));
sys_t_x = tfestimate(input, t_x);
figure;
tfestimate(input, t_x);
np = 2;
nz = 0;
iodelay = 2;
Ts = 0.0005;
in_all = [zeros(np+2,1); ones(size(t_x))];
out_1 = [zeros(np+2,1); t_x];
out_2 = [zeros(np+2,1); t_y];
out_3 = [zeros(np+2,1); t_z];
out_4 = [zeros(np+2,1); q_x];
out_5 = [zeros(np+2,1); q_y];
out_6 = [zeros(np+2,1); q_z];
data_1 = iddata(out_1,in_all,Ts);
data_2 = iddata(out_2,in_all,Ts);
data_3 = iddata(out_3,in_all,Ts);
data_4 = iddata(out_4,in_all,Ts);
data_5 = iddata(out_5,in_all,Ts);
data_6 = iddata(out_6,in_all,Ts);
sysd_1 = tfest(data_1,np,nz,iodelay,'Ts',Ts)
sysd_2 = tfest(data_2,np,nz,iodelay,'Ts',Ts)
sysd_3 = tfest(data_3,np,nz,iodelay,'Ts',Ts)
sysd_4 = tfest(data_4,np,nz,iodelay,'Ts',Ts)
sysd_5 = tfest(data_5,np,nz,iodelay,'Ts',Ts)
sysd_6 = tfest(data_6,np,nz,iodelay,'Ts',Ts)

figure 
ax1 = subplot(3,3,1); % top subplot
plot(ax1,timestamp,q(:,1))
hold on;
plot(ax1, step(sysd_1))
title(ax1,'Virtual prismatic joint x')
ylabel(ax1,'prismatic joints displacement in m')
legend("measured","stepresponse tf estimate" )
ax2 = subplot(3,3,2); % top subplot
plot(ax2,timestamp,q(:,2))
hold on;

plot(ax2, step(sysd_2))
title(ax2,'Virtual prismatic joint y')
ylabel(ax2,'prismatic joints displacement in m')
legend("measured","stepresponse tf estimate" )


ax3 = subplot(3,3,3); % top subplot
plot(ax3,timestamp,q(:,3))
hold on;

plot(ax3,step(sysd_3))
title(ax3,'Virtual prismatic joint z')
ylabel(ax3,'prismatic joints displacement in m')
legend("measured","stepresponse tf estimate" )


ax4 = subplot(3,3,4); % top subplot
plot(ax4,timestamp,q(:,4))
hold on;

plot(ax4,step(sysd_4))
title(ax4,'Virtual revolute joint x')
ylabel(ax4,'revolute joints angle in rad')
legend("measured","stepresponse tf estimate" )


ax5 = subplot(3,3,5); % top subplot
plot(ax5,timestamp,q(:,5))
hold on;

plot(ax5,step(sysd_5))
title(ax5,'Virtual revolute joint y')
ylabel(ax5,'revolute joints angle in rad')
legend("measured","stepresponse tf estimate" )

ax6 = subplot(3,3,6); % top subplot
plot(ax6,timestamp,q(:,6))
hold on;
plot(ax6,step(sysd_6))
title(ax6,'Virtual revolute joint z')
ylabel(ax6,'revolute joints angle in rad')
legend("measured","stepresponse tf estimate" )
