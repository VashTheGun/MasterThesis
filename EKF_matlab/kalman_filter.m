close all;
clear all;

%% TEST

% turning at 0.25 m/s controlled
uiimport('20170308_test_001.txt')
% turning at 0.50 m/s controlled
uiimport('20170308_test_002.txt')
% turning at 0.75 m/s controlled
uiimport('20170308_test_003.txt')
% turning at 1.00 m/s controlled
uiimport('20170308_test_004.txt')
% turning at 1.25 m/s controlled
uiimport('20170308_test_005.txt')
% turning at 1.50 m/s controlled
uiimport('20170308_test_006.txt')
% turning at 1.75 m/s controlled
uiimport('20170308_test_007.txt')
% turning at 2.00 m/s controlled
uiimport('20170308_test_008.txt')
% turning at 2.25 m/s controlled
uiimport('20170308_test_009.txt')


% circle at 0.50 m/s controlled
uiimport('20170308_test_014.txt')
% circle at 1.00 m/s controlled
uiimport('20170308_test_011.txt')
% circle at 1.5 m/s controlled
uiimport('20170308_test_012.txt')
% circle at 2.0 m/s controlled
uiimport('20170308_test_013.txt')

% straight at 1.00 m/s controlled
uiimport('20170308_test_015.txt')
% straight at 1.00 m/s controlled
uiimport('20170308_test_016.txt')
% straight at 1.50 m/s controlled
uiimport('20170308_test_017.txt')
% straight at 2.00 m/s controlled
uiimport('20170308_test_018.txt')


% changing lane at 1.00 m/s controlled
uiimport('20170308_test_020.txt')
% changing lane at 1.00 m/s controlled
uiimport('20170308_test_021.txt')
% changing lane at 1.50 m/s controlled
uiimport('20170308_test_022.txt')
% changing lane at 2.00 m/s controlled
uiimport('20170308_test_024.txt')


pause;
%% POSE t0

pose_t0 = 150*pi/180; %turning test
%pose_t0 = 185*pi/180; %straight
%pose_t0 = 0*pi/180; %lane changing


%% SENSOR FUSION



P(:,:,2) = [1000 0 0 0 0 0 0 0 0;...
            0 1000 0 0 0 0 0 0 0;...
            0 0 1000 0 0 0 0 0 0;...
            0 0 0 1000 0 0 0 0 0;...
            0 0 0 0 1000 0 0 0 0;...
            0 0 0 0 0 1000 0 0 0;...
            0 0 0 0 0 0 1000 0 0;...
            0 0 0 0 0 0 0 1000 0;...
            0 0 0 0 0 0 0 0 1000];    

H = [1 0 0 0 0 0 0 0 0;...
     0 1 0 0 0 0 0 0 0;...
     0 0 1 0 0 0 0 0 0;...
     0 0 0 1 0 0 0 0 0;...
     0 0 0 0 0 1 0 0 0;...
     0 0 0 0 0 0 1 0 0;...
     0 0 0 0 0 0 0 1 0];

Q = [0.00010 0 0 0 0 0 0 0 0;...
     0 0.0005 0 0 0 0 0 0 0;...
     0 0 0.001 0 0 0 0 0 0;...
     0 0 0 0.00010 0 0 0 0 0;...
     0 0 0 0 0.00005 0 0 0 0;...
     0 0 0 0 0 0.005 0 0 0;...
     0 0 0 0 0 0 0.005 0 0;...
     0 0 0 0 0 0 0 0.0001 0;...
     0 0 0 0 0 0 0 0 0.0001];

R = [0.05 0 0 0 0 0 0;...
     0 0.1 0 0 0 0 0;...
     0 0 0.3 0 0 0 0;...
     0 0 0 0.05 0 0 0;...
     0 0 0 0 0.3 0 0;...
     0 0 0 0 0 0.1 0;...
     0 0 0 0 0 0 0.0005];

I = [1 0 0 0 0 0 0 0 0;...
     0 1 0 0 0 0 0 0 0;...
     0 0 1 0 0 0 0 0 0;...
     0 0 0 1 0 0 0 0 0;...
     0 0 0 0 1 0 0 0 0;...
     0 0 0 0 0 1 0 0 0;...
     0 0 0 0 0 0 1 0 0;...
     0 0 0 0 0 0 0 1 0;...
     0 0 0 0 0 0 0 0 1];

for c=1:1:2;
u_enc(c) = (enc_fr(c) + enc_fl(c) + enc_rr(c) + enc_rl(c))/4*0.05;
%Head(c) = atan2((gps_y(c)-gps_y(c-1)),(gps_x(c)-gps_x(c-1)));
x(:,:,c) = [gps_x(c);u_enc(c);acc_x(c);gps_y(c);0;acc_y(c);yaw(c);gyro_z(c);pose_t0];
z(:,:,c) = [gps_x(c);u_enc(c);acc_x(c);gps_y(c);acc_y(c);yaw(c);gyro_z(c)];
x_imu(c) = gps_x(c);
y_imu(c) = gps_y(c);
vx_imu(c) = 0;
vy_imu(c) = 0;
vx_gps(c) = 0;
vy_gps(c) = 0;
v0_gps(c) = 0;
vvxx_new(c) = 0;
vvyy_new(c) = 0;
zero_val(c)=0;
prova_ax(c)=0;
prova_ay(c)=0;
vvxx(c)=0;
vvyy(c)=0;
end



n = length(acc_x);
for c=3:1:n;
    
    E3(:,:,c) = [1 0 0; 0 1 0; 0 0 1];
    E1(:,:,c) = [1 0 0; 0 cos(roll(c)) sin(roll(c)); 0 -sin(roll(c)) cos(roll(c))];
    E2(:,:,c) = [cos(pitch(c)) 0 sin(pitch(c)); 0 1 0; -sin(pitch(c)) 0 cos(pitch(c))];
    
    omega_cut = 20;
    dt = (time(c)-time(c-1));
    alpha(c)  = (omega_cut * dt/1000) / (1+ omega_cut * dt/1000);
    
    
    dt=time_imu(c); 
    
    %longitudinal velocity from encoders
    u_enc(c) = (enc_fr(c) + enc_fl(c))/2*0.05;

    %aceleration
    acc_vec(:,:,c) = [acc_x(c);...
                      acc_y(c);...
                      acc_z(c)];

  
    %KALMAN FILTER
        A(:,:,c) = [x(1,1,c-1)+x(2,1,c-1)*dt*cos(x(9,1,c-1))-x(5,1,c-1)*dt*sin(x(9,1,c-1));...
                    x(2,1,c-1)+x(3,1,c-1)*dt+x(8,1,c-1)*x(5,1,c-1)*dt;...
                    x(3,1,c-1);...
                    x(4,1,c-1)+x(2,1,c-1)*sin(x(9,1,c-1))*dt+x(5,1,c-1)*dt*cos(x(9,1,c-1));...
                    x(5,1,c-1)+x(6,1,c-1)*dt-x(8,1,c-1)*x(2,1,c-1)*dt;...
                    x(6,1,c-1);
                    x(7,1,c-1)+x(8,1,c-1)*dt;...
                    x(8,1,c-1);...
                    x(9,1,c-1)+x(8,1,c-1)*dt];


        F(:,:,c) = [1 dt*cos(x(9,1,c-1)) 0 0 -dt*sin(x(9,1,c-1)) 0 0 0 -dt*(x(5,1,c-1)*cos(x(9,1,c-1))+x(2,1,c-1)*sin(x(9,1,c-1)));...
                    0 1 dt 0 dt*x(8,1,c-1) 0 0 dt*x(5,1,c-1) 0;...
                    0 0 1 0 0 0 0 0 0;...
                    0 dt*sin(x(9,1,c-1)) 0 1 dt*cos(x(9,1,c-1)) 0 0 0 dt*(x(2,1,c-1)*cos(x(9,1,c-1))-x(5,1,c-1)*sin(x(9,1,c-1)));...
                    0 -dt*x(8,1,c-1) 0 0 1 dt 0 -dt*x(2,1,c-1) 0;...
                    0 0 0 0 0 1 0 0 0;
                    0 0 0 0 0 0 1 dt 0;...
                    0 0 0 0 0 0 0 1 0;...
                    0 0 0 0 0 0 0 dt 1];

        
        
        X_hat(:,:,c) = A(:,:,c);
        
        acc_vec_x(c)    = acc_vec(1,1,c);
        acc_vec_y(c)    = acc_vec(2,1,c);
        
        %measure vector
        z(:,:,c) = [gps_x(c);u_enc(c);acc_vec_x(c);gps_y(c);acc_vec_y(c);yaw(c);gyro_z(c)];
     
        p(:,:,c) = F(:,:,c)*P(:,:,c-1)*F(:,:,c)' + Q;
        S(:,:,c) = (H*p(:,:,c)*H'+R);
        K(:,:,c) = p(:,:,c)*H'*S(:,:,c)^(-1);

        %state vector
        x(:,:,c) = X_hat(:,:,c)+K(:,:,c)*((z(:,:,c)-H*X_hat(:,:,c)));
        P(:,:,c) = p(:,:,c)-K(:,:,c)*S(:,:,c)*K(:,:,c)';

        
    x_new(c)        = x(1,1,c);
    y_new(c)        = x(4,1,c);
    yaw_new(c)      = x(7,1,c);
    yaw_rate_new(c) = x(8,1,c);
    theta_new(c)    = x(9,1,c);
    
    if x(2,1,c) < 0.05
        vx_new(c) = 0;
        ax_new(c) = 0;
        vy_new(c) = 0;
        ay_new(c) = 0;
        beta(c)   = 0;
    else
        vx_new(c) = x(2,1,c);
        ax_new(c) = x(3,1,c);
        vy_new(c) = -x(5,1,c);
        ay_new(c) = x(6,1,c);
        beta(c)   = (atan2(vy_new(c),vx_new(c)));
    end
    
    %head angle
    Head(c) = atan2((x(4,1,c)-x(4,1,c-1)),(x(1,1,c)-x(1,1,c-1)));
    
    %vehicle velocity
    v0(c)   = (sqrt(vx_new(c)^2+vy_new(c)^2));

        
end



%% PLOT DATA

figure(1);
subplot(6,1,1)
plot(time,vx_new,time,vy_new)
title('vehicle velocity')
xlabel('[millisec]')
ylabel('[m/s]')
legend('vx','vy')
subplot(6,1,2)
plot(time, ax_new)
title('acc_x')
xlabel('[millisec]')
ylabel('[m/s^2]')
subplot(6,1,3)
plot(time,ay_new)
title('acc_y')
xlabel('[millisec]')
ylabel('[m/s^2]')
subplot(6,1,4)
plot(time,yaw_rate_new)
title('yaw_rate')
xlabel('[millisec]')
ylabel('[rad/sec]')
subplot(6,1,6)
plot(time,steering,time,traction)
title('steering & traction')
xlabel('[millisec]')
ylabel('[%]')
legend('steering','traction')
ylim([-1.2 1.2])
subplot(6,1,5);
plot(time,beta*180/pi);
title('beta');
xlabel('[millisec]')
ylabel('[rad]')