%%
clear all
clc
%% Costanti
R = 0.0325;         % [m] Raggio della ruota
m_wa = 0.026*2;       % [kg] Massa della ruota assemblata
%J_w = 0.0000241;    % [kg*m^2] Momento di inerzia della ruota assemblata
J_w = 0.0000211;    % [kg*m^2] Momento di inerzia della ruota assemblata
l = 0.05;           % [m] Distanza tra centro ruota e centro massa del corpo
%m_b = 0.660;        % [kg] Massa del corpo
m_b = 0.648;        % [kg] Massa del corpo
%J_b = 0.001304;     % [km*m^2] Momento di inerzia del corpo
J_b = 0.0003421;     % [km*m^2] Momento di inerzia del corpo
g = 9.81;           % [m/s^2]

%% Coefficienti
M = m_b + 2*(m_wa+(J_w/R^2)) - ( (m_b*l)^2/((m_b*l^2)+J_b) );
J = J_b + m_b*l^2 - ( (m_b*l)^2/(m_b+2*(m_wa+J_w/R^2)) );

C1 = 1/M * ( g*(m_b*l)^2/((m_b*l^2)+J_b) );
C2 = 2/M * ( 1/R + (m_b*l)/((m_b*l^2)+J_b) );
C3 = m_b*g*l/J;
C4 = 2/(J*R) * ( R+ (m_b*l)/(m_b+2*(m_wa+(J_w/R^2))) );

%% Open-Loop Transfer Function
s=tf('s');
TF_ol = (-s*R*C4)/(C2*s^2 + C1*C4 - C2*C3)

poles_ol = pole(TF_ol)
figure(1)
subplot(1,3,1);
step(TF_ol)

%% Open-Loop Discrete Transfer Function
TF_ol_d = c2d(TF_ol,0.001,'tustin')

%% Closed-Loop Transfer Function (without PID)
TF_cl_woutPID = feedback(TF_ol, 1)
poles_cl = pole(TF_cl_woutPID)
subplot(1,3,2);
step(TF_cl_woutPID)

%% PID controller
%Tuning dei valori tramite PID Tuner
Kp = -2640.80941826754;
Ki = -809301.124872752;
Kd = -0.289054575787711;

PID = pid(Kp, Ki, Kd);

%% Closed-Loop Transfer Function (with PID)
TF_cl_PID = feedback(TF_ol*PID, 1)
poles_cl_PID = pole(TF_cl_PID)
subplot(1,3,3);
step(TF_cl_PID)

%% Closed-Loop Transfer Function (with PID_generated)
% TF_cl_PID = feedback(TF_ol*C, 1)
% poles_cl_PID = pole(TF_cl_PID)
% figure(5)
% step(TF_cl_PID)
%%
t = 0:0.1:3;
theta_set = 0.035;      % 2 gradi
theta_set_vec = [zeros(1,11), ones(1,length(t)-11)*theta_set];
figure(2);
lsim(TF_ol,theta_set_vec,t)
figure(3);
lsim(TF_cl_PID,theta_set_vec,t)

