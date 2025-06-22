clearvars;
clc;

%% Load Data
load('CMD.mat') % CRUISE MISSILEs DATA
load('RegressionModel.mat') %the model of WTO
% If u want to add more data open "dataTable" from the workspace and add it then train
% the model here:
%[RegressionModel, validationRMSE] = trainRegressionModel(dataTable);

%SEALEVEL CONDITIONS
a_SL=340.3;   %m/s
T_SL=288.2;   %k
rho_SL=1.225; %kg/m^3
meu=1.784e-5;

%% MISSION REQUIREMENTS
Slenderness=8.5 %mean(dataTable.Slenderness);    %Slenderness Ratio	
lambda=Slenderness
M_stall=.09; %Input
M_max=.7;   %Input  
M_cr=M_max/1.2;   
Mach=M_max  
R=300;      %km
Range_km_=R
W_PL=60;   %kg
Payload_kg_=W_PL
T=table(Slenderness,Mach,Range_km_,Payload_kg_);
W_TO=RegressionModel.predictFcn(T)
% ASSUMED BASED ON DATA
SFC=33.5;     %(SFC(1/s)*10^5) Worst case from many (TJs and TFs)        
C_D_0=0.0296; 
e=0.9; % Assume
AR=8; % Assume
K=1/pi/e/AR;
%% MATCHING PLOT
W_S=.5*(M_stall*a_SL)^2*1.5*rho_SL;
T_W=rho_SL*(M_max*a_SL)^2*C_D_0/2./W_S+2*K/rho_SL/(M_max*a_SL)^2.*W_S;
S=W_TO/W_S
T_1=T_W*W_TO % AMT TITAN MICROJET ENGINE


%% WingSizing
% PROPFAN CRUISE MISSILE WING AIRFOIL for the Wing
sweepAngle=24.5 % 5-15 deg at leading edge
b=sqrt(S*AR)
t_c=.082 % Maximum thickness
x_c=0.4 % Maximum thickness point
c=S/b
taperRatio=0.5 %c_t/c_r "Assume"
c_r=2*S/b/(1+taperRatio);
c_t=taperRatio*c_r;
MAC=2/3*c_r*(1+taperRatio+taperRatio^2)/(1+taperRatio);

%% TailSizing
% NACA 0012 for the TAIL
t_c_T=.012; %for the tail
x_c_T=0.3; % Maximum thickness point
V_H=0.6;   % Assume 0.6-1
V_v=0.03 ;  % Assume 0.02-0.07
S_H_S=0.3; % Assume 0.2-0.4
S_H=S_H_S*S
L_H=V_H*S*MAC/S_H % Moment Arm
S_v=V_v*S*b/L_H;
AR_H=4; % Assume 3.5-5
AR_v=1.5; % Assume 1.5-2.5
b_H=sqrt(AR_H*S_H)
b_v=sqrt(AR_v*S_v)
c_H=b_H/AR_H
c_v=b_v/AR_v

%% DRAG
Diameter=.3 % (m) from the dataTable
Length=Diameter*lambda
Re=rho_SL*Mach*a_SL*MAC/(meu);
Re_T_H=rho_SL*Mach*a_SL*c_H/(meu);
Re_T_v=rho_SL*Mach*a_SL*c_v/(meu);
Re_fus=rho_SL*Mach*a_SL*Length/(meu);
S_wet=2*(1+.2*t_c)*S;
S_wet_v=2*(1+.2*t_c_T)*S_v;
S_wet_H=2*(1+.2*t_c_T)*S_H;
S_wet_fus=pi*Diameter*Length;
c_f=0.455/log10(Re)^2.58;
c_f_v=0.455/log10(Re_T_v)^2.58;
c_f_H=0.455/log10(Re_T_H)^2.58;
c_f_fus=0.455/log10(Re_fus)^2.58;
FF=1+.5/x_c*t_c+100*t_c^4;
FF_T=1+.5/x_c_T*t_c_T+100*t_c_T^4;
FF_fus=1+60/lambda^3+lambda/400;
C_D0=(c_f*FF*S_wet+c_f*FF*S_wet+c_f_H*FF_T*S_wet_H+c_f_H*FF_T*S_wet_v+c_f_fus*FF_fus*S_wet_fus)/S
T_W=rho_SL*(M_max*a_SL)^2*C_D0/2./W_S+2*K/rho_SL/(M_max*a_SL)^2.*W_S;
T=T_W*W_TO 
sfc=19.333;  %gr/(Kn*sec) get it from the Specs. of the engine with same thrust  
SFC=sfc/10^6*9.8; %1/s
L_D_max=8; %based on nasa paper
WL_WTO=exp(-R*10^3*SFC/340/M_max/L_D_max/0.866); %breguet EQ
W_f=(1-WL_WTO)*W_TO %fuel weight
W_L=WL_WTO*W_TO     %empty weight
% %% Some Relations
% C_L_alpha=a_0/(1+(a_0(1+tau))/(pi*AR));% "tau" is the lift span efficiency factor and is function of both aspect ratio and taper ratio
% C_D0W=cd0 %W:Wing 
% k2=k1+(1+delta)/pi/AR
% CDW = C_D0W + k2*CL^2 
% C_M0=AR*cosd(sweepAngle)^2/(AR+2*cosd(sweepAngle))*C_m0%relation assumes: No geometric or aerodynamic twist in the wing
C_L=W_TO*9.8/(.5*rho_SL*(M_max*a_SL)^2*S)

