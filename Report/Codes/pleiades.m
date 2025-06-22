%% This is the main code that defines the geometry and mass distribution of the airplane, and automatically generates the necessary input files, so that AVL can run them and produce results
clear
clc
close all

addpath('../../aerospace-design-toolbox-master');
case_name = "pleiades";

% important note:
% - any airfoil you use, the title in the beginning of its .dat file should
% be WITHOUT any spaces, also, it should and must include the point (0,0)
% - NACA Report 2144 considers +ve dA is the right aileron trailing edge up
% - NACA Report 2144 considers +ve dR is the rudder trailing edge left

%% =========================== GEOMETRY DATA ===============================
Mach = 0.72;
v_trim = Mach*sqrt(1.4*287*288);

% Symmetry
IYsym = 0;
IZsym = 0;
Zsym = 0;

%% Fuselage
fuselage.type = "fuselike";
fuselage.name = 'Fuselage';
fuselage.D = 0.4;
fuselage.L = 1.0;
fuselage.Nbody = 20;
fuselage.Bspace = 0.0;
fuselage.yduplicate = 0.0;
fuselage.scale = [1.0, 1.0, 1.0];
fuselage.translate = [-(fuselage.L-1.17), 0.0, 0.0];
fuselage.body = "../airfoils/body.dat";

%% Wing
wing.type = "winglike";
wing.name = 'Wing';
wing.Nchordwise = 22;
wing.Cspace = 1.0;
wing.Nspanwise = 32;
wing.Sspace = -1.1;
wing.scale = [1.0, 1.0, 1.0];
wing.translate = [0.0, 0.0, -0.0];
wing.angle = 1.0;
wing.yduplicate = 0.0;
wing.dihedral = 0; % [deg]
wing.sweep = 20; % [deg]
wing.c_H_r = 0.30;
wing.c_H_t = 0.15;
% wing.sweep_tr = 18.5;
Bref = 1.6;
half_span = Bref / 2; % Total distance between x-axis and tip section
offset_y = 0.0; % [m] (AVL suggests that a fictitious wing section should be put to fill the gap between two wings, same for hTail)

wing.section{1}.data = [0.0, offset_y, 0.0, wing.c_H_r, 0.000]; % Xle Yle Zle Chord Ainc [ Nspan Sspace ]
wing.section{1}.airfoil = '../airfoils/RAE102.dat';

wing.section{2}.data = [0.65*tand(wing.sweep), offset_y+0.65, 0.0, wing.c_H_r+0.65/half_span*(wing.c_H_t-wing.c_H_r), 0.000]; % Xle Yle Zle Chord Ainc [ Nspan Sspace ]
wing.section{2}.airfoil = '../airfoils/RAE102.dat';
wing.section{2}.controlName = "aileron";
wing.section{2}.control = [-1.0, 0.65, 0., 0., 0., -1.0]; % gain, Xhinge, XYZhvec, SgnDup

wing.section{3}.data = [(half_span-offset_y)*tand(wing.sweep), half_span, (half_span-offset_y)*tand(wing.dihedral), wing.c_H_t, -0.0000]; % Xle Yle Zle Chord Ainc [Nspan Sspace]
wing.section{3}.airfoil = '../airfoils/RAE102.dat';
wing.section{3}.controlName = "aileron";
wing.section{3}.control = [-1.0, 0.65, 0., 0., 0., -1.0]; % gain, Xhinge, XYZhvec, SgnDup

% Reference Values
[wing.MGC, wing.MAC] = MEAN_CHORD(wing, 2);
Cref = wing.MAC;
Sref = wing.MGC * Bref;

%% Horizontal Tail
hTail.type = "winglike";
L_H = 2.0; % [m]
hTail.name = "Horizontal Tail";
hTail.Nchordwise = 16;
hTail.Cspace = 1.0;
hTail.Nspanwise = 24;
hTail.Sspace = -1.1;
hTail.scale = [1.0, 1.0, 1.0];
hTail.translate = [0.0, 0.0, 0.0];
hTail.angle = -0.0;
hTail.yduplicate = 0;
hTail.dihedral = -0.0;
% hTail.sweep = 12; % [deg]
hTail_offset_y = 0.0;
hTail_half_span = 0.55/2;
hTail.c_H_r = 0.25;
hTail.c_H_t = 0.6*hTail.c_H_r;
hTail.section{1}.data = [L_H, hTail_offset_y, 0.1, hTail.c_H_r, 0.000]; % Xle Yle Zle Chord Ainc [ Nspan Sspace ]
hTail.section{1}.airfoil = '../airfoils/naca0008.dat';

hTail.section{2}.data = [L_H-0.03/hTail_half_span*(hTail.c_H_t-hTail.c_H_r), hTail_offset_y+0.03, 0.1, hTail.c_H_r+0.03/hTail_half_span*(hTail.c_H_t-hTail.c_H_r), 0.000]; % Xle Yle Zle Chord Ainc [ Nspan Sspace ]
hTail.section{2}.airfoil = '../airfoils/naca0008.dat';
hTail.section{2}.controlName = "elevator";
hTail.section{2}.control = [1.0, 0.65, 0., 0., 0., 1.0]; % gain, Xhinge, XYZhvec, SgnDup

hTail.section{3}.data = [L_H+hTail.c_H_r-hTail.c_H_t, hTail_half_span, 0.1+hTail_half_span*tand(hTail.dihedral), hTail.c_H_t, 0.000]; % Xle Yle Zle Chord Ainc [ Nspan Sspace ]
hTail.section{3}.airfoil = '../airfoils/naca0008.dat';
hTail.section{3}.controlName = "elevator";
hTail.section{3}.control = [1.0, 0.65, 0., 0., 0., 1.0]; % gain, Xhinge, XYZhvec, SgnDup

[hTail.MGC, hTail.MAC] = MEAN_CHORD(hTail, 2);

%% Vertical Tail
vTail.type = "winglike";
vTail.name = "Vertical Tail";
vTail.Nchordwise = 9;
vTail.Cspace = 1.0;
vTail.Nspanwise = 14;
vTail.Sspace = 1.0;
vTail.scale = [1.0, 1.0, 1.0];
vTail.translate = [0.0, 0.0, 0.0];
vTail.angle = 0.0;
vTail.dihedral = 0;

vTail.section{1}.data = [L_H, 0.0, 0.2, hTail.c_H_r, 0.000]; % Xle Yle Zle Chord Ainc [ Nspan Sspace ]
vTail.section{1}.airfoil = '../airfoils/naca0008.dat';

vTail.section{2}.data = [L_H-0.06/0.22*(hTail.c_H_t-hTail.c_H_r), 0.0, 0.2+0.06, hTail.c_H_r+0.06/0.22*(hTail.c_H_t-hTail.c_H_r), 0.000]; % Xle Yle Zle Chord Ainc [ Nspan Sspace ]
vTail.section{2}.airfoil = '../airfoils/naca0008.dat';
vTail.section{2}.controlName = "rudder";
vTail.section{2}.control = [-1.0, 0.8, 0., 0., 0., 1.0]; % gain, Xhinge, XYZhvec, SgnDup, this is +ve in the sense of AVL's geometry axes

vTail.section{3}.data = [L_H+hTail.c_H_r-hTail.c_H_t, 0.0, 0.2+0.22, hTail.c_H_t, 0.000]; % Xle Yle Zle Chord Ainc [ Nspan Sspace ]
vTail.section{3}.airfoil = '../airfoils/naca0012.dat';
vTail.section{3}.controlName = "rudder";
vTail.section{3}.control = [-1.0, 0.8, 0., 0., 0., 1.0]; % gain, Xhinge, XYZhvec, SgnDup, this is +ve in the sense of AVL's geometry axes

[vTail.MGC, vTail.MAC] = MEAN_CHORD(vTail, 3);

%% =========================== MASS DATA ===============================
mass = 300; % [kg]
IX = 7.302; % [kg.m^2]
IY = 146.882; % [kg.m^2]
IZ = 147.754; % [kg.m^2]
IXZ = -5; % [kg.m^2]
g = 9.8065; % [m.s^-2]
rho = 1.225; % [Kg.m^-3]

% Reference Location, should be location of CG if doing Trim calculations 
Xref = 0.36;
Yref = 0.0;
Zref = 0.01;

%% =========================== Print All Files ===========================
%% Geometry File
geom_file = fopen(strcat(case_name, ".avl"), "w");
fprintf(geom_file, strcat(case_name, "\n\n"));
fprintf(geom_file, "# Mach\n%f\n\n", Mach);
fprintf(geom_file, "# IYsym   IZsym   Zsym\n%i   %i   %f\n\n", [IYsym, IZsym, Zsym].');
fprintf(geom_file, "# Sref    Cref    Bref\n%f   %f   %f\n\n", [Sref, Cref, Bref].');
fprintf(geom_file, "# Xref    Yref    Zref\n%f   %f   %f\n\n", [Xref, Yref, Zref].');

% print_object(geom_file, fuselage);
print_object(geom_file, wing);
print_object(geom_file, hTail);
print_object(geom_file, vTail);
fclose(geom_file);

%% ================================ Run AVL ================================
system("rm -f *.ps");
system("rm -f *.sb");
system("rm -f *.eig");
system("rm -f *.st");
system("rm -f *.run");

file_id = fopen("input_file.txt", "w");
fprintf(file_id, "load %s.avl\n", case_name);
fprintf(file_id, "plop\ni\na 0.8\nw 0.9\nf 0.025\n\n"); % settings and mass
fprintf(file_id, "oper\ng\nh\n\nO\nR\n\nD1 RM\n\nD2 PM\n\nD3 YM\n\nM\nMN %f\nD %f\nG %f\nM %f\nV %f\n IX %f\n IY %f\nIZ %f\n\nC1\n\nX\nst\n%s.st\nsb\n%s.sb\nt\nh\n\ns\n%s.run\n\n", Mach, rho, g, mass, v_trim, IX, IY, IZ, case_name, case_name, case_name);
fprintf(file_id, "mode\nn\ng\nh\nw\n%s.eig\n\n", case_name);
fprintf(file_id, "quit\n\n");
fclose(file_id);

system("/home/ahmed/packages/AVL/bin/avl < input_file.txt > /dev/null");

%% ================================ Airplane Data ================================
aircraft = flight_cond(case_name, 1);
% aircraft.C = wing.MGC; % NACA report says mean geometric chord
Mach = aircraft.Mach;

%% Check Trim Condition
fprintf("\n\nTrim Condition:\n")
fprintf("Trim Velocity: %f [m/s]\n", aircraft.velocity);
fprintf("Trim AoA: %f [deg]\n", aircraft.Alpha);
% fprintf("Error in Trim Velocity: %f %%\n", (aircraft.velocity - v_trim)/v_trim*100);
fprintf("Elevator Deflection: %f [deg]\n", aircraft.elevator);

%% Some Aerodynamics Checks
v_cruise = aircraft.velocity;
mu = 1.789e-5; % dynamic viscosity
Re_cruise = rho * v_cruise * aircraft.Cref / mu;
W = aircraft.mass * g;
AR = aircraft.Bref^2 / aircraft.Sref;
[CL_Mach0, CDi, delta, tau, alpha_ind, e] = LLT(v_cruise,Bref,Sref,wing.section{1}.data(4),wing.section{2}.data(4),AR, wing.sweep*pi/180,wing.section{2}.data(5)*pi/180,0,1.5,aircraft.Alpha*pi/180,30);
% CL = CL_lowspeed/(sqrt(1-Mach^2)+(Mach^2/(1+sqrt(1-Mach^2)))*CL_lowspeed/2); % Karman-Tsien
CL = CL_Mach0/(sqrt(1-Mach^2)+(Mach^2*(1+.2*Mach^2)/2/sqrt(1-Mach^2))*CL_Mach0); % Laitone (this is more close to AVL results)
a0 = 2*pi; % airfoil lift coefficient
alpha_0 = -0.0; % Alpha zero lift
CL_alpha = a0*cos(deg2rad(wing.sweep)) / (sqrt(1 + (a0*cos(deg2rad(wing.sweep))/pi/AR)^2) + a0*cos(deg2rad(wing.sweep))/pi/AR);
CL_max = CL_alpha * (13.7*pi/180 - alpha_0); % this is a very naive assumption

%% Required Thrust
fprintf("\n\nThrust Checks:\n")
v_stall = sqrt(W / 0.5 / rho / Sref / CL_max);
b_H = 2*(hTail.section{end}.data(2) - hTail.section{1}.data(2));
S_H = b_H * hTail.MGC;
b_V = vTail.section{end}.data(3) - vTail.section{1}.data(3);
S_V = b_V * vTail.MGC;
t_max = 0.10;
x_max = 0.36;
t_max_H = str2double(hTail.section{1}.airfoil(end-5:end-4))/100;
x_max_H = 0.3; % naca 4-digit always at 0.3C
t_max_V = str2double(hTail.section{1}.airfoil(end-1-5:end-4))/100;
x_max_V = 0.3; % naca 4-digit always at 0.3C
[T_stat, T_dyn] = required_thrust(Re_cruise, v_cruise, v_stall, W, AR, Sref, Cref, Bref, CL_max, t_max, x_max, S_H, hTail.MAC, b_H, t_max_H, x_max_H, S_V, vTail.MAC, b_V, t_max_V, x_max_V, aircraft.e, rho, g, 0.4, 3.2, 0.006);
% Check Required Thrust
fprintf("Static Thrust Required: %f [N]\nDynamic Thrust Required: %f [N]\n\n", T_stat, T_dyn);

%% Stability
BETA0 = aircraft.Beta *pi/180; % (rad)
ALPHA0 = aircraft.Alpha *pi/180; % (rad)
GAMMA0 = 0; % (rad)
VTO = aircraft.velocity; % Take care if this is the true missile velocity, or the one I designed for using AVL
V0 = VTO * sin(BETA0); % (m/sec)
W0 = sqrt((VTO^2 * tan(ALPHA0)^2 * cos(BETA0)^2) / (1 + tan(ALPHA0)^2)); % (m/sec)
U0 = sqrt(VTO^2 - V0^2 - W0^2); % (m/sec)
P0 = 0; % (rad/sec)
Q0 = 0; % (rad/sec)
R0 = 0; % (rad/sec)
PHI0 = 0; % (rad)
THETA0 = ALPHA0 + GAMMA0; % (rad)
PSI0 = 0*pi/180; % (rad)
rho = aircraft.density;
C = aircraft.Cref; % NACA report says mean geometric chord
Bref = aircraft.Bref;
Sref = aircraft.Sref;
g0 = aircraft.grav_acc;
mass = aircraft.mass;
Ixx = aircraft.Ixx;
Iyy = aircraft.Iyy;
Izz = aircraft.Izz;
inertia = [aircraft.Ixx, -aircraft.Ixy, -aircraft.Izx;
          -aircraft.Ixy, aircraft.Iyy, -aircraft.Iyz;
          -aircraft.Izx, -aircraft.Iyz, aircraft.Izz];
invInertia = inv(inertia);

% Body Axes Derivatives, All derivatives are normalized by corresponding mass or moment of inertia
XU = rho * Sref * U0 * aircraft.CXu / 2 / mass;
ZU = rho * Sref * U0 * aircraft.CZu / 2 / mass;
MU =  rho * Sref * C * U0 * aircraft.Cmu / 2 / aircraft.Iyy;
XW = rho * Sref * U0 * aircraft.CXw / 2 / mass;
ZW = rho * Sref * U0 * aircraft.CZw / 2 / mass;
MW =  rho * Sref * C * U0 * aircraft.Cmw / 2 / aircraft.Iyy;
ZWD = 0; % This needs fixing
ZQ = rho * Sref * VTO * C * aircraft.CZq / 4 / mass;
MQ =  rho * Sref * C^2 * VTO * aircraft.Cmq / 4 / aircraft.Iyy;
MWD = 0; % This needs fixing
XDE = rho * Sref * VTO^2 * aircraft.surface(2).CX / 2 / mass;
ZDE = rho * Sref * VTO^2 * aircraft.surface(2).CZ / 2 / mass;
MDE = rho * Sref * C * VTO^2 * aircraft.surface(2).Cm / 2 / aircraft.Iyy;
XDTH = 0.001; % This needs fixing
ZDTH = 0.001; % This needs fixing
MDTH = -0.006; % This needs fixing

YV = rho * Sref * VTO          / 2 / mass * aircraft.CYv;
LV = rho * Sref * VTO * Bref   / 2 / aircraft.Ixx * aircraft.Clv;
NV = rho * Sref * VTO * Bref   / 2 / aircraft.Izz * aircraft.Cnv;
YP = rho * Sref * VTO * Bref   / 4 / mass * aircraft.CYp;
LP = rho * Sref * VTO * Bref^2 / 4 / aircraft.Ixx * aircraft.Clp;
NP = rho * Sref * VTO * Bref^2 / 4 / aircraft.Izz * aircraft.Cnp;
YR = rho * Sref * VTO * Bref   / 4 / mass * aircraft.CYr;
LR = rho * Sref * VTO * Bref^2 / 4 / aircraft.Ixx * aircraft.Clr;
NR = rho * Sref * VTO * Bref^2 / 4 / aircraft.Izz * aircraft.Cnr;
YDA = rho * Sref * VTO^2 / 2 / mass * aircraft.surface(1).CY;
LDA = rho * Sref * VTO^2 * Bref / 2 / aircraft.Ixx * aircraft.surface(1).Cl;
NDA = rho * Sref * VTO^2 * Bref / 2 / aircraft.Izz * aircraft.surface(1).Cn;
YDR = rho * Sref * VTO^2 / 2 / mass * aircraft.surface(3).CY;
LDR = rho * Sref * VTO^2 * Bref / 2 / aircraft.Ixx * aircraft.surface(3).Cl;
NDR = rho * Sref * VTO^2 * Bref / 2 / aircraft.Izz * aircraft.surface(3).Cn;

G = 1 / (1 - aircraft.Izx^2/aircraft.Ixx/aircraft.Izz);
LV_DASH = (LV + aircraft.Izx/aircraft.Ixx*NV) * G;
LP_DASH = (LP + aircraft.Izx/aircraft.Ixx*NP) * G;
LR_DASH = (LR + aircraft.Izx/aircraft.Ixx*NR) * G;
NV_DASH = (NV + aircraft.Izx/aircraft.Izz*LV) * G;
NP_DASH = (NP + aircraft.Izx/aircraft.Izz*LP) * G;
NR_DASH = (NR + aircraft.Izx/aircraft.Izz*LR) * G;
LDA_DASH = (LDA + aircraft.Izx/aircraft.Ixx*NDA) * G;
LDR_DASH = (LDR + aircraft.Izx/aircraft.Ixx*NDR) * G;
NDA_DASH = (NDA + aircraft.Izx/aircraft.Izz*LDA) * G;
NDR_DASH = (NDR + aircraft.Izx/aircraft.Izz*LDR) * G;

% Longitudinal Dynamics
A_long=[XU,XW,-W0,-g0*cos(THETA0);...
        ZU/(1-ZWD),ZW/(1-ZWD),(ZQ+U0)/(1-ZWD),-g0*sin(THETA0)/(1-ZWD);...
        MU+MWD*ZU/(1-ZWD),MW+MWD*ZW/(1-ZWD),MQ+MWD*(ZQ+U0)/(1-ZWD),-MWD*g0*sin(THETA0)/(1-ZWD);...
        0,0,1,0];
B_long=[XDE,XDTH ;...
        ZDE/(1-ZWD),ZDTH/(1-ZWD);...
        MDE+MWD*ZDE/(1-ZWD),MDTH+MWD*ZDTH/(1-ZWD);...
        0,0];
C_long=eye(4);
D_long=zeros(4,2);
SS_LON = ss(A_long, B_long, C_long, D_long);
TF_LON = tf(SS_LON);
theta_de = TF_LON(4,1);

% Lateral Dynamics
A_lat = [YV , (W0+YP) , -U0+YR, g0*cos(THETA0), 0;...
        LV_DASH , LP_DASH , LR_DASH , 0 , 0;...
        NV_DASH , NP_DASH , NR_DASH , 0 , 0;...
        0 , 1 , tan(THETA0) , 0 , 0;...
        0 , 0 , 1/cos(THETA0) ,0 ,0];
B_lat = [YDA , YDR;...
         LDA_DASH , LDR_DASH;...
         NDA_DASH , NDR_DASH;...
         0 , 0;...
         0 , 0]; % check this, might need fixing
C_lat=eye(5);
D_lat=zeros(5,2);

% Long Period (Phugoid)
omega_lp = sqrt(-g*cos(THETA0)*ZU/(ZQ+U0) + g*sin(THETA0)*XU/(ZQ+U0));
zeta_lp = -(XU+g*sin(THETA0)/(ZQ+U0)) / 2 / omega_lp;

% Short Period
omega_sp = sqrt(-MW*U0 + ZW*MQ);
zeta_sp = -(ZW+MQ+MWD*U0) / 2 / omega_sp;

% Rolling Mode
tau_roll = -1/LP_DASH;

% 2DOF Dutch Roll
omega_DR = sqrt(YV*NR_DASH+NV_DASH*(U0+YR));
zeta_DR = -(YV+NR_DASH) / 2 / omega_DR;

SM = (aircraft.XNP - aircraft.XCG) / aircraft.Cref * 100;

fprintf("\n\nStability Checks:\n");
fprintf("Static Margin = %f %%\n", SM);

fprintf("Long Period Mode: zeta = %f, wn = %f\n", zeta_lp, omega_lp);
fprintf("Short Period Mode: zeta = %f, wn = %f\n", zeta_sp, omega_sp);
fprintf("1DOF Roll Mode: tau = %f\n", tau_roll);
fprintf("2DOF Dutch Roll Period Mode: zeta = %f, wn = %f\n", zeta_DR, omega_DR);

% Lateral Dynamics Check
fprintf("(Clb*Cnr)/(Clr*Cnb) = %f  (>1 if spirally stable)\n", aircraft.Clv*aircraft.Cnr/(aircraft.Clr*aircraft.Cnv));

% pre_autopilot;
