function mass_data = inertia2(wing, rho, ax)

if(length(wing.section) > 2)
    warning("This function isn't yet able to deal with multiple wing sections, it assumes the wing consists of 2 sections only, you should care for this message only if you use different airfoils, incidence angles or sweep angles for the different sections, otherwise, it's ok to use this function and ignore this warning");
end
if(~exist("ax", "var"))
    ax = 2;
end

%% Wing Data
b = wing.section{end}.data(ax) - wing.section{1}.data(ax); % semi-span length
% Root Airfoil
AIRFOIL_r = readtable(wing.section{1}.airfoil).Variables;
x = AIRFOIL_r(:, 1);
y = AIRFOIL_r(:, 2);
ind_zero = find(x==0);
xu_r = x(1:ind_zero);
xl_r = flip(x(ind_zero:end));
yu_r = y(1:ind_zero);
yl_r = flip(y(ind_zero:end));
if(~(all(size(xl_r)==size(xu_r)) && all(xl_r==xu_r)))
    yl_r = interp1(xl_r, yl_r, xu_r, "linear", "extrap");
end
thickness_r = yu_r - yl_r;
cr = wing.section{1}.data(4);
tau_r = max(thickness_r); % max thickness per root chord
thickness_coeff_r = airfoil_eq(xu_r, thickness_r) / tau_r;
clear x y % this is just a check because I shouldn't use them later...

% Tip Airfoil
AIRFOIL_t = readtable(wing.section{end}.airfoil).Variables;
x = AIRFOIL_t(:, 1);
y = AIRFOIL_t(:, 2);
ind_zero = find(x==0);
xu_t = x(1:ind_zero);
xl_t = flip(x(ind_zero:end));
yu_t = y(1:ind_zero);
yl_t = flip(y(ind_zero:end));
if(~(all(size(xl_t)==size(xu_t)) && all(xl_t==xu_t)))
        yl_t = interp1(xl_t, yl_t, xu_t, "spline", "extrap");
end
thickness_t = yu_t - yl_t;
ct = wing.section{end}.data(4);
tau_t = max(thickness_t); % max thickness per tip chord
thickness_coeff_t = airfoil_eq(xu_t, thickness_t) / tau_t;
clear x y % this is just a check because I shouldn't use them later...

% This is a trial to deal with the wing if it has aerodynamic twist
a0 = (thickness_coeff_r(1) + thickness_coeff_t(1)) / 2;
a1 = (thickness_coeff_r(2) + thickness_coeff_t(2)) / 2;
a2 = (thickness_coeff_r(3) + thickness_coeff_t(3)) / 2;
a3 = (thickness_coeff_r(4) + thickness_coeff_t(4)) / 2;
a4 = (thickness_coeff_r(5) + thickness_coeff_t(5)) / 2;

% Sweep Angle
lambda_leading = atan((wing.section{end}.data(1) - wing.section{1}.data(1)) / b);
lambda_trailing = atan(((wing.section{end}.data(1)+wing.section{end}.data(4)) - (wing.section{1}.data(1)+wing.section{1}.data(4))) / b);
lambda = lambda_leading*3/4 + lambda_trailing*1/4; % sweep angle at 0.25*C

Ainc = (wing.section{1}.data(5)*cr + wing.section{end}.data(5)*ct) / (cr+ct);

%% Constants
Ka = tau_r*(3*cr^2 + 2*cr*ct + ct^2) + tau_t*(cr^2 + 2*cr*ct + 3*ct^2);
Kb = tau_r*(4*cr^3 + 3*cr^2*ct + 2*cr*ct^2 + ct^3) + tau_t*(cr^3 + 2*cr^2*ct + 3*cr*ct^2 + 4*ct^3);
Kc = tau_r*(3*cr^2 + 4*cr*ct + 3*ct^2) + 2*tau_t*(cr^2 + 3*cr*ct + 6*ct^2);
Kd = tau_r*(cr + ct) * (2*cr^2 + cr*ct + 2*ct^2) + tau_t*(cr^3 + 3*cr^2*ct + 6*cr*ct^2 + 10*ct^3);
Ke = tau_r*(5*cr^4 + 4*cr^3*ct + 3*cr^2*ct^2 + 2*cr*ct^3 + ct^4) + tau_t*(cr^4 + 2*cr^3*ct + 3*cr^2*ct^2 + 4*cr*ct^3 + 5*ct^4);
Kf = tau_r*(cr^2 + 2*cr*ct + 2*ct^2) + tau_t*(cr^2 + 4*cr*ct + 10*ct^2);
Kg = tau_r^3*(35*cr^4 + 20*cr^3*ct + 10*cr^2*ct^2 + 4*cr*ct^3 + ct^4)...
    + tau_r^2*tau_t*(15*cr^4 + 20*cr^3*ct + 18*cr^2*ct^2 + 12*cr*ct^3 + 5*ct^4)...
    + tau_r*tau_t^2*(5*cr^4 + 12*cr^3*ct + 18*cr^2*ct^2 + 20*cr*ct^3 + 15*ct^4)...
    + tau_t^3*(cr^4 + 4*cr^3*ct + 10*cr^2*ct^2 + 20*cr*ct^3 + 35*ct^4);

v0 = 1/60*(40*a0 + 30*a1 + 20*a2 + 15*a3 + 12*a4);
v1 = 1/60*(56*a0 + 50*a1 + 40*a2 + 33*a3 + 28*a4);
v2 = 1/980*(856*a0 + 770*a1 + 644*a2 + 553*a3 + 484*a4);
v3 = 2/5*a0^3 + a0^2*a1 + 3/4*a0^2*a2 + 3/5*a0^2*a3 + 1/2*a0^2*a4 + 6/7*a0*a1^2 + 4/3*a0*a1*a2 + 12/11*a0*a1*a3 + 12/13*a0*a1*a4 ...
   + 6/11*a0*a2^2 + 12/13*a0*a2*a3 + 4/5*a0*a2*a4 + 2/5*a0*a3^2 + 12/17*a0*a3*a4 + 6/19*a0*a4^2 + 1/4*a1^3 + 3/5*a1^2*a2 ...
   + 1/2*a1^2*a3 + 3/7*a1^2*a4 + 1/2*a1*a2^2 + 6/7*a1*a2*a3 + 3/4*a1*a2*a4 + 3/8*a1*a3^2 + 2/3*a1*a3*a4 + 3/10*a1*a4^2 ...
   + 1/7*a2^3 + 3/8*a2^2*a3 + 1/3*a2^2*a4 + 1/3*a2*a3^2 + 3/5*a2*a3*a4 + 3/11*a2*a4^2 + 1/10*a3^3 + 3/11*a3^2*a4 + 1/4*a3*a4^2 + 1/13*a4^3;

%% Mass
V = b/12*Ka*v0;
mass = rho*V;

%% Centre of Gravity
CG = nan(3, 1);
CG(1) = -(3*Kb*v1 + 4*b*Kc*v0*tan(lambda)) / (20*Ka*v0);
CG(2) = b*Kc/5/Ka;
CG(3) = (sum(yu_r + yl_r)/(length(yu_r) + length(yl_r))*cr + sum(yu_t + yl_t)/(length(yu_t) + length(yl_t))*ct)/(cr+ct); % This is made by ASalahHammad

%% Moments of Inertia at Origin
I_tensor = nan(3, 3);
I_tensor(1, 1) = rho*b/3360*(56*b^2*Kf*v0 + Kg*v3);
I_tensor(2, 2) = rho*b/10080*(84*b*(2*b*Kf*v0*tan(lambda)^2 + Kd*tan(lambda)*v1) + 49*Ke*v2 + 3*Kg*v3);
I_tensor(3, 3) = rho*b/1440*(12*b*(2*b*(tan(lambda)^2+1)*Kf*v0 + Kd*v1*tan(lambda)) + 7*Ke*v2);
I_tensor(1, 2) = -rho*b^2/240*(4*b*Kf*v0*tan(lambda) + Kd*v1);
I_tensor(1, 3) = 0;
I_tensor(2, 3) = 0;
I_tensor(2, 1) = I_tensor(1, 2);
I_tensor(3, 1) = I_tensor(1, 3);
I_tensor(3, 2) = I_tensor(2, 3);

%% Parallel Axis THeorem
I_tensor(1, 1) = I_tensor(1, 1) - mass*CG(2)^2;
I_tensor(2, 2) = I_tensor(2, 2) - mass*CG(1)^2;
I_tensor(3, 3) = I_tensor(3, 3) - mass*CG(1)^2;
I_tensor(1, 2) = I_tensor(1, 2) - mass*CG(1)*CG(2);
I_tensor(1, 3) = I_tensor(1, 3) - mass*CG(1)*CG(3);
I_tensor(2, 3) = I_tensor(2, 3) - mass*CG(2)*CG(3);
I_tensor(2, 1) = I_tensor(1, 2);
I_tensor(3, 1) = I_tensor(1, 3);
I_tensor(3, 2) = I_tensor(2, 3);

%% Rotation Matrices
Rx = @(phi) [1,    0,         0;
             0, cos(phi), -sin(phi);
             0, sin(phi),  cos(phi)];

Ry = @(theta) [cos(theta),  0, sin(theta);
                   0,       1,    0;
               -sin(theta), 0, cos(theta)];

Rz = @(psi) [cos(psi), -sin(psi), 0;
             sin(psi),  cos(psi), 0;
                 0,        0,     1]; %# ok

%% Apply transformations due to inclination -> dihedral -> vertical stabilizer -> AVL's Geometry axes
%% Transformation due to inclination angle
CG = Ry(Ainc) * CG;
I_tensor = Ry(Ainc) * I_tensor * Ry(Ainc).';

%% Dihedral
% Very Important Note
% Notice that my definition of +ve dihedral angle is opposite to the +ve
% angle definition using stability axes, this is why I use a -ve sign
CG = Rx(-wing.dihedral*pi/180) * CG;
I_tensor = Rx(-wing.dihedral*pi/180) * I_tensor * Rx(-wing.dihedral*pi/180).';

%% Transformation of Vertical Stabilizer
if(ax==3)
    CG = Rx(-pi/2) * CG;
    I_tensor = Rx(-pi/2) * I_tensor * Rx(-pi/2).';
end

%% Transformation from Stability axes to AVL's geometric axes
CG = Ry(pi) * CG;
CG(1) = CG(1) + wing.section{1}.data(1);
CG(2) = CG(2) + wing.section{1}.data(2);
CG(3) = CG(3) + wing.section{1}.data(3);
I_tensor = Ry(pi) * I_tensor * Ry(pi).';

%% Output
mass_data = [mass, CG.', I_tensor(1, 1), I_tensor(2, 2), I_tensor(3, 3), I_tensor(1, 2), I_tensor(1, 3), I_tensor(2, 3)];

end % endfuntion
