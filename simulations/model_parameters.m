%% Model parameters
g = 9.81;
l = 0.085;
lb = 0.075;
mb = 0.123;  % kg
mw = 0.152;  % kg
Ib = 4.9e-3;  % m^-2
Iw = 2.7e-3;  % m^-2
Cb = 1.02e-3;
Cw = 0.05e-3;
Km = 25.1e-3;

%% Old
% l = 0.085;
% lb = 0.075;
% mb = 0.419;
% mw = 0.204;
% Ib = 3.34e-3;
% Iw = 0.57e-3;
% Cb = 1.02e-3;
% Cw = 0.05e-3;
% Km = 25.1e-3;

%% Diff Eq coefficients
a0 = (mb * lb + mw * l) * g / (Ib + mw * l^2);
a1 = -1 / (Ib + mw * l^2);
a2 = -Cb / (Ib + mw * l^2);
a3 = Cw/(Ib + mw * l^2);
b0 = -a0;
b1 = (Ib + Iw + mw * l^2) / (Iw * (Ib + mw * l^2));
b2 = -a2;
b3 = -Cw * b1;

%% Control system parameters
maxDCCurrent = 10;
sampleTime = 0.01;