
%% DC MOTOR PARAMETER

La = 4.1e-3;        % Inductance (H)
Ra = 3.2;           % Resistance (Ohm) 
In = 1.9;           % Nominal current (A)
Cn = 0.22;          % Nominal Torque (Nm)
km = 0.12;          %  (Nm/A)
Jm = 0.352e-4;      % (kg m2)
Nv = 1750;          % Idle speed
Nn = 1500;          % Nominal speed in rpm 
Wn = Nn*2*pi/60;    % Nominal speed in rad/s 
b = 0.0003;         % Coefficient of friction

%% OTHER PARAMETER

Fsw  = 10e3;        % PWM Frequency
TSample = 0.1e-3;    % TSample
Jmbl = 0.38e-4;
J = Jm+Jmbl;      % Total Inertia;

% Write here Estimated value (R-L-J-b)
% MOTOR 4
La = 0.0061;
Ra = 3.3369;
J = 8.3561e-05;
b = 8.5764e-04;

tauel_new = La/Ra;
taumecc_new = J/b;


%% Encoder
PulseNumber = 2048*4;
RadiantsPerCount=2*pi/(PulseNumber-1);

%% CONVERSION
RadToRpm = 60/(2*pi); %Convert rad->RPM
RpmToRads = 2*pi/60;

%% Omega filter
TaoFilterOmega = 0.001;

%% TRANSFER FUNCTION
s=tf('s');          
z=tf('z',TSample);

%DC Motor continuous transfer function
%Gmot=tf([Km],[La*J Ra*J Km*Km]);
Gmot=tf([km],[La*J Ra*J+b*La km*km+b*Ra]);
Gmot=24*Gmot;

%DC Motor discrete transfer function
Gpz=c2d(Gmot,TSample)
zpk(Gpz)
Dz=minreal((1/Gpz)*((z^-1)/(1-(z^-1))))
zpk(Dz)


Bz = Dz.Numerator{1,1};
Az = Dz.Denominator{1,1};

% F(z): anti-windup polynomial, u = (B/F)*e + ((F-A)/F)*u_sat
Fz = 20*(z-0.93)*(z-0.96); % Fz


FFNum = Fz.Numerator{1,1} - Az;
FFDen = Fz.Numerator{1,1};