clear
close all
clc
%% MCU Parameters

Jdc = 0.352*10^-4;
Jbr = 0.38*10^-4;
J = Jdc+Jbr;
km = 0.12;

kv = km;  % Voltage Constant [V*s/rad]
kt = km;  % Torque Constant [Nm/A]
La = 4.1e-3;    % Armature inductance [H]
Ra = 3.2;   % Armature Resistance [Ohm]
Va = 24;    % Rated Voltage [V]

% MCU frequency
MCU_Frequency = 200e6;
MCU_Period = 1/MCU_Frequency;
%PWM Switching frequency
PWM_Frequency 	= 1e4;    			
PWM_Period      = 1/PWM_Frequency;  %s  	// PWM switching time period
%PWM register
TBPRD  = uint16 (MCU_Frequency/(2*PWM_Frequency)); % Time Base Period Register

%%Sample Period
TSample = PWM_Period; %0.0001s


%% Analog Reading
%input sensing
ADCResolution = 2^12;
ADCCount = ADCResolution-1;
VrefADC = 3;
GainVsense = 4.99/(62+4.99);
GainCurrentSense = 10;
RShunt = 0.007;

%Current Offset
OffsetADCINC4 = 2284; % Right value CHECKED

%%Encoder
PulseNumber = 2048*4;
RadiantsPerCount = 2*pi/(PulseNumber-1);

%% CONVERSION
RadToRpm = 60/(2*pi);     % Convert rad->RPM
RpmToRads = 2*pi/60;    % Convert rpm->RAD/S

%% Filter
%Offset Current Filter
TaoFilterOffsetCurrent=1; 

% Omega filter - To Be Defined

TaoFilterOmega = 0.001; % Right value CHECKED


%% Motor Poles & Time constants

taumecc = 85e-3;    % Mechanical Time Constant [s], FROM DATASHEET
tauel = La/Ra; % Electrical Time Constant [s]


b = J/taumecc; % Viscous friction, derived from datasheet parameters

%% ------------- PI CURRENT parameters ----------------

TiI = tauel;    % Current Integral time constant 
taui = 0.001;   % Current Desired time constant <-Chosen value
KpI = La/taui;  % Current Proportional constant 

%% ------------- PI SPEED parameters ----------------

TiW = taumecc;  % Speed Integral time constant
tauw = 10*taui;    % Speed Desired time constant  <-Chosen Value
KpW = J/(tauw*km); % Speed Proportional constant

%% ------------ WITH ESTIMATED PARAMETERS --------------

% I set this flag to true to retune both PI loops with the identified parameters
% (see 02_parameter_estimation), or to false to keep the datasheet-based tuning
useEstimatedParams = true;


% MOTOR 4
La_estim = 0.0061;
Ra_estim = 3.3369;
J_estim = 8.3561e-05;
b_estim = 8.5764e-04;

tauel_new = La_estim/Ra_estim;
taumecc_new = J_estim/b_estim;

if useEstimatedParams
    TiI = tauel_new;
    KpI = La_estim/taui;   % fixed: previously assigned to 'Kpi', so the estimated gain was never used

    TiW = taumecc_new;
    KpW = J_estim/(tauw*km);
end



%% Hardware
comport='COM4';