clear
close all

%% PLOTTING PARAMETERS

plotnumber = 3; %number of signals in each plot
TSample = 1e-4;

% Number of samples to plot

% N = 10000;

N = 5000;


%% LOAD INTO WORKSPACE ACQUIRED DATA
% I select up to 'plotnumber' acquisitions from this folder (same reference signal)
cd(fileparts(mfilename('fullpath')));


% data1 = load('PI_CURRENT_SISATSIAW_6_5.mat').simout;
% data2 = load('PI_ESTIM_CURRENT_SISATSIAW_6_5_2.mat').simout;
% % data3 = load('PI_SPEED_SISATSIAW.mat').simoutModel;

% data1 = load('PI_ESTIM_CURRENT_SISATSIAW_6_5.mat').simout;

data1 = load('DEADBEAT_SPEED_NOSATNOAW_IDEAL.mat').simout;
data2 = load('DEADBEAT_SPEED_SISATNOAW_IDEAL.mat').simout;
data3 = load('DEADBEAT_SPEED_SISATNOAW_REAL.mat').simout;


for i=1:N
    data(i,1) = data1(i,1);                     % Reference (equal for all acquired data)
    for j=1:plotnumber
         datatemp = eval(['data' num2str(j)]);  % In order to obtain "dataj" with j = index
         data(i,j+1) = datatemp(i,2);           % All acquired data in one variable "data"
    end
end
time = (0:TSample:(length(data)-1)*TSample);    % Time on the X Axes
%% PLOT DATA
figure('Name','PLOT')
hold on
grid on
clrgraph = rand(3,plotnumber+1);
for i=1:plotnumber+1
    plot(time,data(:,i),'LineWidth', 2, 'Color', [clrgraph(:,i)])
end
hold off

title('DEADBEAT Speed') 
ylabel('Speed [rpm]')
% ylabel('Current [A]')
xlabel('Time [s]')
legend('Reference','IDEAL, NO SAT','IDEAL + SAT','REAL+SAT','Location','southeast')

