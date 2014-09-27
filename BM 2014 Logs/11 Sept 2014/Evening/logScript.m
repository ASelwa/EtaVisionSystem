clear all
clc

% m =csvread('LOG376 Todd.CSV', 4, 1)

%  filename = ['save' , num2str(index+7), ' - Column load, rectangular beam - apparatus state.txt'];
filename = 'LOG381.CSV';


logFile = importdata(filename, '\t');
% More frequently updated Power log file
SRMFile = importdata('SLG381.CSV', '\t');


% Organize data into a matrices
data = [];
for index = 20:length(logFile)
    temp = logFile(index);  % access the row
    temp = temp{1};  % get the string for sscanf 
    data = [data; sscanf(temp, '%d:%d:%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f', [1, inf])];
end
SRMdata = [];
for index = 20:length(SRMFile)
    temp = SRMFile(index);  % access the row
    temp = temp{1};  % get the string for sscanf 
    SRMdata = [SRMdata; sscanf(temp, '%f, %f, %f, %f, %f, %f, %f, %f, %f, %f', [1, inf])];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Structure of 'data'
% 1      2        3        4     5     6    7     8     9          10            11     12       13         14        15         16
% Hours  Minutes  Seconds  Lat   Long  Alt  Dist  Disp  GPS Speed  Target Speed  Power  Cadence  Sim Speed  Sim Dist  HeartRate  Battery
%                           (degrees)  (m)  (m)   (m)     (km/hr)   (km/hr)       (W)    (RPM)    (km/hr)    (m)        (BPM)     (V)
%
%    Structure of 'SRMdata'
% 1      2     3     4          5             6      7        8          9         10         
% Time   Dist  Disp  GPS Speed  Target Speed  Power  Cadence  Sim Speed  Sim Dist  HeartRate  
%  (ms)   (m)   (m)   (km/hr)   (km/hr)       (W)    (RPM)    (km/hr)    (m)        (BPM)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Separate all the data from 'data' and 'SRMdata'
time = data(:, 1)*3600 + data(:, 2)*60 + data(:,3) - (data(1, 1)*3600 + data(1, 2)*60 + data(1,3));
dist = data(:, 7);
disp = data(:, 8);
GPSSpeed = data(:, 9);
power = data(:, 11);
cadence = data(:, 12);
simSpeed = data(:, 13);

SRMtime = (SRMdata(:, 1) - SRMdata(1,1))/1000;
SRMdist = SRMdata(:, 2);
SRMdisp = SRMdata(:, 3);
SRMGPSSpeed = SRMdata(:, 4);
SRMpower = SRMdata(:, 6);
SRMcadence = SRMdata(:, 7);
SRMsimSpeed = SRMdata(:, 8);


%% Plotting
hold all
plot(SRMtime, SRMGPSSpeed, '--r')
plot(SRMtime, SRMsimSpeed, '--b')
plot(time, GPSSpeed, 'r')
plot(time, simSpeed, 'b')
xlabel('Time (s)')
ylabel('Speed (km/hr)')
legend('SRM GPS', 'SRM Simulated', 'GPS', 'Simulated', 'Location', 'best')
title('Actual Speed compared to Simulated Speed')

figure
plot(time, power)
xlabel('Time (seconds)')
ylabel('Power (Watts)')
title('Power versus Time')

figure
plot(SRMtime, SRMcadence)
hold
plot(SRMtime, SRMGPSSpeed, 'r')

figure
plot(dist, GPSSpeed)

%%

% Plotting
plot(SRMtime, SRMGPSSpeed, '-r')
hold
plot(SRMtime, SRMsimSpeed, '-b')
xlabel('Time (s)')
ylabel('Speed (km/hr)')
legend('GPS', 'Simulated')
title('Actual Speed compared to Simulated Speed')

figure
plot(SRMtime, SRMpower)
xlabel('Time (seconds)')
ylabel('Power (Watts)')
title('Power versus Time')


