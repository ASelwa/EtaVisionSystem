%Speed
U = 145/3.6; %Speed

%Constants
g = 9.81; %m/s
mu = 1.85E-5; %Pa*s
rho = 1; %kg/m^3 Battle Mountain
q = 0.5*rho*U^2; % Dynamic Pressure

% Rolling parameters
Wpilot = 80*g; %N
Whpv = 20*g; %N
Wwheels = 2*g; %N
W = Wpilot + Whpv + Wwheels; %N
M = W/g; %kg
Mwheels = Wwheels/g; %kg
Crr1 = 0.0015; 
Crr2 = 2/3*4.1E-5*3.6; %s/m

% Aerodynamic parameters
L = 2.88; % Body length [m]
Lnose = 1; % Nose length [m]
h = 0.75; % Body height [m]
w = 0.45; % Body width [m]
xt = 1.9; % Transition point [m]
Af = pi*(h/2)*(w/2); % Frontal area [m^2]
Aside = 0.5*pi*Lnose*(h/2) + (L-Lnose)*h; % Side view area
Awet = (0.037*Af^2 + 0.02*Af + 1)*Aside*2; % Wetted area



%-----------------
% Drivetrain efficiency
etaD = 0.96334;



%-----------------
% Rolling resistance
Crr = Crr1 + Crr2*U; % Rolling Resistance coefficient
Droll = Crr*W; % Rolling Drag
Proll = Droll*U; % Rolling Power


%--------------
% Aerodynamic Drag

% Flat plate drag
Dflam = 1.328*h*q*sqrt(mu/U/rho)*sqrt(xt); %Laminar drag
deltalamxt = 5*sqrt(mu/U/rho)*sqrt(xt); %Lam BL thickness at xt
deltaturbxt = 0.13/0.097*deltalamxt; %Turb BL thickness at xt
xdel = (deltaturbxt/0.375*(U*rho/mu)^0.2)^(1/0.8);
x0 = xt - xdel; %imaginary turb start
Dfturb = 0.0576/0.8*h*q*(mu/U/rho)^0.2*((L-x0)^0.8 - xdel^0.8);
Cfflat = (Dflam + Dfturb)/(q*h*L);

% Body Drag
Cdwet = Cfflat*(1 + 1.8*(Af^0.75)/(L^1.5) + 39*(Af^3)/(L^6));
CdAbody = Cdwet*Awet;

% Other Drag
CdAfwheel = 0.002; %Full fairing, very small, sealed, wheel well
CdArwheel = 0.002; %Full fairing, very small, sealed, wheel well
CdAunclean = 0.001; %Smooth surface, few seams

% Total aerodynamic drag
CdA = CdAbody + CdAfwheel + CdArwheel + CdAunclean;
Cd = CdA/Af;
Daero = q*CdA;
Paero = Daero*U;