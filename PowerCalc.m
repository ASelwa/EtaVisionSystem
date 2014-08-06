% Validation: Friction coefficient vs. x using figure 2.2.7
% Validation: Boundary layer thickness vs. x using figure 2.2.6
% Validation: Cfwet vs. Re and xt using figure 3.2.1, computed value may be
% low by a few perfect, but all trends match exactly

function PowerCalc(BM,bikeModel,mode,speed)

% Constants
g = 9.81; %m/s
mu = 1.85E-5; %Pa*s
rho = 1.13; %kg/m^3
rho = 1;
if BM
    rho = 1; %kg/m^3 Battle Mountain
end

if mode == 1
    Npts = 1;
    Uset = speed;
else
    Npts = 60;
    Uset = linspace(0,160/3.6,Npts); %m/s
end

%ACE
if bikeModel == 1
    % Rolling parameters
    Wpilot = 80*g; %N
    Whpv = 34*g; %N
    Wwheels = 2*g; %N
    Crr1 = 0.004;
    Crr2 = 2/3*4.1E-5*3.6; %s/m

    % Aerodynamic parameters
    L = 2.81; % Body length [m]
    Lnose = 0.9; % Nose length [m]
    h = 0.74; % Body height [m]
    w = 0.56; % Body width [m]
    hcanopy = 0.2; % Canopy height [m]
    wcanopy = 0.3; % Canopy width [m]
    xt = 0.2; % Transition point [m]
    Af = pi*(h/2)*(w/2); % Frontal area [m^2] (0.325 from SolidWorks)
    Aside = 0.5*pi*Lnose*(h/2) + (L-Lnose)*h; % Side view
    Awet = (0.037*Af^2 + 0.02*Af + 1)*Aside*2; % Wetted area (7.63 in SolidWorks)
    Awet = 7.63; % Wetted area from SolidWorks
    Afcanopy = 0.5*pi*hcanopy*(wcanopy/2);
    AfTot = Af + Afcanopy; % Total frontal area [m^2] (0.408 from SolidWorks)
    
    % Drivetrain efficiency
    etaD = 0.88;
    
%Vortex
elseif bikeModel == 2
    % Rolling parameters
    Wpilot = 80*g; %N
    Whpv = 25*g; %N
    Wwheels = 4.5*g; %N
    Crr1 = 0.004;
    Crr2 = 2/3*4.1E-5*3.6; %s/m

    % Aerodynamic parameters
    L = 2.33; % Body length [m]
    Lnose = 0.9; % Nose length [m]
    h = 0.71; % Body height [m]
    w = 0.45; % Body width [m]
    hcanopy = 0.15; % Canopy height [m]
    wcanopy = 0.25; % Canopy width [m]
    xt = 0.7; % Transition point [m]
    Af = pi*(h/2)*(w/2); % Frontal area [m^2]
    Aside = 0.5*pi*Lnose*(h/2) + (L-Lnose)*h; % Side view
    Awet = (0.037*Af^2 + 0.02*Af + 1)*Aside*2; % Wetted area
    Awet = 4.3; % Wetted area from SolidWorks
    Afcanopy = 0.5*pi*hcanopy*(wcanopy/2);
    AfTot = 0.325; % Total frontal area from SolidWorks

    % Drivetrain efficiency
    etaD = 0.88;
    
    %185 at 60 kph
    %225 at 70 kph
    %306 at 80 kph
    
%Bluenose
elseif bikeModel == 3
    % Rolling parameters
    Wpilot = 80*g; %N
    Whpv = 20*g; %N
    Wwheels = 2*g; %N
    Crr1 = 0.00273; 
    Crr2 = (2/3*4.1E-5*3.6); %s/m

    % Aerodynamic parameters
    L = 2.6; % Body length [m]
    Lnose = 1; % Nose length [m]
    h = 0.8; % Body height [m]
    w = 0.56; % Body width [m]
    hcanopy = 0; % Canopy height [m]
    wcanopy = 0; % Canopy width [m]
    xt = 1.5; % Transition point [m]
    Af = pi*(h/2)*(w/2); % Frontal area [m^2]
    Aside = 0.5*pi*Lnose*(h/2) + (L-Lnose)*h; % Side view
    Awet = (0.037*Af^2 + 0.02*Af + 1)*Aside*2; % Wetted area
    Awet = 4.26; % Wetted area from SolidWorks
    Afcanopy = 0.5*pi*hcanopy*(wcanopy/2);
    AfTot = 0.325; % Total frontal area from SolidWorks
    
    % Drivetrain efficiency
    etaD = 0.92;

%Eta
elseif bikeModel == 4
    % Rolling parameters
    Wpilot = 80*g; %N
    Whpv = 20*g; %N
    Wwheels = 2*g; %N
    Crr1 = 0.0015; 
    Crr2 = 2/3*4.1E-5*3.6; %s/m

    % Aerodynamic parameters
    
    %Baseline (wide nose, short tail)
    L = 2.7; % Body length [m]
    Lnose = 0.9; % Nose length [m]
    h = 0.7; % Body height [m]
    w = 0.45; % Body width [m]
    hcanopy = 0; % Canopy height [m]
    wcanopy = 0; % Canopy width [m]
    xt = 1.9; % Transition point [m]
    
    %Long tail
    L = L+0.08; %add 8cm to tail (lower because it is low drag area
    %Upright
    h = h+0.05; %upright body position
    %Wide Nose
    L = L+0.1; %subtract 10cm to nose, Trip effectively moves 10cm forward
    Lnose = Lnose+0.1;
    
    Af = pi*(h/2)*(w/2); % Frontal area [m^2]
    Aside = 0.5*pi*Lnose*(h/2) + (L-Lnose)*h; % Side view
    Awet = (0.037*Af^2 + 0.02*Af + 1)*Aside*2; % Wetted area
    Afcanopy = 0.5*pi*hcanopy*(wcanopy/2);
    AfTot = 0.325; % Total frontal area from SolidWorks
    
    % Drivetrain efficiency
    etaD = 0.96334;
end

W = Wpilot + Whpv + Wwheels; %N
M = W/g; %kg
Mwheels = Wwheels/g; %kg

% Power calculation
for i = 1:1:Npts
    U = Uset(i);
    q = 0.5*rho*U^2;
    
    % Rolling drag
    Crr(i) = Crr1 + Crr2*U;
    Droll(i) = Crr(i)*W;
    Proll(i) = Droll(i)*U;
    
    % Flat plate drag
    Dflam = 1.328*h*q*sqrt(mu/U/rho)*sqrt(xt); %Laminar drag
    deltalamxt = 5*sqrt(mu/U/rho)*sqrt(xt); %Lam BL thickness at xt
    deltaturbxt = 0.13/0.097*deltalamxt; %Turb BL thickness at xt
    xdel = (deltaturbxt/0.375*(U*rho/mu)^0.2)^(1/0.8);
    x0 = xt - xdel; %imaginary turb start
    Dfturb = 0.0576/0.8*h*q*(mu/U/rho)^0.2*((L-x0)^0.8 - xdel^0.8);
    Cfflat(i) = (Dflam + Dfturb)/(q*h*L);
    if mode == 1
        Nx = 100;
        for j = 1:Nx
            x(j) = (j-1)/(Nx-1)*L;
            if x <= xt
                del(j) = 5*sqrt(mu/U/rho)*x(j)^0.5;
                Ctau(j) = 0.664*sqrt(mu/U/rho)*x(j)^(-0.5);
            else
                del(j) = 0.375*(mu/U/rho)^0.2*(x(j)-x0)^0.8;
                Ctau(j) = 0.0576*(mu/U/rho)^0.2*(x(j)-x0)^(-0.2);
            end
        end
        figure
        plot(x, del*1000);
        title('Boundary Layer Thickness vs. x');
        xlabel('x (m)');
        ylabel('delta (mm)');
        figure
        plot(x, Ctau);
        title('Friction Coefficient vs. x');
        xlabel('x (m)');
        ylabel('C_{tau}');
    end
    
    % Body drag
    Cdwet(i) = Cfflat(i)*(1 + 1.8*(Af^0.75)/(L^1.5) + 39*(Af^3)/(L^6));
    CdAbody(i) = Cdwet(i)*Awet;
    
    % Front wheel drag (full solar car wheel sticking out)
    %CdAfwheel(i) = 0.019; %no fairing
    %CdAfwheel(i) = 0.012; %half fairing
    %CdAfwheel(i) = 0.010; %square full fairing
    %CdAfwheel(i) = 0.006; %swept full fairing
    %CdAfwheel(i) = CdAfwheel(i) + 0.01; %if steering not sealed
    %Our wheel show about half the size
    %Dan: no wheel well + 0.001
    %Dan: no fairing + 0.004
    
    if bikeModel == 1
        CdAfwheel(i) = 0.010; %Fairing: Swept, half sealed by fairing, quite large
        %CdAfwheel(i) = 0.014; %No fairing, not sealed
    elseif bikeModel == 2
        CdAfwheel(i) = 0.003; %Full fairing, very small, sealed, wheel well
    elseif bikeModel == 3
        CdAfwheel(i) = 0.0025; %Full fairing, very small, sealed, wheel well
    elseif bikeModel == 4
        CdAfwheel(i) = 0.002; %Full fairing, very small, sealed, wheel well
    end
   
    
    % Rear wheel drag (full solar car wheel sticking out)
    %CdArwheel(i) = 0.019; %no fairing
    %CdArwheel(i) = 0.012; %half fairing
    %CdArwheel(i) = 0.010; %square full fairing
    %CdArwheel(i) = 0.006; %swept full fairing
    
    if bikeModel == 1
        CdArwheel(i) = 0.003; %With wheel well, Swept, quite large
        %CdArwheel(i) = 0.004; %No fairing, not sealed
    elseif bikeModel == 2
        CdArwheel(i) = 0.003; %Full fairing, very small, sealed, wheel well
    elseif bikeModel == 3
        CdArwheel(i) = 0.0025; %Full fairing, very small, sealed, wheel well
    elseif bikeModel == 4
        CdArwheel(i) = 0.002; %Full fairing, very small, sealed, wheel well
    end
    
    % Canopy drag
    %Cdcanopy = 0.17; %blunt nose and tail
    %Cdcanopy = 0.12; %swept nose, blunt tail
    %Cdcanopy = 0.07; %blunt nose, swept tail
    Cdcanopy = 0.05; %swept nose, swept tail
    CdAcanopy(i) = Cdcanopy*Afcanopy;
    
    % Uncleanliness drag
    if bikeModel == 1
        CdAunclean(i) = 0.001; %Smooth surface, only one horizontal seam
    elseif bikeModel == 2
        CdAunclean(i) = 0.003; %Several vertical cuts, bad front fairing edges
    elseif bikeModel == 3
        CdAunclean(i) = 0.001; %Smooth surface, few seams
    elseif bikeModel == 4
        CdAunclean(i) = 0.001; %Smooth surface, few seams
    end
    
    % Total aerodynamic drag
    CdA(i) = CdAbody(i) + CdAfwheel(i) + CdArwheel(i) + CdAcanopy(i) + CdAunclean(i);
    Cd(i) = CdA(i)/AfTot;
    Daero(i) = q*CdA(i);
    if U == 0
        Daero(i) = 0;
    end
    Paero(i) = Daero(i)*U;
    
    % Slope Power
    if BM
        Dslope(i) = g*0.0066*M;
        Pslope(i) = Dslope(i)*U;
    end
    
    % Total drag
    if BM
        D(i) = Droll(i) + Daero(i) - Dslope(i);
        P(i) = Proll(i) + Paero(i) - Pslope(i);
    else
        D(i) = Droll(i) + Daero(i);
        P(i) = Proll(i) + Paero(i);
    end
        
end

if mode == 1
    
    %Output areas
    Af
    Aside
    Awet
    Afcanopy
    
    %Output CdA
    CdAbody
    CdAfwheel
    CdArwheel
    CdAcanopy
    CdA
    Cd
    Daero
    Paero
    Droll
    Proll
    D
    P
    
    
elseif mode == 2
    U = Uset*3.6; %kph
    Re = Uset*rho*L/mu;
    
    % Drag coefficient
    figure
    plot(U,Crr,U,Cfflat,U,Cdwet);
    title('Drag Coefficient vs. Velocity');
    legend('Rolling Drag, C_{rr}','Flat Plate Drag, C_{f,flat}','Skin Friction Drag, C_{d_{wet}}');
    
    % Drag areas
    figure
    plot(U,CdAbody,U,CdAfwheel,U,CdArwheel,U,CdAcanopy,U,CdAunclean,U,CdA);
    title('Drag Areas vs. Velocity');
    legend('Body','Front Wheel','Rear Wheel','Canopy','Unclean','Total');
    xlabel('Speed (km/hr)');
    ylabel('Drag Area (m^2)');
    
    if ~BM
        % Drag
        figure
        plot(U,Droll,U,Daero,U,D);
        title('Drag vs. Velocity');
        legend('Rolling Drag','Aerodynamic Drag','Total Drag');
        xlabel('Speed (km/hr)');
        ylabel('Drag (N)');

        % Power
        figure
        plot(U,Proll,U,Paero,U,P);
        title('Power vs. Velocity');
        legend('Rolling Power','Aerodynamic Power','Total Power');
        xlabel('Speed (km/hr)');
        ylabel('Power (W)');
    else
        % Drag
        figure
        plot(U,Droll,U,Daero,U,Dslope,U,D);
        title('Drag vs. Velocity');
        legend('Rolling Drag','Aerodynamic Drag','Slope Force','Total Drag');
        xlabel('Speed (km/hr)');
        ylabel('Drag (N)');

        % Power
        figure
        plot(U,Proll,U,Paero,U,Pslope,U,P);
        title('Power vs. Velocity');
        legend('Rolling Power','Aerodynamic Power','Slope Power','Total Power');
        xlabel('Speed (km/hr)');
        ylabel('Power (W)');
    end

% Sprint calculation
elseif mode == 3
    
    %Power profile (Todd 2013)
%     pProfile(1) = 200; %Sub Zone 1 160
%     pProfile(2) = 250;  %Zone 1
%     pProfile(3) = 350; %Zone 3
%     pProfile(4) = 500; %Zone 5
%     pProfile(5) = 650; %Zone 5+
%     pProfile(6) = pProfile(5);
    
    %Power profile (Trefor)
%     pProfile(1) = 150; %Sub Zone 1 160
%     pProfile(2) = 200;  %Zone 1
%     pProfile(3) = 300; %Zone 3
%     pProfile(4) = 450; %Zone 5
%     pProfile(5) = 600; %Zone 5+
%     pProfile(6) = pProfile(5);
    
%     Power profile (Todd 2014 Reclined)
%     pProfile(1) = 210; %Zone 1
%     pProfile(2) = 260;  %Zone 1
%     pProfile(3) = 370; %Zone 3
%     pProfile(4) = 530; %Zone 5
%     pProfile(5) = 680; %Zone 5+
%     pProfile(6) = pProfile(5);

%     %Power profile (Todd 2014 More Upright)
    pProfile(1) = 210; %Zone 1
    pProfile(2) = 270;  %Zone 1
    pProfile(3) = 380; %Zone 3
    pProfile(4) = 550; %Zone 5
    pProfile(5) = 700; %Zone 5+
    pProfile(6) = pProfile(5);
    
    pProfile(1) = 400; %Zone 1
    pProfile(2) = 400;  %Zone 1
    pProfile(3) = 400; %Zone 3
    pProfile(4) = 400; %Zone 5
    pProfile(5) = 400; %Zone 5+
    pProfile(6) = pProfile(5);
    
    %Power profile (Sebastian)
%     pProfile(1) = 375; %Zone 1
%     pProfile(2) = 400;  %Zone 1
%     pProfile(3) = 450; %Zone 3
%     pProfile(4) = 600; %Zone 5
%     pProfile(5) = 700; %Zone 5+
%     pProfile(6) = pProfile(5);

%     %Power profile (ASME course)
%     pProfile(1) = 800; %Zone 1
%     pProfile(2) = 150;  %Zone 1
%     pProfile(3) = 200; %Zone 3
%     pProfile(4) = 560; %Zone 5
%     pProfile(5) = 630; %Zone 5+
%     pProfile(6) = pProfile(5);
    
    %Time profile;
    tZone(1) = 150;
    tZone(2) = 80;%+120*0.1;
    tZone(3) = 60;
    tZone(4) = 30;
    tZone(5) = 20;
    tProfile(1) = 0;
    for i = 2:6
        tProfile(i) = tProfile(i-1) + tZone(i-1);
    end
    
    tStart = 0;
    plotflag = 1;
    simRun(tStart)
    
end   

function simRun(tStart)
    v(1) = 0;
    v2(1) = 0; %vel squared
    d(1) = 0;
    t(1) = 0;
    dt = 1; %seconds

    Ppedal(1) = interp1(tProfile,pProfile,tStart);
    Pdrag(1) = 0;
    Pnet(1) = Ppedal(1);
    i = 1;
    while t(i) < tProfile(end)-tStart
        i = i+1;
        t(i) = (i-1)*dt;
        Ppedal(i) = etaD*interp1(tProfile,pProfile,t(i)+tStart);
        Pdrag(i) = interp1(Uset,P,v(i-1));
        Pnet(i) = Ppedal(i) - Pdrag(i);
        v2(i) = v2(i-1) + 2/(M+Mwheels)*Pnet(i)*dt;
        v(i) = sqrt(v2(i));
        d(i) = d(i-1) + 0.5*(v(i-1) + v(i))*dt;

    end
    
    if plotflag
        % Drag race
        v(end)*3.6
        t(end)

        loaded = load('-mat','Figures and Data/BMdata.m');
        dData = loaded.d;
        UData = loaded.U;
        tData = loaded.t;

        figure
        plot(t,Ppedal,t,Pdrag,t,Pnet);
        title('Sprint Power');
        legend('Total Power','Drag Power','Acceleration Power','Location','NorthWest');
        ylabel('Power (Watts)');
        xlabel('Time (s)');
        
        figure
        plotyy(t,v*3.6,t,Ppedal);
        line(tData,UData*3.6);
        title('Power and Velocity vs. Time');
        ylabel('Velocity (km/h)');
        xlabel('Time (s)');
        
        figure
        plotyy(d/1000/1.6,v*3.6,d/1000/1.6,Ppedal);
        line(dData/1000/1.5,UData*3.6);
        title('Power and Velocity vs. Distance');
        ylabel('Velocity (km/h)');
        xlabel('Distance (Miles)');
        
        
        
      
    end
    
end


end


