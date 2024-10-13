%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PART 1 : ANTI-WINDUP DESIGN & UNSTABLE LIMIT-CYCLE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Question 1   - PID controller design
% -------------------------------------
% Ki=0.5372 , Kp = 1.4968, Kd = 1.6797
% the (non-placed) actuator dynamic is moved from -5 to -2.69 : ok
[a,b,c,d]=linmod('scheme1_design');
xi=sqrt(0.5); om=1;
ev=[-1 ; roots([1 2*xi*om om^2])];
vw=zeros(5,3);
for i=1:3
    vw(:,i)=null([a-ev(i)*eye(4) b]);
end
K=real(vw(end,:)/(c*vw(1:4,:)));
damp(a+b*K*c)
Ki=K(1); Kp=K(2); Kd=K(3);

%% Question  2 - Simulation-based analysis of saturation effects
% ---------------------------------------------------------------

%% a) nominal case : saturation not active
refmag=0.3;
Tsim=10;
[t,x,y]=sim('scheme1_sim');
figure(1); clf; 
subplot(211); plot(t,y(:,1)); grid; ylabel('Output y');
title('Ref = 0.3 : nominal behavior');
subplot(212); plot(t,y(:,2)); hold on; grid; ylabel('Control u');
plot([0 Tsim],[0.1 0.1],'r','LineWidth',2);
plot([0 Tsim],-[0.1 0.1],'r','LineWidth',2);

%% b) unstable case : saturation highly active
refmag=1;
Tsim=50;
[t,x,y]=sim('scheme1_sim');
figure(2); clf; 
subplot(211); plot(t,y(:,1)); grid; ylabel('Output y');
title('Ref = 1 : instability detected');
subplot(212); plot(t,y(:,2)); hold on; grid; ylabel('Control u');
plot([0 Tsim],[0.1 0.1],'r','LineWidth',2);
plot([0 Tsim],-[0.1 0.1],'r','LineWidth',2);

%% c) unstable limit-cycle detected
refmag=0.9977;
Tsim=100;
[t,x,y]=sim('scheme1_sim');
figure(3); clf; 
subplot(211); plot(t,y(:,1)); grid;  ylabel('Output y');
title('Ref = 0.997 : limit-cycle detected : Tc = 12.6 s');
subplot(212); plot(t,y(:,2)); hold on; grid; ylabel('Control u');
xlabel('Time (sec)');
plot([0 Tsim],[0.1 0.1],'r','LineWidth',2);
plot([0 Tsim],-[0.1 0.1],'r','LineWidth',2);

%% d) limit-cycle characteristics
% x0sim= 0.744, wcsim=0.496
plot([27.5 65.5],[0.744 0.744],'k','LineWidth',2);
plot([27.5 27.5],[0.65 0.8],'k','LineWidth',2);
plot([65.5 65.5],[0.65 0.8],'k','LineWidth',2);
text(45,0.85,'3*Tc=38');
x0sim=0.744; Tcsim=(65.5-27.5)/3;
wcsim=2*pi/Tcsim;  % 


%% Question  3 - Describing function analysis
% --------------------------------------------

% a) Simulink file : scheme1_analysis
[a,b,c,d]=linmod('scheme1_analysis');
sys=-ss(a,b,c,d); % M(s) viewed by the saturation with negative feedback

% b) Nyquist plot and critical locus
omeg=[0.4:0.01:10]; % no need to cover all frequencies
[re,im]=nyquist(sys,omeg);
re=squeeze(re); im=squeeze(im);
figure(4); clf; plot(re,im); grid; hold on;
plot([-9 -1],[0 0],'r','LineWidth',2);
xlabel('Re (M(j\omega)');
ylabel('Im (M(j\omega)');
title('Nyquist Plot and Critical Locus');
text(-5.5,0.6,'Intersection for Re=-5.21');
text(-4.5,0.35,'=>  N(x0) = 0.1919');

%% c) LC chracateristics : x0DFA=0.663, wcDFA=0.53
% N = 4*L / pi*x0 => x0 = 4*L/pi*N
x0DFA=4*0.1/(pi*0.1919);
figure(5); clf; plot(omeg,im); grid; 
hold on; plot([0 10],[0 0],'r','LineWidth',1);
wcDFA=0.531;
plot(wcDFA+0.0001*1i,'r*'); title('\omega_c=0.531');
xlabel('Pulsation (rad/s)');
ylabel('Im (M(j\omega)');

%% d) Ratio R = |M(3j*omc)/M(j*omc)| = 0.2 => not excellent
% this confirms the observed differences between simulation & dfa
%  --------------------------------------
%       |    SIM     |  DFA      | delta
%  --------------------------------------
%  wc   |    0.496   |   0.531   |   +7%
%  x0   |    0.744   |   0.663   |  -11%
% ---------------------------------------
R=abs(freqresp(sys,3*wcDFA)/freqresp(sys,wcDFA));


%% Question  4 - Anti-windup design
% ---------------------------------

% a) Simulink file : scheme1_analysis_aw
% b) Nyquist plots for Kaw = 0 , 0.1, ...,1
%    kawc=0.25
figure(6); clf;
omeg=[0.3:0.01:10];
for Kaw=0:0.1:1
    [a,b,c,d]=linmod('scheme1_analysis_aw');
    sys=-ss(a,b,c,d);
    [re,im]=nyquist(sys,omeg);
    re=squeeze(re); im=squeeze(im);
    plot(re,im); hold on;
end
grid
figure(7); clf;
omeg=[0.25:0.01:10];
icoul=0; coul='bmk';
for Kaw=[0.2 0.25 0.3]
    icoul=icoul+1;
    [a,b,c,d]=linmod('scheme1_analysis_aw');
    sys=-ss(a,b,c,d);
    [re,im]=nyquist(sys,omeg);
    re=squeeze(re); im=squeeze(im);
    plot(re,im,coul(icoul)); hold on;
end
plot([-35 -1],[0 0],'r','LineWidth',2);
legend('Kaw=0.2','Kaw=0.25','Kaw=0.3','Location','NorthEast');
grid

% c) Simulation with anti-windup Kaw > Kawc
% we choose Kaw >> Kawc : the higher the gain, the better here. In this
% case the integral action is freezed rather than just limited. This is the
% Krikelis proposed scheme.
Kaw=10; 
Tsim=50;
[t,x,y]=sim('scheme1_sim_aw');
figure(8); clf; 
subplot(211); plot(t,y(:,1),'LineWidth',2); grid; ylabel('Output y');
title('Ref = 1 : simulation with anti-windup works fine');
subplot(212); plot(t,y(:,2)); hold on; grid; ylabel('Control u');
plot([0 Tsim],[0.1 0.1],'r','LineWidth',2);
plot([0 Tsim],-[0.1 0.1],'r','LineWidth',2);


