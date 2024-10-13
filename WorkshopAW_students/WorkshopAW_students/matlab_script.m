%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    ANTI-WINDUP DESIGN & UNSTABLE LIMIT-CYCLE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear 
close all

%% Question 1   - PID controller design
% -------------------------------------
% >>> Insert your code here to find the values Kp, Ki, Kd below <<<<

% Check results
Ki=0.5372; Kp = 1.4968; Kd = 1.6797; % to be found ! 
refmag=1; Tsim=10; 
[a,b,c,d]=linmod('scheme1_sim'); % saturation replaced by gain=1 when linearizing
damp(a)


%% Question  2 - Simulation-based analysis of saturation effects
% ---------------------------------------------------------------

% a) nominal case : saturation not active
refmag=0.3;
Tsim=10;
[t,x,y]=sim('scheme1_sim');
figure(1); clf; 
subplot(211); plot(t,y(:,1)); grid; ylabel('Output y');
title('Ref = 0.3 : nominal behavior');
subplot(212); plot(t,y(:,2)); hold on; grid; ylabel('Control u');
plot([0 Tsim],[0.1 0.1],'r','LineWidth',2);
plot([0 Tsim],-[0.1 0.1],'r','LineWidth',2);

% b) unstable case : saturation highly active
refmag=1;
Tsim=50;
[t,x,y]=sim('scheme1_sim');
figure(2); clf; 
subplot(211); plot(t,y(:,1)); grid; ylabel('Output y');
title('Ref = 1 : instability detected');
subplot(212); plot(t,y(:,2)); hold on; grid; ylabel('Control u');
plot([0 Tsim],[0.1 0.1],'r','LineWidth',2);
plot([0 Tsim],-[0.1 0.1],'r','LineWidth',2);

% c) critical case : limit-cycle ?
refmag=xxxx;
Tsim=100;
[t,x,y]=sim('scheme1_sim');
figure(3); clf; 
subplot(211); plot(t,y(:,1)); grid;  ylabel('Output y');
subplot(212); plot(t,y(:,2)); hold on; grid; ylabel('Control u');
xlabel('Time (sec)');
plot([0 Tsim],[0.1 0.1],'r','LineWidth',2);
plot([0 Tsim],-[0.1 0.1],'r','LineWidth',2);

% d) limit-cycle characteristics
% x0sim = ? , wcsim = ?



%% Question  3 - Describing function analysis
% --------------------------------------------

% a) Simulink file : scheme1_analysis to be derived from scheme1_sim
[a,b,c,d]=linmod('scheme1_analysis');
sys=-ss(a,b,c,d); % M(s) viewed by the saturation with negative feedback

% b) Nyquist plot and critical locus
om_min=xxxx;
om_max=xxxx;
omeg=[om_min:0.01:om_max]; % no need to cover all frequencies
[re,im]=nyquist(sys,omeg);
re=squeeze(re); im=squeeze(im);
figure(4); clf; plot(re,im); grid; hold on;
plot([-9 -1],[0 0],'r','LineWidth',2);
xlabel('Re (M(j\omega)');
ylabel('Im (M(j\omega)');
title('Nyquist Plot and Critical Locus');


% c) LC chracateristics : x0DFA = ? , wcDFA = ?


% d) Ratio R = |M(3j*omc)/M(j*omc)| = ?
% comments ?


%% Question  4 - Anti-windup design
% ---------------------------------

% a) Simulink file : scheme1_analysis_aw to be defined
% b) Nyquist plots for different values of Kaw
% c) Simulation with anti-windup Kaw > Kawc




