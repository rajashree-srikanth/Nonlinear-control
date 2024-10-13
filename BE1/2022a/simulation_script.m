%**************************************************************************
%**************************************************************************
% Feedback linearization applied to helicopter control 
%**************************************************************************
%**************************************************************************

draw_plot = true;

%==========================================================================
% Run the section of the code that you want
%==========================================================================
% 'Open-loop'
% 'Feedback linearization'
% 'Acceleration control'
% 'Speed control'
% 'Position control'

%==========================================================================
% Data
%==========================================================================

% Helicopter dynamics
m = 10;
J = 0.2;
f = 0.1;
g = 9.81;

% Actuators
tau = 0.1;
Lmin = 0; Lmax = 200;  
Tmin = -0.035; Tmax = 0.035;

%%
%==========================================================================
% Open-loop simulation
%==========================================================================

% Simulation
Tsim = 30;
DTsim = 0.01;

% Input demand on the Lift
d1_step_start = 5;
d1_step_stop  = Tsim+1;
d1_step_initial_value = m*g;
d1_step_final_value = m*g*1.1;

% Input demand on the Torque
d2_step_start = 10;
d2_step_stop  = Tsim+1;
d2_step_initial_value = 0;
d2_step_final_value = 0.01;

% Simulation
sim('simulation_model_1');

% Plots
if draw_plot
    figure;
    figtitle='Open-loop response';
    subplot(5,1,1); plot(t,Ld,t,L,'linewidth',2); grid on; legend('L_d','L'); ylabel('Lift');
    subplot(5,1,2); plot(t,Td,t,T,'linewidth',2); grid on; legend('T_d','T'); ylabel('Torque');
    subplot(5,1,3); plot(t,gx,'linewidth',2); grid on; legend('\gamma_x'); ylabel('Hori. Acc.');
    subplot(5,1,4); plot(t,gz,'linewidth',2); grid on; legend('\gamma_z'); ylabel('Vert. Acc.');
    subplot(5,1,5); plot(t,q,'linewidth',2); grid on; legend('q'); ylabel('Pitch rate');
    subplot(5,1,1); hold on; title(figtitle);
end

return

%%
%==========================================================================
% Feedback linearization (all poles at zero)
%==========================================================================

% Simulation
Tsim = 30;
DTsim = 0.01;

% Input demand on the second derivative of the x acceleration
d1_step_start = 5;
d1_step_stop  = Tsim+1;
d1_step_initial_value = 0;
d1_step_final_value = 2*(0.1*g)/Tsim^2;

% Input demand on the second derivative of the z acceleration
d2_step_start = 10;
d2_step_stop  = Tsim+1;
d2_step_initial_value = 0;
d2_step_final_value = 2*(g)/Tsim^2;

% Simulation
sim('simulation_model_2');

% [TODO]
gx = gx(:);
gz = gz(:);

% Plots
if draw_plot
    figure;
    figtitle='Inversion';
    subplot(4,1,1); plot(t,wx,'linewidth',2); grid on; hold on; plot(t,wx1,'linewidth',2); 
    legend('$\ddot{\gamma}_x$','$d^2/dt^2({\gamma}_x)$','interpreter','latex'); 
    subplot(4,1,2); plot(t,wz,'linewidth',2); grid on; hold on; plot(t,wz1,'linewidth',2); 
    legend('$\ddot{\gamma}_z$','$d^2/dt^2({\gamma}_z)$','interpreter','latex'); 
    subplot(4,1,3); plot(t,gx,'linewidth',2); grid on; legend('\gamma_x'); ylabel('Hori. Acc.');
    subplot(4,1,4); plot(t,gz,'linewidth',2); grid on; legend('\gamma_z'); ylabel('Vert. Acc.');
    subplot(4,1,1); hold on; title(figtitle);
end

%%
%==========================================================================
% Acceleration control
%==========================================================================

% Simulation
Tsim = 30;
DTsim = 0.01;

% Input demand on the x acceleration
d1_step_start = 5;
d1_step_stop  = Tsim+1;
d1_step_initial_value = 0;
d1_step_final_value = 0.1*g;

% Input demand on the z acceleration
d2_step_start = 10;
d2_step_stop  = Tsim+1;
d2_step_initial_value = 0;
d2_step_final_value = g;

% Stabilizing feedback gains (PD control)
xi = 0.7; 
Trep = 1;

% [TODO] Controller design
omega_ = 3/(xi*Trep);

kp_gx = [omega_^2];
kd_gx = [2*xi*omega_];
kp_gz = [omega_^2];
kd_gz = [2*xi*omega_];

% Simulation
sim('simulation_model_3');

% [TODO]
Ld = Ld(:);
Td = Td(:);
L = L(:);
T = T(:);
gx = gx(:);
gz = gz(:);
q = q(:);

% Plots
if draw_plot
    figure;
    figtitle='Acceleration control';
    subplot(5,1,1); plot(t,Ld,t,L,'linewidth',2); grid on; legend('L_d','L'); ylabel('Lift');
    subplot(5,1,2); plot(t,Td,t,T,'linewidth',2); grid on; legend('T_d','T'); ylabel('Torque');
    subplot(5,1,3); plot(t,gx,t,gxd,'linewidth',2); grid on; legend('\gamma_x','{\gamma_x^*}'); ylabel('Hori. Acc.');
    subplot(5,1,4); plot(t,gz,t,gzd,'linewidth',2); grid on; legend('\gamma_z','{\gamma_z^*}'); ylabel('Vert. Acc.');
    subplot(5,1,5); plot(t,q,'linewidth',2); grid on; legend('q');
    subplot(5,1,1); hold on; title(figtitle);
end

%%
%==========================================================================
% Velocity control
%==========================================================================

% Feedback gains (P control)
Trep = 3;

%
% Trep = 1; % not good
% Trep = 2; % not good
% Trep = 4; % not good

kp_vx = [3/Trep]; % 2/Trap
kp_vz = [3/Trep]; % 2/Trep

% Simulation
sim('simulation_model_4');

% [TODO]
Ld = Ld(:);
Td = Td(:);
L = L(:);
T = T(:);
vx = vx(:);
vz = vz(:);
q = q(:);

% Plots
if draw_plot
    figure;
    figtitle='Velocity control';
    subplot(5,1,1); plot(t,Ld,t,L,'linewidth',2); grid on; legend('L_d','L'); ylabel('Lift');
    subplot(5,1,2); plot(t,Td,t,T,'linewidth',2); grid on; legend('T_d','T'); ylabel('Torque');
    subplot(5,1,3); plot(t,vx,t,vxd,'linewidth',2); grid on; legend('v_x','v_x^*'); ylabel('Hori. Vel.');
    subplot(5,1,4); plot(t,vz,t,vzd,'linewidth',2); grid on; legend('v_z','v_z^*'); ylabel('Vert. Vel.');
    subplot(5,1,5); plot(t,q,'linewidth',2); grid on; legend('q'); ylabel('Pitch rate');
    subplot(5,1,1); hold on; title(figtitle);
end

%%
%==========================================================================
% Position control
%==========================================================================

% Feedback gains (PI control)
xi = 0.7;  % temporary
Trep = 4.2;

% record
% Trep = 3; % diverge
% Trep = 4.5; % good
% Trep = 5; % good

kp_x = [3/Trep];
ki_x = [0.00001];
kp_z = [3/Trep];
ki_z = [0.00001];

% Simulation
sim('simulation_model_5');

% [TODO]
Ld = Ld(:);
Td = Td(:);
L = L(:);
T = T(:);
x = x(:);
z = z(:);
q = q(:);

% Plots
if draw_plot
    figure;
    figtitle='Position control';
    subplot(5,1,1); plot(t,Ld,t,L,'linewidth',2); grid on; legend('L_d','L'); ylabel('Lift');
    subplot(5,1,2); plot(t,Td,t,T,'linewidth',2); grid on; legend('T_d','T'); ylabel('Torque');
    subplot(5,1,3); plot(t,x,t,xd,'linewidth',2); grid on; legend('x','x^*'); ylabel('Hori. Pos.');
    subplot(5,1,4); plot(t,z,t,zd,'linewidth',2); grid on; legend('z','z^*'); ylabel('Vert. Pos.');
    subplot(5,1,5); plot(t,q,'linewidth',2); grid on; legend('q'); ylabel('Pitch rate');
    subplot(5,1,1); hold on; title(figtitle);
end
