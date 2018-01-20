function BOBB(control_trise, control_zeta, observer_gain, use_lqr, Q, R, use_kalman,kalman_Q, kalman_R, g1, g2);
%where did the 14.3 come from
F=[0 1;0 0]; G= [0 -9.3]'; H = [1 0]; J = 0;

sysC = ss(F, G, H, J); %continuous
%zpk(sysC);
%pause;
T = 0.008; %sample time in ms
sysD = c2d(sysC,T,'zoh');
[phi,gamma,H,J]=ssdata(sysD);
%pause
I = [1 0;0 1];
N = inv([phi-I gamma; H J])*[0 0 1]';
Nx = [N(1,1) N(2,1)]';
Nu = N(3,1);
%pause

omega_n = 1.8/control_trise;
sigma = control_zeta*omega_n;
omega_d = omega_n*sqrt(1-control_zeta^2);
pk1 = -1*sigma-i*omega_d;
pk2 = -1*sigma+i*omega_d;
s_pk = [pk1 pk2];
z_pk = exp(s_pk*T);
K = place(phi,gamma,z_pk)
Q = [Q 0;0 0];
[K_lqr,S_lqr,E_lqr] = dlqr(phi,gamma,Q,R,0);
K_lqr

%pause
s_pl = observer_gain*s_pk;
z_pl = exp(s_pl*T);
L = place(phi', H', z_pl)'
%pause
%kalman_sysC = ss(F, [G G], H, [J J]); %continuous
%kalman_sysD = c2d(kalman_sysC,T,'zoh');
%[kalman_sys,L_kalman,P_kalman,M_kalman,Z_kalman] = kalman(sysD, kalman_Q, kalman_R);
%L_kalman
%M_kalman %use this for predictor calculations
%L_kalman = M_kalman;
%pause
[M_kalman,P_kalman,Z_kalman, E_kalman] = dlqe(sysD.a, [g1 g2]', sysD.c, kalman_Q, kalman_R);

if (use_lqr == 1)
K = K_lqr;
end
if (use_kalman == 1)
L = M_kalman;
end
N_bar = Nu + K*Nx;
M1 = phi-gamma*K-L*H*phi+L*H*gamma*K;
M3 = (gamma+L*H*gamma)*N_bar;

M1
L
M3
K
N_bar

fid = fopen('C:\apps\atmel\source_code\atmega328\atmega328_BOBB\ball-on-beam\state-space-v2\matrix.txt', 'wt');
fprintf(fid, 'double M1[2][2] = {{%5.4f,%5.4f},{%5.4f,%5.4f}};\n', M1(1,1), M1(1,2), M1(2,1), M1(2,2));
fprintf(fid, 'double L[2] = {%5.4f,%5.4f};\n', L(1), L(2));
fprintf(fid, 'double M3[2] = {%5.4f,%5.4f};\n', M3(1), M3(2));
fprintf(fid, 'double KK[2] = {%5.4f,%5.4f};\n', K(1), K(2));
fprintf(fid, 'double N_bar = %5.4f;\n', N_bar);
fclose(fid);




