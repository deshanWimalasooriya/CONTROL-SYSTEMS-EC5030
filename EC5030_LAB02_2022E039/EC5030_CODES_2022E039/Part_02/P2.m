% WIMALASOORIYA G.H.N.P.D.
% 2022E039
%% DC Motor Model

% Parameters
J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;

% Transfer Functions
aux = tf(K, conv([L R], [J b])); 
Gv = feedback(aux, K);           % Voltage to velocity (angular)
Ga = tf(1, [1 0]) * Gv;          % Voltage to angle

% Label input/output
Gv.InputName = 'Voltage';
Gv.OutputName = 'Velocity';
Ga.InputName = 'Voltage';
Ga.OutputName = 'Angle';

%% Convert to state-space and zero-pole-gain representations
ss_Gv = ss(Gv);
zpk_Gv = zpk(Gv);

ss_Ga = ss(Ga);
zpk_Ga = zpk(Ga);

disp('State-space form of Gv:');
ss_Gv
disp('Zero-pole-gain form of Gv:');
zpk_Gv

disp('State-space form of Ga:');
ss_Ga
disp('Zero-pole-gain form of Ga:');
zpk_Ga

%% Poles and Zeros
poles_Gv = pole(Gv);
zeros_Gv = zero(Gv);

poles_Ga = pole(Ga);
zeros_Ga = zero(Ga);

disp('Poles of Gv:');
disp(poles_Gv);
disp('Zeros of Gv:');
disp(zeros_Gv);

disp('Poles of Ga:');
disp(poles_Ga);
disp('Zeros of Ga:');
disp(zeros_Ga);

% System Stability
if all(real(poles_Gv) < 0)
    disp('Gv is stable.');
else
    disp('Gv is unstable.');
end
if all(real(poles_Ga) < 0)
    disp('Ga is stable.');
else
    disp('Ga is unstable.');
end










%% Controllability and Observability
[A, B, C, D] = tf2ss(Gv.num{1}, Gv.den{1});
Co = ctrb(A, B);
Ob = obsv(A, C);

is_controllable = rank(Co) == size(A,1);
is_observable = rank(Ob) == size(A,1);

disp(['Is the system controllable? ', mat2str(is_controllable)]);
disp(['Is the system observable? ', mat2str(is_observable)]);

%% Time and Frequency Domain Responses
figure(1);
step(Ga);
hold on;
step(Gv);
title('Step Responses');
legend('Angle (Ga)', 'Velocity (Gv)');
grid on;

figure(2);
impulse(Gv);
hold on;
impulse(Ga);
title('Impulse Responses');
legend('Velocity (Gv)', 'Angle (Ga)');
grid on;

figure(3);
bode(Ga);
hold on;
bode(Gv);
title('Bode Plots');
legend('Angle (Ga)', 'Velocity (Gv)');
grid on;
