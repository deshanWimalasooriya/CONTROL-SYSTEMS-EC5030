%WIJESINGHE H.M.S.S.
%% DC Motor Model
J=0.01;
b=0.1;
K=0.01;
R=1;
L=0.5;


%% Method(1)
aux = tf(K,conv([L R],[J b])); % or aux = tf(K,[L R])*tf(1,[J b]);
Gv = feedback(aux,K); % transfer function for voltage and angle
Ga = tf(1,[1 0])*Gv; % transfer function for voltage and angular velocity
%% Method (2)
s = tf([1 0],1);
Gv = K/((L*s + R)*(J*s + b) + K^2); % transfer function for voltage and angle
Ga = Gv/s; % transfer function for voltage and angular velocity
% Labeling the input and output
Gv.InputName = 'Voltage';
Gv.OutputName = 'Velocity';
Ga.InputName = 'Voltage';
Ga.OutputName = 'Angle';

%% Plotting the time and frequency domain responses

subplot
figure(1);
subplot(3,2,1);
step(Ga);
hold on

step(Gv);
subplot(3,2,2);
impulse(Ga);
hold on

impulse(Gv);
subplot(3,2,3);
bode(Ga);
hold on
bode(Gv)

%% Adding a PID controller
Kp = 1;
Ki = 0.8;
Kd = 0.3;
C = tf([Kd Kp Ki],[1 0]);
C = tf([Kd Kp Ki],[1 0]); % (Kd*s^2+Kp*s+Ki)/s
Gc = feedback(Ga*C,1);

subplot(3,2,4);
step(Gc)

subplot(3,2,5);
rlocus(Ga*C);
Kp = rlocfind(Ga*C);
Gc1 = feedback(Ga*C*Kp,1);

subplot(3,2,6);
step(Gc1)