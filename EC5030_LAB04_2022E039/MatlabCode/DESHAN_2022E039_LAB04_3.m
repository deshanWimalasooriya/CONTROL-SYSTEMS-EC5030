% Given values
R = 52e3;        % Ohms
C1 = 560e-12;    % Farads
C2 = 6e-12;      % Farads

% Numerator and denominator coefficients
num = [R*C1 0];
den = [R*(C1 + C2) 1];

% Create transfer function
H = tf(num, den);

% Display
disp(H)

% Plot step or frequency response if needed
step(H)
% bode(H)
