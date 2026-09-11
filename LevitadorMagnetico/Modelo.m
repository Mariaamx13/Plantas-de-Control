% Parámetros del sistema
x1 = 0.036;
m = 0.00452;
kf = 1.1971e-7;
kf_prima = 3.8195e-5;
u = 2.45;
R = 2.44;
L = 0.015;
b = 0.03;
kv = 0;
g = 9.81;

% Coeficientes matriz A
A21 = ((4*kf*(u/R))/(m*(x1^5))) + (2*kf_prima/(m*(x1^3)));
A22 = 0;
A23 = -(kf/m)*(1/(x1^4));
A31 = 0;
A32 = -(kv/L)*(x1/(((b^2)+(x1^2))^(5/2)));
A33 = -R/L;

% Matrices del sistema
A = [0, 1, 0; A21, A22, A23; A31, A32, A33];
B = [0; 0; (1/L)];
C = [1, 0, 0];
D = [0];

% Función de transferencia continua
[num, den] = ss2tf(A, B, C, D);
den = den/den(1);
Ys = tf(num, den);

disp('Polos en tiempo continuo:')
pole(Ys)

% Período de muestreo
T = 0.022;

% Función de transferencia discreta
Yz = c2d(Ys, T);

disp('Polos en tiempo discreto:')
pole(Yz)

% Lazo abierto con sensor
ks = -62107.5;
Lz = Yz * ks;

% Root Locus
figure
rlocus(Lz)
zgrid
title('Root Locus')