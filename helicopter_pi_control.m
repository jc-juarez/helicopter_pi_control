% Tecnológico de Monterrey
% Computerized Control
% Homework - Helicopter with PI Control
% PI Control with three Rules: Bilinear, Forward Integration and Backward Integration
% Bilinear: m(k) = 1/2 * ((2 * kp + ki * T) * e(k) + (ki * T - 2 * kp) * e(k-1)) + m(k-1)
% Forward: m(k) = kp * e(k) + (-kp + ki * T) * e(k-1) + m(k-1)
% Backward: m(k) = (kp + ki * T) * e(k) -kp * e(k-1) + m(k-1)

% Predefined values
clc;
T = 0.1;
n = 100;
r = ones(size(1:n));

% Tuning Values

kp = 0.5;
ki = 1;

% Initial Values for non Zero Indexing (Only c(k) and e(k))

c(1) = 0;
e(1) = r(1) - c(1);

c(2) = 0.04004 * m(1) + 1.4563 * c(1);
e(2) = r(2) - c(2);

% -------------------------------------------------

% 1 - PI with Bilinear Transformation

% Initial Values for m(k)

m(1) = 1/2 * ((2 * kp + ki * T) * e(1));

m(2) = 1/2 * ((2 * kp + ki * T) * e(2) + (ki * T - 2 * kp) * e(1)) + m(1);

% Loop

for k=3:1:n
    c(k) = 0.04004 * m(k-1) + 0.0322 * m(k-2) + 1.4563 * c(k-1) - 0.522 * c(k-2);
    e(k) = r(k) - c(k);
    m(k) = 1/2 * ((2 * kp + ki * T) * e(k) + (ki * T - 2 * kp) * e(k-1)) + m(k-1);
end

t = 1*(1:n);
figure(1);
subplot(3,1,1),plot(t,r,'m',t,c,'b--');
xlabel('Iterations');
ylabel('Output');
legend({'r(k)','c(k)'},'Location','southeast')
title('Output Response');
subplot(3,1,2),plot(t,m,'b');
xlabel('Iterations');
ylabel('Output');
legend({'m(k)'},'Location','southeast')
title('Controller Signal');
subplot(3,1,3),plot(t,e,'r');
xlabel('Iterations');
ylabel('Output');
legend({'e(k)'},'Location','southeast')
title('Error Signal');


% -----------------------------------------------

% 2 - PI with Forward Integration Rule

% Initial Values for m(k)

m(1) = kp * e(1);

m(2) = kp * e(2) + (-kp + ki * T) * e(1) + m(1);

% Loop

for k=3:1:n
    c(k) = 0.04004 * m(k-1) + 0.0322 * m(k-2) + 1.4563 * c(k-1) - 0.522 * c(k-2);
    e(k) = r(k) - c(k);
    m(k) = kp * e(k) + (-kp + ki * T) * e(k-1) + m(k-1);
end

figure(2);
subplot(3,1,1),plot(t,r,'m',t,c,'b--');
xlabel('Iterations');
ylabel('Output');
legend({'r(k)','c(k)'},'Location','southeast')
title('Output Response');
subplot(3,1,2),plot(t,m,'b');
xlabel('Iterations');
ylabel('Output');
legend({'m(k)'},'Location','southeast')
title('Controller Signal');
subplot(3,1,3),plot(t,e,'r');
xlabel('Iterations');
ylabel('Output');
legend({'e(k)'},'Location','southeast')
title('Error Signal');

% -----------------------------------------------

% 3 - PI with Backward Integration Rule

% Initial Values for m(k)

m(1) = (kp + ki * T) * e(1);

m(2) = (kp + ki * T) * e(2) -kp * e(1) + m(1);

% Loop

for k=3:1:n
    c(k) = 0.04004 * m(k-1) + 0.0322 * m(k-2) + 1.4563 * c(k-1) - 0.522 * c(k-2);
    e(k) = r(k) - c(k);
    m(k) = (kp + ki * T) * e(k) -kp * e(k-1) + m(k-1);
end

figure(3);
subplot(3,1,1),plot(t,r,'m',t,c,'b--');
xlabel('Iterations');
ylabel('Output');
legend({'r(k)','c(k)'},'Location','southeast')
title('Output Response');
subplot(3,1,2),plot(t,m,'b');
xlabel('Iterations');
ylabel('Output');
legend({'m(k)'},'Location','southeast')
title('Controller Signal');
subplot(3,1,3),plot(t,e,'r');
xlabel('Iterations');
ylabel('Output');
legend({'e(k)'},'Location','southeast')
title('Error Signal');
