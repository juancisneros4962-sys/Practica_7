% Parámetros del motor DC obtenidos del documento
Ra = 2;      
Kt = 0.01;   
b  = 0.0012; 
La = 0.023;  
Ke = 0.01;   
J  = 0.001;  

% Condiciones iniciales [Corriente (A), Velocidad angular (rad/s), Posición (rad)]
x0 = [0; 0; 0]; 

% Tiempo de simulación (de 0 a 5 segundos)
tspan = [0 5];

% Resolución del sistema de ecuaciones diferenciales usando ode45
[t, x] = ode45(@(t,x) motor_dc_ode(t, x, Ra, Kt, b, La, Ke, J), tspan, x0);

% Gráficas de los resultados
figure;
subplot(3,1,1);
plot(t, x(:,1), 'LineWidth', 1.5);
title('Corriente de Armadura (i_a)');
ylabel('Amperios'); grid on;

subplot(3,1,2);
plot(t, x(:,2), 'LineWidth', 1.5, 'Color', 'r');
title('Velocidad Angular (\omega_m)');
ylabel('rad/s'); grid on;

subplot(3,1,3);
plot(t, x(:,3), 'LineWidth', 1.5, 'Color', 'g');
title('Posición Angular (\theta_m)');
xlabel('Tiempo (s)'); ylabel('rad'); grid on;

% Función con las ecuaciones de estado
function dxdt = motor_dc_ode(t, x, Ra, Kt, b, La, Ke, J)
va = 12; % Voltaje de entrada constante (escalón de 12V)

% Asignación de estados
ia = x(1);
omega = x(2);

% Ecuaciones diferenciales derivadas de la Figura 1
dia_dt = (va - Ra*ia - Ke*omega) / La;
domega_dt = (Kt*ia - b*omega) / J;
dtheta_dt = omega;

% Vector de derivadas
dxdt = [dia_dt; domega_dt; dtheta_dt];
end