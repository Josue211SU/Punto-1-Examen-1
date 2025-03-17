clc; clear; close all;

% Conectar a la cámara y establecer la resolución
cam = webcam(2);
cam.Resolution = '1920x1080';

% Conectar al dispositivo Bluetooth ESP32
bt = bluetooth("Carrito_Mono_2", 1);
if isempty(bt)
    error('No se pudo conectar al dispositivo Bluetooth.');
else
    disp('Conexión Bluetooth establecida.');
end

% Definir el punto objetivo en la imagen (ajustado a la resolución 1920x1080)
alpha_d_pixels = [960, 540];

% Configurar la figura para visualizar el video en tiempo real
hFig = figure('Name', 'Video en tiempo real con detección de AprilTag');
hAx = axes('Parent', hFig);
title(hAx, 'Video en tiempo real con detección de AprilTag');

% Parámetros del controlador
k = 1;        % Ganancia del control proporcional
L = 0.068;    % Distancia entre el centro de masa y el eje de las ruedas
r = 0.032;    % Radio de las ruedas
tol = 25;     % Tolerancia para detener el carrito
no = 0.5;     % Factor de ajuste de velocidad
d_alpha = 0.04; % Distancia entre el centro del AprilTag y el punto alpha (4 cm)

% Bucle principal de control
while ishandle(hFig)  % Mientras la ventana de video esté abierta
    % Capturar imagen desde la cámara
    img = snapshot(cam);
    grayImg = rgb2gray(img);
    
    % Detección de AprilTags en la imagen
    [ID, locs] = readAprilTag(grayImg, "tag16h5");
    tag_detected = false;

    % Procesar cada AprilTag detectado en la imagen
    for i = 1:numel(ID)
        currentID = ID(i);
        p1 = squeeze(locs(1, :, i));
        p2 = squeeze(locs(2, :, i));
        p3 = squeeze(locs(3, :, i));
        p4 = squeeze(locs(4, :, i));
        centro = mean([p1; p2; p3; p4], 1);
        Theta = -(atan2d(p2(2) - p1(2), p2(1) - p1(1)));

        % Verificar si el ID detectado corresponde al AprilTag de interés
        if currentID == 6
            % Validar la geometría del AprilTag detectado
            if ~validar_tag(p1, p2, p3, p4)
                continue;
            end
            
            % Convertir coordenadas a referencia con centro en (960, 540)
            x = centro(1) - 960;
            y = 540 - centro(2);
            th = deg2rad(Theta);
            tag_detected = true;

            % Calcular la escala de conversión de metros a píxeles
            escala_pixeles = norm(p1 - p3) / 0.1; % Relación tamaño real/píxeles
            L1_pixeles = d_alpha * escala_pixeles; % Convertir L1 a píxeles

            % Calcular el punto alpha desplazado hacia el frente del AprilTag
            alpha_x = centro(1) - L1_pixeles * sin(th); % Dirección en X
            alpha_y = centro(2) - L1_pixeles * cos(th); % Dirección en Y

            % Dibujar el AprilTag detectado en la imagen
            img = insertShape(img, 'Line', [p1; p2; p3; p4; p1], 'LineWidth', 2, 'Color', 'green');
            img = insertShape(img, 'Circle', [centro, 5], 'LineWidth', 2, 'Color', 'red'); % Centro del AprilTag
            
            % Dibujar el punto alpha en la imagen
            img = insertShape(img, 'Circle', [alpha_x, alpha_y, 5], 'LineWidth', 2, 'Color', 'yellow'); % Punto alpha

            % Insertar texto informativo en la imagen
            texto = sprintf("ID=%d\nX=%.1f, Y=%.1f, %c=%.1f°", currentID, centro(1), centro(2), char(952), Theta);
            img = insertText(img, [centro(1)+10, centro(2)-10], texto, 'FontSize', 14, 'TextColor', 'black', 'BoxOpacity', 0.6);
        end
    end

    % Dibujar el punto deseado en la imagen
    img = insertShape(img, 'Circle', [alpha_d_pixels, 5], 'LineWidth', 2, 'Color', 'blue');
    imshow(img, 'Parent', hAx);
    drawnow;

    % Si no se detectó el AprilTag correcto, detener el carrito
    if ~tag_detected
        write(bt, "0,0\n", "string");
        continue;
    end

    % Cálculo de la variable de salida basada en el punto alpha
    alpha = [alpha_x - 960; 540 - alpha_y];
    alpha_d = alpha_d_pixels' - [960; 540];

    % Verificar si el carrito ha llegado al punto deseado
    error_norm = norm(alpha_d - alpha);
    if error_norm < tol
        write(bt, "0,0\n", "string");
        break;
    end

    % Ley de control proporcional
    u_bar = -k * (alpha_d - alpha);
    
    % Linealización inversa para obtener velocidades deseadas
    A = [cos(th) -L * sin(th);
         sin(th)  L * cos(th)];
    u = A \ u_bar;

    % Normalización y transformación a velocidades de rueda
    ux = u(1);
    uy = u(2);
    norm_val = max(sqrt(ux.^2 + uy.^2), 1e-6);
    ux_norm = no * ux / norm_val;
    uy_norm = no * uy / norm_val;
    u = [ux_norm; uy_norm];

    w_p = modelo_velocidad_ruedas(u);
    
    phi_izq = max(min(w_p(1), 5), -5);
    phi_der = max(min(w_p(2), 5), -5);

    % Enviar velocidades al ESP32
    data = sprintf("%.2f,%.2f\n", -phi_izq, phi_der);
    write(bt, data, "string");

    % Mostrar salida de velocidades en la consola
    fprintf("Velocidades enviadas:    %.2f     %.2f\n", phi_izq, phi_der);

    pause(0.1);
end

% Cerrar conexiones
clear cam;
clear bt;
disp('Conexión cerrada.');

% Función auxiliar para validar el tamaño y proporción del AprilTag
function valido = validar_tag(p1, p2, p3, p4)
    % Calcular longitudes de los lados
    l1 = norm(p1 - p2);
    l2 = norm(p2 - p3);
    l3 = norm(p3 - p4);
    l4 = norm(p4 - p1);

    % Calcular área usando la fórmula de un cuadrilátero
    d1 = norm(p1 - p3); % Diagonal 1
    d2 = norm(p2 - p4); % Diagonal 2
    area = 0.5 * d1 * d2;

    % Definir límites aceptables
    area_min = 500;  
    area_max = 50000;
    proporcion_min = 0.7;
    proporcion_max = 1.3;

    % Verificar área y proporción de lados
    valido = (area > area_min && area < area_max) && ...
             (proporcion_min < (l1 / l3) && (l1 / l3) < proporcion_max) && ...
             (proporcion_min < (l2 / l4) && (l2 / l4) < proporcion_max);
end

% Función auxiliar para transformar velocidades a ruedas
function w_p = modelo_velocidad_ruedas(u)
    L = 0.05535; % Distancia entre las ruedas
    r = 0.017;   % Radio de las ruedas
    T = [1/r  L/r;
         1/r -L/r];
    w_p = T * u;
end


% Función auxiliar para calcular la posición del punto alpha
% Esta función ajusta la ubicación del punto alpha con respecto 
% a la posición del centro del AprilTag y su orientación.
function alpha = variable_de_salida(x, y, th)
    L1 = 0.04; % Distancia entre el punto alpha y el eje de las ruedas (en metros)

    % Calcular la posición del punto alpha en coordenadas del sistema de referencia del robot
    p = x + L1 * cos(th); % Componente X del punto alpha
    q = y + L1 * sin(th); % Componente Y del punto alpha

    % Retornar las coordenadas del punto alpha en un vector columna
    alpha = [p; q];
end
