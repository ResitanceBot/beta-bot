%% FICHERO DE TELEMETRIA EXPERIMENTOS BETA-BOT AMPLI ROBÓTICA
close all; 
clear all; clc;

nameExp = 'f1_static_';
formatFile = '.txt';
N_REPLIC = 3;

for exp = 1:N_REPLIC %Replicaciones
%     close all;

    str_exp = num2str(exp);
    FILE = strcat(nameExp,str_exp,formatFile);

    %% Ground Truth data
    fileTest=load(FILE);
    fileTest=fileTest(3:end,:);

    time = (fileTest(:,1)-fileTest(1,1)); %Time

    x_gt = fileTest(:,2); %Position X
    y_gt = fileTest(:,3); %Position Y
    z_gt = fileTest(:,4); %Position Z
    roll_gt = fileTest(:,5)*(180/pi); %Ang roll
    pitch_gt = fileTest(:,6)*(180/pi); %Ang pitch
    yaw_gt = fileTest(:,7)*(180/pi); %Ang yaw

    muestras = length(x_gt);

    %% Localization data

    x_loc = fileTest(:,8); %Position X
    y_loc = fileTest(:,9); %Position Y
    z_loc = fileTest(:,10); %Position Z
    roll_loc = fileTest(:,11)*(180/pi); %Ang roll
    pitch_loc = fileTest(:,12)*(180/pi); %Ang pitch
    yaw_loc = fileTest(:,13)*(180/pi); %Ang yaw

%     incertidumbre_loc = fileTest(:,14);

    %% REPRESENTACIONES

    %% TRAYECTORIA 3D
    figure()
    hold on
    plot3(x_gt,y_gt,z_gt,'b','LineWidth',2);
    plot3(x_gt(1),y_gt(1),z_gt(1),'xr','MarkerSize',20,'LineWidth',4);
    plot3(x_gt(end),y_gt(end),z_gt(end),'xg','MarkerSize',20,'LineWidth',4);
    plot3(x_loc,y_loc,z_loc,'k--','LineWidth',1);
    plot3(x_loc(1),y_loc(1),z_loc(1),'or','MarkerSize',15,'LineWidth',2);
    plot3(x_loc(end),y_loc(end),z_loc(end),'og','MarkerSize',15,'LineWidth',2);
    xlabel('X(m)');
    ylabel('Y(m)');
    zlabel('Z(m)');
    title('TRAYECTORIA ESPACIAL');
    legend('Trayectoria Real','Inicio Real','Final Real','Trayectoria Estimada','Inicio Estimado','Final Estimado')
    grid minor;
    grid on;
    hold off

    %% EVOLUCION TEMPORAL 
    figure()
    subplot(3,2,1)
    hold on
    plot(time,x_gt,'b','LineWidth',4);
    plot(time,x_loc,'--k','LineWidth',0.5);
    xlabel('Tiempo (s)');
    ylabel('X (m)');
    title('POSICIÓN X');
    legend('Real','Estimación')
    grid minor;
    grid on;
    hold off
    subplot(3,2,3)
    hold on
    plot(time,y_gt,'b','LineWidth',4);
    plot(time,y_loc,'--k','LineWidth',0.5);
    xlabel('Tiempo (s)');
    ylabel('Y (m)');
    title('POSICIÓN Y');
    legend('Real','Estimación')
    grid minor;
    grid on;
    hold off
    subplot(3,2,5)
    hold on
    plot(time,z_gt,'b','LineWidth',4);
    plot(time,z_loc,'--k','LineWidth',0.5);
    xlabel('Tiempo (s)');
    ylabel('Z (m)');
    title('POSICIÓN Z');
    legend('Real','Estimación')
    grid minor;
    grid on;
    hold off
    subplot(3,2,2)
    hold on
    plot(time,roll_gt,'b','LineWidth',4);
    plot(time,roll_loc,'--k','LineWidth',0.5);
    xlabel('Tiempo (s)');
    ylabel('ROLL (º)');
    title('ORIENTACIÓN ROLL');
    legend('Real','Estimación')
    grid minor;
    grid on;
    hold off
    subplot(3,2,4)
    hold on
    plot(time,pitch_gt,'b','LineWidth',4);
    plot(time,pitch_loc,'--k','LineWidth',0.5);
    xlabel('Tiempo (s)');
    ylabel('PITCH (º)');
    title('ORIENTACIÓN PITCH');
    legend('Real','Estimación')
    grid minor;
    grid on;
    hold off
    subplot(3,2,6)
    hold on
    plot(time,yaw_gt,'b','LineWidth',4);
    plot(time,yaw_loc,'--k','LineWidth',0.5);
    xlabel('Tiempo (s)');
    ylabel('YAW (º)');
    title('ORIENTACIÓN YAW');
    legend('Real','Estimación')
    grid minor;
    grid on;
    hold off

    %% EVOLUCIÓN ERROR POSICIÓN y DISTANCIA EUCLÍDEA
    errorX = abs(x_gt-x_loc);
    errorY = abs(y_gt-y_loc);
    errorZ = abs(z_gt-z_loc);
    errorDist = sqrt((x_gt-x_loc).^2+(y_gt-y_loc).^2+(z_gt-z_loc).^2);
    figure()
    subplot(4,1,1)
    hold on
    plot(time,errorX,'b','LineWidth',1);
    xlabel('Tiempo (s)');
    ylabel('Error X (m)');
    title('Error X');
    grid minor;
    grid on;
    subplot(4,1,2)
    hold on
    plot(time,errorY,'b','LineWidth',1);
    xlabel('Tiempo (s)');
    ylabel('Error Y (m)');
    title('Error Y');
    grid minor;
    grid on;
    subplot(4,1,3)
    hold on
    plot(time,errorZ,'b','LineWidth',1);
    xlabel('Tiempo (s)');
    ylabel('Error Z (m)');
    title('Error Z');
    grid minor;
    grid on;
    hold off
    subplot(4,1,4)
    hold on
    plot(time,errorDist,'r','LineWidth',1);
    xlabel('Tiempo (s)');
    ylabel('ErrorDist (m)');
    title('Error Distancia');
    grid minor;
    grid on;
    hold off

    %% ERROR EN ORIENTACIÓN Y DISTANCIA ANGULAR

    errorRoll = abs(angulo_minimo(roll_gt,roll_loc));
    errorPitch = abs(angulo_minimo(pitch_gt,pitch_loc));
    errorYaw = abs(angulo_minimo(yaw_gt,yaw_loc));
    errorAng = [];

    % Distancia angular
    for i=1:muestras
        % Matrices Rot GT
        Rx_gt(:,:,i) = [1 0 0; 0 cosd(roll_gt(i)) -sind(roll_gt(i)); 0 sind(roll_gt(i)) cosd(roll_gt(i))];
        Ry_gt(:,:,i) = [cosd(pitch_gt(i)) 0 sind(pitch_gt(i)); 0 1 0; -sind(pitch_gt(i)) 0 cosd(pitch_gt(i))];
        Rz_gt(:,:,i) = [cosd(yaw_gt(i)) -sind(yaw_gt(i)) 0; sind(yaw_gt(i)) cosd(yaw_gt(i)) 0; 0 0 1];
        R_gt(:,:,i) = Rx_gt(:,:,i)*Ry_gt(:,:,i)*Rz_gt(:,:,i);
        % Matrices Rot Loc
        Rx_loc(:,:,i) = [1 0 0; 0 cosd(roll_loc(i)) -sind(roll_loc(i)); 0 sind(roll_loc(i)) cosd(roll_loc(i))];
        Ry_loc(:,:,i) = [cosd(pitch_loc(i)) 0 sind(pitch_loc(i)); 0 1 0; -sind(pitch_loc(i)) 0 cosd(pitch_loc(i))];
        Rz_loc(:,:,i) = [cosd(yaw_loc(i)) -sind(yaw_loc(i)) 0; sind(yaw_loc(i)) cosd(yaw_loc(i)) 0; 0 0 1];
        R_loc(:,:,i) = Rx_loc(:,:,i)*Ry_loc(:,:,i)*Rz_loc(:,:,i);
        % Distancia angular
        errorAng(i) = acosd((trace(R_gt(:,:,i)*R_loc(:,:,i)')-1)/2);
    end

    figure()
    subplot(4,1,1)
    hold on
    plot(time,errorRoll,'b','LineWidth',1);
    xlabel('Tiempo (s)');
    ylabel('Error Roll (º)');
    title('Error Roll');
    grid minor;
    grid on;
    subplot(4,1,2)
    hold on
    plot(time,errorPitch,'b','LineWidth',1);
    xlabel('Tiempo (s)');
    ylabel('Error Pitch (º)');
    title('Error Pitch');
    grid minor;
    grid on;
    subplot(4,1,3)
    hold on
    plot(time,errorYaw,'b','LineWidth',1);
    xlabel('Tiempo (s)');
    ylabel('Error Yaw (º)');
    title('Error Yaw');
    grid minor;
    grid on;
    hold off
    subplot(4,1,4)
    hold on
    plot(time,errorAng,'r','LineWidth',1);
    xlabel('Tiempo (s)');
    ylabel('ErrorAng (m)');
    title('Error Ángulo');
    grid minor;
    grid on;
    hold off

    %% EVOLUCION INCERTIDUMBRE (DETERMINANTE DE SIGMA)
%     figure()
%     hold on
%     plot(time,incertidumbre_loc,'b','LineWidth',4);
%     xlabel('Tiempo (s)');
%     ylabel('Incertidumbre');
%     title('INCERTIDUMBRE (DETERMINANTE DE SIGMA)');
%     grid minor;
%     grid on;
%     hold off

    %% ESTADÍSTICAS DE ERRORES REPLICACIONES

    disp('-------- RESULTADOS ESTADÍSTICAS REPLICACIÓN--------')
    % POSICIÓN
    errorMedioX = mean(errorX);
    errorMaxX = max(errorX);
    desvTipX = std(errorX);

    errorMedioY = mean(errorY);
    errorMaxY = max(errorY);
    desvTipY = std(errorY);

    errorMedioZ = mean(errorZ);
    errorMaxZ = max(errorZ);
    desvTipZ = std(errorZ);

    errorMedioDist = mean(errorDist);
    errorMaxDist = max(errorDist);
    desvTipDist = std(errorDist);

    % ORIENTACIÓN
    errorMedioRoll = mean(errorRoll);
    errorMaxRoll = max(errorRoll);
    desvTipRoll = std(errorRoll);

    errorMedioPitch = mean(errorPitch);
    errorMaxPitch = max(errorPitch);
    desvTipPitch = std(errorPitch);

    errorMedioYaw = mean(errorYaw);
    errorMaxYaw = max(errorYaw);
    desvTipYaw = std(errorYaw);

    errorMedioAng = mean(errorAng);
    errorMaxAng = max(errorAng);
    desvTipAng = std(errorAng);

    % Tabla Experimento Actual
    errorMedio = round([errorMedioX,errorMedioY,errorMedioZ,errorMedioDist,errorMedioRoll,errorMedioPitch,errorMedioYaw,errorMedioAng],2);
    errorMax = round([errorMaxX,errorMaxY,errorMaxZ,errorMaxDist,errorMaxRoll,errorMaxPitch,errorMaxYaw,errorMaxAng],2);
    desvTip = round([desvTipX,desvTipY,desvTipZ,desvTipDist,desvTipRoll,desvTipPitch,desvTipYaw,desvTipAng],2);

    Errores = table(errorMedio', errorMax', desvTip', 'VariableNames', {'Error Medio', 'Error Max', 'Desv.Típ. Error'}, 'RowNames', {'X [m]', 'Y [m]', 'Z [m]','Dist [m]','Roll [º]','Pitch [º]','Yaw [º]','Ang [º]'})
    % writetable(Errores, 'Tabla.txt', 'Delimiter', '\t', 'WriteRowNames', true);

    % Almacenar resultados para Total replicaciones
    errorMedioExp(exp,:) = [errorMedioX,errorMedioY,errorMedioZ,errorMedioDist,errorMedioRoll,errorMedioPitch,errorMedioYaw,errorMedioAng];
    errorMaxExp(exp,:) = [errorMaxX,errorMaxY,errorMaxZ,errorMaxDist,errorMaxRoll,errorMaxPitch,errorMaxYaw,errorMaxAng];

    autoArrangeFigures();

end

%% ESTADÍSTICAS TOTALES DE TODAS LAS REPLICACIONES
mediaErrorMedioExp = round(mean(errorMedioExp),2); % Media de los errores medios obtenidos en todos los experimentos
devtipErrorMedioExp = round(std(errorMedioExp),2); % Desv.Tip de los errores medios obtenidos en todos los experimentos
mediaErrorMaxExp = round(mean(errorMaxExp),2); % Media de los errores maximos obtenidos en todos los experimentos
devtipErrorMaxExp = round(std(errorMaxExp),2); % Desv.Tip de los errores maximos obtenidos en todos los experimentos

ResultExp = table(mediaErrorMedioExp', devtipErrorMedioExp', mediaErrorMaxExp',devtipErrorMaxExp', 'VariableNames', {'Media de Errores Medios', 'Desv.Típ de Errores Medios', 'Media de Errores Máximos', 'Desv.Típ de Errores Máximos'}, 'RowNames', {'X [m]', 'Y [m]', 'Z [m]','Dist [m]','Roll [º]','Pitch [º]','Yaw [º]','Ang [º]'})
writetable(ResultExp, strcat(nameExp,'ResTabla',formatFile), 'Delimiter', '\t', 'WriteRowNames', true);


%% FUNCIONES AUXILIARES


function ang_diff = angulo_minimo(ang1, ang2)

ang_diff = mod(ang2 - ang1 + 180, 360) - 180; % Calcula la diferencia y la ajusta a -180 a 180

end