%% ============================================================
% PRECISION XY STAGE CONTROL
%
% PID + VELOCITY FEEDFORWARD
% DISTURBANCE REJECTION
% SENSOR NOISE
% ACTUATOR SATURATION
%
% MATLAB CONTROL ENGINEERING PROJECT
% ============================================================

clear;
clc;
close all;

fprintf('\n');
fprintf('============================================================\n');
fprintf('          PRECISION XY STAGE CONTROL PROJECT\n');
fprintf('============================================================\n');


%% ============================================================
% 1. CREATE RESULTS FOLDER
% ============================================================

projectFolder = pwd;

resultsFolder = fullfile(projectFolder,'results');

if ~exist(resultsFolder,'dir')
    mkdir(resultsFolder);
end

fprintf('\nProject folder:\n');
fprintf('%s\n',projectFolder);

fprintf('\nResults folder:\n');
fprintf('%s\n',resultsFolder);


%% ============================================================
% 2. SYSTEM PARAMETERS
% ============================================================

m = 1.0;

b = 2.0;

Kmotor = 10.0;


fprintf('\n');
fprintf('============================================================\n');
fprintf(' SYSTEM PARAMETERS\n');
fprintf('============================================================\n');

fprintf('Mass       = %.2f kg\n',m);
fprintf('Damping    = %.2f N.s/m\n',b);
fprintf('Motor Gain = %.2f\n',Kmotor);


%% ============================================================
% 3. PLANT MODEL
%
% m*x'' + b*x' = K*u
%
%             K
% G(s) = -------------
%        s*(m*s+b)
%
% ============================================================

s = tf('s');

G = Kmotor/(s*(m*s+b));


fprintf('\n');
fprintf('============================================================\n');
fprintf(' PLANT TRANSFER FUNCTION\n');
fprintf('============================================================\n');

disp(G);

fprintf('Equivalent mathematical model:\n');

fprintf('             %.2f\n',Kmotor);

fprintf('G(s) = -----------------------\n',1);

fprintf('       s*(%.2f*s + %.2f)\n',m,b);


%% ============================================================
% 4. OPEN LOOP RESPONSE
% ============================================================

fig1 = figure;

step(G,5);

grid on;

title('Open-Loop Precision Stage');

xlabel('Time (s)');

ylabel('Position');

drawnow;

print(fig1,...
    fullfile(resultsFolder,...
    '01_open_loop_response.png'),...
    '-dpng','-r150');


%% ============================================================
% 5. PID CONTROLLER
% ============================================================

Kp = 18;

Ki = 5;

Kd = 3;

C_PID = pid(Kp,Ki,Kd);


fprintf('\n');
fprintf('============================================================\n');
fprintf(' POSITION PID CONTROLLER\n');
fprintf('============================================================\n');

fprintf('Kp = %.4f\n',Kp);

fprintf('Ki = %.4f\n',Ki);

fprintf('Kd = %.4f\n',Kd);


%% ============================================================
% 6. CLOSED LOOP SYSTEM
% ============================================================

T_PID = feedback(C_PID*G,1);


%% ============================================================
% 7. PID STEP RESPONSE
% ============================================================

targetPosition = 0.05;


fig2 = figure;

step(targetPosition*T_PID,5);

grid on;

title('Closed-Loop PID Position Control');

xlabel('Time (s)');

ylabel('Position (m)');

drawnow;

print(fig2,...
    fullfile(resultsFolder,...
    '02_pid_position_response.png'),...
    '-dpng','-r150');


%% ============================================================
% 8. PID PERFORMANCE
% ============================================================

info = stepinfo(T_PID);


fprintf('\n');
fprintf('============================================================\n');
fprintf(' PID STEP RESPONSE PERFORMANCE\n');
fprintf('============================================================\n');

fprintf('Rise Time     = %.6f s\n',info.RiseTime);

fprintf('Settling Time = %.6f s\n',info.SettlingTime);

fprintf('Overshoot     = %.2f %%\n',info.Overshoot);


%% ============================================================
% 9. TIME VECTOR
% ============================================================

t = (0:0.005:10)';


%% ============================================================
% 10. POSITION TRAJECTORY
% ============================================================

reference = zeros(size(t));


reference(t >= 1 & t < 3) = 0.02;

reference(t >= 3 & t < 5) = 0.05;

reference(t >= 5 & t < 7) = 0.08;

reference(t >= 7) = 0.04;


%% ============================================================
% 11. TRAJECTORY SIMULATION
% ============================================================

[y_PID,~] = lsim(T_PID,reference,t);

y_PID = y_PID(:);

reference = reference(:);


%% ============================================================
% 12. TRAJECTORY TRACKING
% ============================================================

fig3 = figure;

plot(t,reference,'--','LineWidth',1.5);

hold on;

plot(t,y_PID,'LineWidth',1.5);

grid on;

title('Precision Stage Trajectory Tracking');

xlabel('Time (s)');

ylabel('Position (m)');

legend('Commanded Position',...
       'Actual Position');

drawnow;

print(fig3,...
    fullfile(resultsFolder,...
    '03_trajectory_tracking.png'),...
    '-dpng','-r150');


%% ============================================================
% 13. TRACKING ERROR
% ============================================================

trackingError = reference-y_PID;


maximumError = max(abs(trackingError));

RMSError = sqrt(mean(trackingError.^2));

finalError = abs(trackingError(end));


fig4 = figure;

plot(t,trackingError,'LineWidth',1.5);

grid on;

title('Position Tracking Error');

xlabel('Time (s)');

ylabel('Error (m)');

drawnow;

print(fig4,...
    fullfile(resultsFolder,...
    '04_tracking_error.png'),...
    '-dpng','-r150');


fprintf('\n');
fprintf('============================================================\n');
fprintf(' TRAJECTORY TRACKING PERFORMANCE\n');
fprintf('============================================================\n');

fprintf('Maximum Error = %.8f m\n',maximumError);

fprintf('RMS Error     = %.8f m\n',RMSError);

fprintf('Final Error   = %.8f m\n',finalError);


%% ============================================================
% 14. VELOCITY FEEDFORWARD
% ============================================================

dt = t(2)-t(1);


referenceVelocity = zeros(size(t));


referenceVelocity(2:end) = ...
    diff(reference)./dt;


Kff = b/Kmotor;


u_feedforward = Kff.*referenceVelocity;


G_control = feedback(G,C_PID);


[y_FF_effect,~] = ...
    lsim(G_control,u_feedforward,t);


y_FF_effect = y_FF_effect(:);


y_PID_FF = y_PID+y_FF_effect;


%% ============================================================
% 15. FEEDFORWARD COMPARISON
% ============================================================

fig5 = figure;

plot(t,reference,'--','LineWidth',1.5);

hold on;

plot(t,y_PID,'LineWidth',1.3);

plot(t,y_PID_FF,'LineWidth',1.3);

grid on;

title('PID vs PID + Velocity Feedforward');

xlabel('Time (s)');

ylabel('Position (m)');

legend('Reference',...
       'PID Only',...
       'PID + Feedforward');

drawnow;

print(fig5,...
    fullfile(resultsFolder,...
    '05_feedforward_comparison.png'),...
    '-dpng','-r150');


fprintf('\nVelocity feedforward analysis completed.\n');


%% ============================================================
% 16. DISTURBANCE REJECTION
% ============================================================

disturbance = zeros(size(t));


disturbance(t >= 5 & t <= 5.5) = -0.5;


G_disturbance = feedback(G,C_PID);


[y_disturbance,~] = ...
    lsim(G_disturbance,disturbance,t);


y_disturbance = y_disturbance(:);


y_with_disturbance = ...
    y_PID+y_disturbance;


%% ============================================================
% 17. DISTURBANCE REJECTION GRAPH
% ============================================================

fig6 = figure;

plot(t,reference,'--','LineWidth',1.5);

hold on;

plot(t,y_with_disturbance,'LineWidth',1.5);

grid on;

title('PID Disturbance Rejection');

xlabel('Time (s)');

ylabel('Position (m)');

legend('Reference',...
       'Position with Disturbance');

drawnow;

print(fig6,...
    fullfile(resultsFolder,...
    '06_disturbance_rejection.png'),...
    '-dpng','-r150');


%% ============================================================
% 18. SENSOR NOISE
% ============================================================

rng(10);


noiseAmplitude = 0.001;


sensorNoise = ...
    noiseAmplitude*randn(size(y_PID));


measuredPosition = ...
    y_PID+sensorNoise;


%% ============================================================
% 19. SENSOR NOISE GRAPH
% ============================================================

fig7 = figure;

plot(t,y_PID,'LineWidth',1.3);

hold on;

plot(t,measuredPosition,'LineWidth',0.8);

grid on;

title('Effect of Sensor Noise');

xlabel('Time (s)');

ylabel('Position (m)');

legend('True Position',...
       'Measured Position');

drawnow;

print(fig7,...
    fullfile(resultsFolder,...
    '07_sensor_noise.png'),...
    '-dpng','-r150');


%% ============================================================
% 20. ACTUATOR SATURATION
% ============================================================

integralError = ...
    cumtrapz(t,trackingError);


derivativeError = ...
    zeros(size(t));


derivativeError(2:end) = ...
    diff(trackingError)./dt;


u_PID = ...
    Kp.*trackingError ...
    + Ki.*integralError ...
    + Kd.*derivativeError;


actuatorLimit = 1.0;


u_saturated = ...
    min(max(u_PID,-actuatorLimit),...
    actuatorLimit);


%% ============================================================
% 21. ACTUATOR SATURATION GRAPH
% ============================================================

fig8 = figure;

plot(t,u_PID,'LineWidth',1.1);

hold on;

plot(t,u_saturated,'LineWidth',1.5);


% Horizontal upper limit
plot(t,...
     actuatorLimit*ones(size(t)),...
     '--','LineWidth',1.0);


% Horizontal lower limit
plot(t,...
     -actuatorLimit*ones(size(t)),...
     '--','LineWidth',1.0);


grid on;

title('Actuator Saturation');

xlabel('Time (s)');

ylabel('Control Command');

legend('Unconstrained Command',...
       'Saturated Command',...
       'Upper Limit',...
       'Lower Limit');

drawnow;

print(fig8,...
    fullfile(resultsFolder,...
    '08_actuator_saturation.png'),...
    '-dpng','-r150');


fprintf('\nActuator saturation analysis completed.\n');


%% ============================================================
% 22. AUTOMATIC PID TUNING
% ============================================================

autoTuningAvailable = true;


try

    PID_auto = pidtune(G,'PID');


    T_auto = ...
        feedback(PID_auto*G,1);


    fprintf('\n');
    fprintf('============================================================\n');
    fprintf(' AUTOMATIC PID TUNING\n');
    fprintf('============================================================\n');

    fprintf('Auto Kp = %.6f\n',PID_auto.Kp);

    fprintf('Auto Ki = %.6f\n',PID_auto.Ki);

    fprintf('Auto Kd = %.6f\n',PID_auto.Kd);


    fig9 = figure;

    step(targetPosition*T_auto,5);

    grid on;

    title('Automatically Tuned PID');

    xlabel('Time (s)');

    ylabel('Position (m)');

    drawnow;

    print(fig9,...
        fullfile(resultsFolder,...
        '09_automatic_pid.png'),...
        '-dpng','-r150');


catch

    autoTuningAvailable = false;

    fprintf('\nAutomatic PID tuning unavailable.\n');

end


%% ============================================================
% 23. FEEDFORWARD PERFORMANCE
% ============================================================

FF_error = ...
    reference-y_PID_FF;


FF_RMS_Error = ...
    sqrt(mean(FF_error.^2));


FF_Max_Error = ...
    max(abs(FF_error));


%% ============================================================
% 24. DISTURBANCE PERFORMANCE
% ============================================================

disturbanceError = ...
    reference-y_with_disturbance;


disturbanceMaxError = ...
    max(abs(disturbanceError));


%% ============================================================
% 25. SAVE NUMERICAL RESULTS
% ============================================================

resultsFile = ...
    fullfile(resultsFolder,...
    'performance_results.txt');


fid = fopen(resultsFile,'w');


fprintf(fid,...
    'PRECISION XY STAGE CONTROL\n');


fprintf(fid,...
    '===========================\n\n');


fprintf(fid,...
    'SYSTEM MODEL\n');


fprintf(fid,...
    'Mass = %.4f kg\n',m);


fprintf(fid,...
    'Damping = %.4f N.s/m\n',b);


fprintf(fid,...
    'Motor Gain = %.4f\n\n',Kmotor);


fprintf(fid,...
    'MANUAL PID CONTROLLER\n');


fprintf(fid,...
    'Kp = %.6f\n',Kp);


fprintf(fid,...
    'Ki = %.6f\n',Ki);


fprintf(fid,...
    'Kd = %.6f\n\n',Kd);


fprintf(fid,...
    'STEP RESPONSE PERFORMANCE\n');


fprintf(fid,...
    'Rise Time = %.6f s\n',info.RiseTime);


fprintf(fid,...
    'Settling Time = %.6f s\n',info.SettlingTime);


fprintf(fid,...
    'Overshoot = %.6f %%\n\n',info.Overshoot);


fprintf(fid,...
    'TRAJECTORY TRACKING\n');


fprintf(fid,...
    'Maximum Error = %.8f m\n',maximumError);


fprintf(fid,...
    'RMS Error = %.8f m\n',RMSError);


fprintf(fid,...
    'Final Error = %.8f m\n\n',finalError);


fprintf(fid,...
    'VELOCITY FEEDFORWARD\n');


fprintf(fid,...
    'Feedforward Gain = %.6f\n',Kff);


fprintf(fid,...
    'Maximum Feedforward Error = %.8f m\n',...
    FF_Max_Error);


fprintf(fid,...
    'RMS Feedforward Error = %.8f m\n\n',...
    FF_RMS_Error);


fprintf(fid,...
    'DISTURBANCE REJECTION\n');


fprintf(fid,...
    'Maximum Disturbance Error = %.8f m\n\n',...
    disturbanceMaxError);


fprintf(fid,...
    'ACTUATOR SATURATION\n');


fprintf(fid,...
    'Actuator Limit = %.4f\n\n',...
    actuatorLimit);


if autoTuningAvailable

    fprintf(fid,...
        'AUTOMATIC PID TUNING\n');

    fprintf(fid,...
        'Auto Kp = %.6f\n',...
        PID_auto.Kp);

    fprintf(fid,...
        'Auto Ki = %.6f\n',...
        PID_auto.Ki);

    fprintf(fid,...
        'Auto Kd = %.6f\n',...
        PID_auto.Kd);

end


fclose(fid);


%% ============================================================
% 26. VERIFY FILES
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('          PROJECT COMPLETED SUCCESSFULLY\n');
fprintf('============================================================\n');


expectedFiles = {

    '01_open_loop_response.png'

    '02_pid_position_response.png'

    '03_trajectory_tracking.png'

    '04_tracking_error.png'

    '05_feedforward_comparison.png'

    '06_disturbance_rejection.png'

    '07_sensor_noise.png'

    '08_actuator_saturation.png'

    'performance_results.txt'

    };


if autoTuningAvailable

    expectedFiles{end+1} = ...
        '09_automatic_pid.png';

end


fprintf('\nFiles saved in:\n');

fprintf('%s\n\n',resultsFolder);


for k = 1:length(expectedFiles)

    currentFile = ...
        fullfile(resultsFolder,...
        expectedFiles{k});


    if exist(currentFile,'file') == 2

        fileInfo = dir(currentFile);


        fprintf(...
            '[OK] %s   %.1f KB\n',...
            expectedFiles{k},...
            fileInfo.bytes/1024);

    else

        fprintf(...
            '[ERROR] %s\n',...
            expectedFiles{k});

    end

end


%% ============================================================
% 27. FINAL SUMMARY
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' CONTROL PROJECT SUMMARY\n');
fprintf('============================================================\n');

fprintf('PID Rise Time       : %.4f s\n',...
    info.RiseTime);

fprintf('PID Settling Time   : %.4f s\n',...
    info.SettlingTime);

fprintf('PID Overshoot       : %.2f %%\n',...
    info.Overshoot);

fprintf('Maximum Track Error : %.6f m\n',...
    maximumError);

fprintf('RMS Track Error     : %.6f m\n',...
    RMSError);

fprintf('Final Track Error   : %.6f m\n',...
    finalError);

fprintf('\n');

fprintf('Open the RESULTS folder to view the graphs.\n');

fprintf('============================================================\n');