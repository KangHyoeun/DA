%% System identification code 2023.04.11 시작 
clear; clc; close all;

%% 기본값
DTR = pi/180; RTD = 180/pi; knots = 0.5144; 
dt = 0.1;

%% Load bag data
test_date   = 240530;
test_num    = 2;
bag_data    = rosbag('C:\bag data\Result_data_'+string(test_num)+'.bag');
msg_data    = readMessages(select(bag_data,'Topic','/pknu_tuning_info'),'DataFormat','struct');
num_msg     = size(msg_data,1); % 데이터 사이즈


% 빈 데이터 확인 및 제거
non_empty_indices = cellfun(@(x) ~isempty(x.Data), msg_data);
cleaned_msg_data  = msg_data(non_empty_indices);
msg_data          = cleaned_msg_data;
num_cleaned_msg   = size(cleaned_msg_data, 1);
num_msg           = num_cleaned_msg;
  
st_index  = 5000; % 시작인덱스
end_index = 13800; % 종료인덱스
num_ship  = 2; % 데이터를 불러올 ship 개수
shipID    = 1;

%% plot 변수
PLOT      = "ON"; % 그림 ON / OFF
ANIMATION = "OFF"; % 애니메이션 ON / OFF 
p_rate    = 10;

%% bag 데이터 불러오기 ******************msg 형태에 따라 수정해야함
for i = 1:num_ship
    for j = 1:num_msg
        Raw_data{i}(j,1)  = double(msg_data{j}.Header.Stamp.Sec) + double(msg_data{j}.Header.Stamp.Nsec)*10^-9 ;
        Raw_data{i}(j,2)  = msg_data{j}.Data(i).X        ;
        Raw_data{i}(j,3)  = msg_data{j}.Data(i).Y        ;
        Raw_data{i}(j,4)  = msg_data{j}.Data(i).Psi      ;
        if num_ship == 1 
            Raw_data{i}(j,5)  = msg_data{j}.Data(i).Speed * cos(msg_data{j}.Data(i).Beta) ;
            Raw_data{i}(j,6)  = msg_data{j}.Data(i).Speed * sin(msg_data{j}.Data(i).Beta) ;
        else %% 가상선 편류각 각도 단위 : deg
            Raw_data{i}(j,5)  = msg_data{j}.Data(i).Speed * cosd(msg_data{j}.Data(i).Beta) ;
            Raw_data{i}(j,6)  = msg_data{j}.Data(i).Speed * sind(msg_data{j}.Data(i).Beta) ;
        end
        Raw_data{i}(j,7)  = msg_data{j}.Data(i).R        ;
        Raw_data{i}(j,8)  = msg_data{j}.Data(i).Delta    ;
        Raw_data{i}(j,9)  = msg_data{j}.Data(i).DeltaC   ;
        Raw_data{i}(j,10) = msg_data{j}.Data(i).Rps      ;
        Raw_data{i}(j,11) = msg_data{j}.Data(i).RpsC     ;
        Raw_data{i}(j,12) = msg_data{j}.Data(i).PsiD     ;
        Raw_data{i}(j,13) = msg_data{j}.Data(i).Beta     ;
        Raw_data{i}(j,14) = msg_data{j}.Data(i).Speed    ;
        Raw_data{i}(j,15) = msg_data{j}.Data(i).SpeedD   ;
        Raw_data{i}(j,16) = msg_data{j}.Data(i).KpY      ;
        Raw_data{i}(j,17) = msg_data{j}.Data(i).KdY      ;
        Raw_data{i}(j,18) = msg_data{j}.Data(i).KiY      ; 
        Raw_data{i}(j,19) = msg_data{j}.Data(i).KpS      ;
        Raw_data{i}(j,20) = msg_data{j}.Data(i).KdS      ;
        Raw_data{i}(j,21) = msg_data{j}.Data(i).KiS      ;
        Raw_data{i}(j,22) = msg_data{j}.Data(i).WpX      ;
        Raw_data{i}(j,23) = msg_data{j}.Data(i).WpY      ;
        Raw_data{i}(j,24) = msg_data{j}.Data(i).WpXKaist ;
        Raw_data{i}(j,25) = msg_data{j}.Data(i).WpYKaist ;
        Raw_data{i}(j,26) = msg_data{j}.Data(i).WpXInha  ;
        Raw_data{i}(j,27) = msg_data{j}.Data(i).WpYInha  ;
        Raw_data{i}(j,28) = msg_data{j}.Data(i).ShipID   ;
    end
end

%% LPF
alpha     = 0.7  ; % LPF 파라미터
for i = 1:num_ship
    Data{i}(:,1)    = Raw_data{i}(st_index:end_index,1) - Raw_data{i}(st_index,1) ;
    Data{i}(:,2)    = Fn_LPF(Raw_data{i}(st_index:end_index,2), alpha)'           ;
    Data{i}(:,3)    = Fn_LPF(Raw_data{i}(st_index:end_index,3), alpha)'           ;
    Data{i}(:,4)    = Fn_LPF(Raw_data{i}(st_index:end_index,4), alpha)'           ;
    Data{i}(:,5)    = Fn_LPF(Raw_data{i}(st_index:end_index,5), alpha)'           ;
    Data{i}(:,6)    = Fn_LPF(Raw_data{i}(st_index:end_index,6), alpha)'           ;
    Data{i}(:,7)    = Fn_LPF(Raw_data{i}(st_index:end_index,7), alpha)'           ;      
    Data{i}(:,8:28) = Raw_data{i}(st_index:end_index,8:28)                        ;
end

num_msg = size(Data{shipID},1); % 데이터 사이즈

%% 변수 할당
u     = Data{shipID}(:,5);
v     = Data{shipID}(:,6);
r     = Data{shipID}(:,7);
delta = Data{shipID}(:,8);

%% u', v'
for j = 2:num_msg-1
    if j == 2 || j == num_msg-1
        u_dot(j,1) = (u(j+1) - u(j-1,1))/(dt*2) ; % u'
        v_dot(j,1) = (v(j+1) - v(j-1,1))/(dt*2) ; % v'
        r_dot(j,1) = (r(j+1) - r(j-1,1))/(dt*2) ; % r'
    else
        u_dot(j,1) = (u(j+2) - u(j-2,1))/(dt*4) ; % u'
        v_dot(j,1) = (v(j+2) - v(j-2,1))/(dt*4) ; % v'
        r_dot(j,1) = (r(j+2) - r(j-2,1))/(dt*4) ; % r'
    end
    
    if j == num_msg-1
        u_dot(j+1) = u_dot(j,1)                 ; % u'
        v_dot(j+1) = v_dot(j,1)                 ; % v'
        r_dot(j+1) = r_dot(j,1)                 ; % r'
    end
end

%% Speed model
U0     = 0.9                            ; % ********************수정해야함
U_mat  = u_dot' * pinv([u-U0 delta.^2])' ; % u' = U_mat * [u-U0 δ^2]
Tu     = -1/U_mat(1)                     ; %
Ku     = U_mat(2)*Tu                     ; %

%% Nomoto 1st model parameter
KT_mat = r_dot' * pinv([r delta])' ; % r' = KT_mat * [r δ]
T      = -1/KT_mat(1)              ; %
K      = KT_mat(2)*T               ; %

%% Gain Tuning
ST   = 4                         ; % 구간 감소 => gain 값 증가, Saturation 발생 
zeta = 1                          ;
wn   = 4 /(zeta*ST)               ;

Kp   = round(wn^2*T/K,1)          ;  
Kd   = round((2*T*zeta*wn-1)/K,1) ; 

Parameter = [round(K,4) round(T,4) round(Ku,4) round(Tu,4) Kp Kd]     

%% Cross track error : desired track 에서 벗어난 수직이탈거리, RMSE 계산





%% PLOT
if PLOT == "ON"
    hFig = figure(1);
    set(hFig, 'position', [00 00 1600 900],'Color','white');
    % set(gca, 'color', 'none')

    % 색상 목록 설정
    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']; % 필요한 만큼 색상을 추가
    colorIndex = 1; % 초기 색상 인덱스

    % 경유점과 반경 설정
    waypoint    = [176.636, -8.0786];
    radius      = 5; % 반경 (근방의 범위 설정)

    % X, Y 경로 플롯
    subplot(5,3,[1,2,4,5,7,8,10,11,13,14]);
    hold on;
    grid on;
    
    % 초기 위치 설정
    in_circle = false;

    % 현재 선분을 저장할 리스트 초기화
    x_segment = [];
    y_segment = [];
    lap_count = 0; % 바퀴 수를 세기 위한 변수 초기화
    lap_data = {}; % 바퀴별 데이터를 저장할 셀 배열
    lap_times = {}; % 바퀴별 시간을 저장할 셀 배열

    for i = 1:length(Data{shipID}(:,3))
        % 현재 점이 경유점 근방에 있는지 확인
        distance = sqrt((Data{shipID}(i,3) - waypoint(1))^2 + (Data{shipID}(i,2) - waypoint(2))^2);
        if distance < radius
            if ~in_circle
                % 경유점 근방에 처음 진입한 경우 이전 선분을 그립니다.
                if ~isempty(x_segment)
                    plot(x_segment, y_segment, 'Color', colors(colorIndex), 'LineStyle', '--', 'LineWidth', 1.5);
                    lap_count = lap_count + 1;
                    lap_data{lap_count} = [x_segment', y_segment'];
                    lap_times{lap_count} = Data{shipID}(1:length(x_segment), 1); % 시간 저장
                    x_segment = [];
                    y_segment = [];
                end
                % 색상 변경
                colorIndex = mod(colorIndex, length(colors)) + 1;
                in_circle = true;
            end
        else
            in_circle = false;
        end

        % 현재 점을 선분 리스트에 추가
        x_segment = [x_segment, Data{shipID}(i,3)];
        y_segment = [y_segment, Data{shipID}(i,2)];
    end

    % 마지막 선분을 그립니다.
    if ~isempty(x_segment)
        plot(x_segment, y_segment, 'Color', colors(colorIndex), 'LineStyle', '--', 'LineWidth', 1.5);
        lap_count = lap_count + 1;
        lap_data{lap_count} = [x_segment', y_segment'];
        lap_times{lap_count} = Data{shipID}(1:length(x_segment), 1); % 시간 저장
    end

    xlabel('Y [m]');
    ylabel('X [m]');
    title('Path with Different Colors for Each Lap');

    % 경유점 표시 및 선으로 연결
    waypointX = [20, 10, 50, -20, 20];
    waypointY = [140, 200, 160, 180, 140];
    
    plot(waypointY, waypointX, 'ro', 'MarkerSize', 8, 'LineWidth', 2); % 경유점을 빨간 원으로 표시
    plot(waypointY, waypointX, 'r-', 'LineWidth', 1.5); % 경유점을 선으로 연결

    legend(arrayfun(@(x) ['Lap ' num2str(x)], 1:lap_count, 'UniformOutput', false));
    hold off;

    waypointY = waypointY + (1:length(waypointY))*1e-5; % 고유하게 만들기 위해 약간의 변화 추가


    % 지표 계산
    for lap = 1:lap_count

        lap_x = lap_data{lap}(:, 1);
        lap_y = lap_data{lap}(:, 2);
        lap_time = lap_times{lap};

        % RMSE와 최대 오차 계산
        rmse = calculateRMSE(lap_x, lap_y, waypointX, waypointY);
        maxError = calculateMaxError(lap_x, lap_y, waypointX, waypointY);
        fprintf('Lap %d - RMSE: %.2f, Max Error: %.2f\n', lap, rmse, maxError);

        % 오버슈트와 정착 시간 계산 (y 좌표 기준)
        overshoot = calculateOvershoot(lap_y, waypointX);
        settlingTime = calculateSettlingTime(lap_time, lap_y, waypointX, 0.05);
        fprintf('Lap %d - Overshoot: %.2f%%, Settling Time: %.2f sec\n', lap, overshoot, settlingTime);

        % 제어 입력 RMS 값 계산 (u 속도 기준)
        controlRMS = calculateControlRMS(Data{shipID}(1:length(lap_x), 5));
        fprintf('Lap %d - Control RMS: %.2f\n', lap, controlRMS);

        % 외란에 의한 오차 변화 (경유점을 지나는 두 시점 간)
        disturbanceErrorChange = calculateDisturbanceErrorChange(lap_y, interp1(waypointY, waypointX, lap_y), [1, length(lap_y)]);
        fprintf('Lap %d - Disturbance Error Change: %.2f\n', lap, disturbanceErrorChange);
    end

       
    subplot(5,3,3)
    plot(Data{shipID}(:,1),sqrt(Data{shipID}(:,14)),'linewidt',2) ; hold on ; grid on ; 
    xlabel('Time [sec]') ; ylabel('Velocity [m/s]') ;
%     set(gca, 'color', 'none')

    
    subplot(5,3,6)
    plot(Data{shipID}(:,1),Data{shipID}(:,5),'linewidt',2) ; hold on ; grid on ; 
%         plot(Data{shipID}(:,1),Data{shipID}(:,6),'--','linewidt',2) ;
    xlabel('Time [sec]') ; ylabel('u [m/s]') ;
%     legend("u", "v")
%     set(gca, 'color', 'none')

    
    subplot(5,3,9)
    plot(Data{shipID}(:,1),Data{shipID}(:,4),'linewidt',2) ; hold on ; grid on ; 
    plot(Data{shipID}(:,1),Data{shipID}(:,12),'--','linewidt',2) ;
    xlabel('Time [sec]') ; ylabel('\psi [°]') ;
%     set(gca, 'color', 'none')

    subplot(5,3,12)
    plot(Data{shipID}(:,1),Data{shipID}(:,13),'linewidt',2) ; hold on ; grid on ; 
    xlabel('Time [sec]') ; ylabel('Rate of turn [°/s]') ;
%     set(gca, 'color', 'none')

    subplot(5,3,15)
    plot(Data{shipID}(:,1),Data{shipID}(:,8),'linewidt',2) ; hold on ; grid on ;
    plot(Data{shipID}(:,1),Data{shipID}(:,9),'--','linewidt',2) ;
    xlabel('Time [sec]') ; ylabel('Delta [°]') ;
    legend("\delta", "\delta_d")
%     set(gcf, 'color', 'none')
%     set(gca, 'color', 'none')
end

%% Animation
if ANIMATION == "ON"
    hFig = figure(2);
    set(hFig, 'position', [0 0 2560 1330],'Color','white')
    
    writerObj = VideoWriter('Simulation.avi'); 
    writerObj.FrameRate = 20 ;
    writerObj.Quality = 100 ;
    open(writerObj); 
    
    for k = 1:floor((Time_f/Time_s))/p_rate
        if k == 1
            subplot(5,3,[1,2,4,5,7,8,10,11,13,14])
            PATH{shipID} = plot(Data{shipID}(1:k*p_rate,3),Data{shipID}(1:k*p_rate,2),'Color', Line_color(i,1:3), 'LineStyle', Line_style(i),'LineWidth',2); hold on; grid on;
            axis([-1 16 -2 16])  % 창원대학교
            xlabel("Y [m]"); ylabel("X [m]")

            subplot(5,3,3)
            SPEED{shipID} = plot(Data{shipID}(1:k*p_rate,1),sqrt(Data{shipID}(1:k*p_rate,14)),'Color', Line_color(i,1:3), 'LineStyle', '-','LineWidth',2); hold on; grid on;
            xlabel('Time [s]'); ylabel('Velocity [m/s]');
%                 legend('u','v',"Location", "southeast")

            subplot(5,3,6)
            U{shipID} = plot(Data{shipID}(1:k*p_rate,1),Data{shipID}(1:k*p_rate,5),'Color', Line_color(i,1:3), 'LineStyle', '-','LineWidth',2); hold on; grid on;
            UD{shipID} = plot(Data{shipID}(1:k*p_rate,1),Data{shipID}(1:k*p_rate,16),'Color', Line_color(i+3,1:3), 'LineStyle', '--','LineWidth',2);
            xlabel('Time [s]'); ylabel('Serge velocity [m/s]');
            
            subplot(5,5,9)
            PSI{shipID} = plot(Data{shipID}(1:k*p_rate,1),Data{shipID}(1:k*p_rate,4), 'Color', Line_color(i,1:3), 'LineStyle','-', 'LineWidth',2); hold on; grid on;
            PSID{shipID} = plot(Data{shipID}(1:k*p_rate,1),Data{shipID}(1:k*p_rate,12), 'Color', Line_color(i+3,1:3), 'LineStyle','--', 'LineWidth',2); 
            xlabel('Time [s]'); ylabel('Yaw angle [°]')
            
            subplot(5,5,12)
            YAWRATE{shipID} = plot(Data{shipID}(1:k*p_rate,1),Data{shipID}(1:k*p_rate,7), 'Color', Line_color(i,1:3),'LineStyle', '-', 'LineWidth',2); hold on; grid on;
            xlabel('Time [s]'); ylabel('Yaw rate [°/s]')
            
            subplot(5,5,15)
            Delta{shipID} = plot(Data{shipID}(1:k*p_rate,1),Data{shipID}(1:k*p_rate,8), 'Color', Line_color(i,1:3), 'LineStyle','-', 'LineWidth',2); hold on; grid on;
            Delta_d{shipID} = plot(Data{shipID}(1:k*p_rate,1),Data{shipID}(1:k*p_rate,9),'Color', Line_color(i+3,1:3), 'LineStyle','--', 'LineWidth', 2);
            xlabel('Time [s]'); ylabel('\delta [°]');
%                 legend('Left','Right',"Location", "southeast")
        else 
            %% 데이터 set
            set(PATH{shipID}, 'XData', Data{shipID}(1:k*p_rate,3), 'YData', Data{shipID}(1:k*p_rate,2)) 
            set(SPEED{shipID},'XData', Data{shipID}(1:k*p_rate,1), 'YData', sqrt(Data{shipID}(1:k*p_rate,14)))
            set(U{shipID},'XData', Data{shipID}(1:k*p_rate,1), 'YData', Data{shipID}(1:k*p_rate,5))
            set(UD{shipID},'XData', Data{shipID}(1:k*p_rate,1), 'YData', Data{shipID}(1:k*p_rate,15))
            set(PSI{shipID},'XData', Data{shipID}(1:k*p_rate,1), 'YData', Data{shipID}(1:k*p_rate,4))
            set(PSID{shipID},'XData', Data{shipID}(1:k*p_rate,1), 'YData', Data{shipID}(1:k*p_rate,12))
            set(YAWRATE{shipID},'XData', Data{shipID}(1:k*p_rate,1), 'YData', Data{shipID}(1:k*p_rate,7))
            set(Delta{shipID},'XData', Data{shipID}(1:k*p_rate,1), 'YData', Data{shipID}(1:k*p_rate,8))
            set(Delta_d{shipID},'XData', Data{shipID}(1:k*p_rate,1), 'YData', Data{shipID}(1:k*p_rate,9))
            drawnow;
            frame = getframe(gcf); 
            writeVideo(writerObj, frame);
        end
    end
    
    hold off
    close(writerObj) ;
end


% 유클리드 거리 기반 RMSE 계산 함수
function rmse = calculateRMSE(actual_x, actual_y, waypoint_x, waypoint_y)
    distances = calculateDistances(actual_x, actual_y, waypoint_x, waypoint_y);
    rmse = sqrt(mean(distances .^ 2));
end

% 최대 오차 계산 함수
function maxError = calculateMaxError(actual_x, actual_y, waypoint_x, waypoint_y)
    distances = calculateDistances(actual_x, actual_y, waypoint_x, waypoint_y);
    maxError = max(distances);
end

% 각 점에서 가장 가까운 경유점 선분과의 수직 거리 계산 함수
function distances = calculateDistances(actual_x, actual_y, waypoint_x, waypoint_y)
    num_points = length(actual_x);
    distances = zeros(num_points, 1);
    
    for i = 1:num_points
        % 현재 점
        x0 = actual_x(i);
        y0 = actual_y(i);
        
        % 각 선분에 대한 수직 거리 계산
        min_distance = inf;
        for j = 1:(length(waypoint_x) - 1)
            x1 = waypoint_x(j);
            y1 = waypoint_y(j);
            x2 = waypoint_x(j + 1);
            y2 = waypoint_y(j + 1);
            
            % 선분의 두 점 사이의 수직 거리 계산
            distance = point_to_line_distance(x0, y0, x1, y1, x2, y2);
            if distance < min_distance
                min_distance = distance;
            end
        end
        
        distances(i) = min_distance;
    end
end

% 점에서 선분으로의 수직 거리 계산 함수
function distance = point_to_line_distance(x0, y0, x1, y1, x2, y2)
    % 선분의 길이를 계산
    A = x0 - x1;
    B = y0 - y1;
    C = x2 - x1;
    D = y2 - y1;

    dot = A * C + B * D;
    len_sq = C * C + D * D;
    param = -1;
    if len_sq ~= 0
        param = dot / len_sq;
    end

    if param < 0
        xx = x1;
        yy = y1;
    elseif param > 1
        xx = x2;
        yy = y2;
    else
        xx = x1 + param * C;
        yy = y1 + param * D;
    end

    dx = x0 - xx;
    dy = y0 - yy;
    distance = sqrt(dx * dx + dy * dy);
end


function overshoot = calculateOvershoot(actual, reference)
% 오버슈트 계산 함수
    peakValue = max(actual);
    targetValue = reference(end); % Assume the last value as the target
    overshoot = (peakValue - targetValue) / targetValue * 100;
end

function settlingTime = calculateSettlingTime(time, actual, reference, tolerance)
% 정착 시간 계산 함수
    targetValue = reference(end); % Assume the last value as the target
    upperBound = targetValue * (1 + tolerance);
    lowerBound = targetValue * (1 - tolerance);
    settlingIndex = find(actual > upperBound | actual < lowerBound, 1, 'last');
    if isempty(settlingIndex)
        settlingTime = 0; % If it never goes outside the bounds, settling time is zero
    else
        settlingTime = time(settlingIndex);
    end
end

function controlRMS = calculateControlRMS(controlInput)
% 제어 입력의 RMS 값 계산 함수
    controlRMS = sqrt(mean(controlInput .^ 2));
end

function disturbanceErrorChange = calculateDisturbanceErrorChange(actual, reference, disturbanceIndices)
% 외란에 의한 오차 변화 계산 함수
    preDisturbanceErrors = abs(actual(disturbanceIndices(1)) - reference(disturbanceIndices(1)));
    postDisturbanceErrors = abs(actual(disturbanceIndices(2)) - reference(disturbanceIndices(2)));
    disturbanceErrorChange = postDisturbanceErrors - preDisturbanceErrors;
end


function lpf_data = Fn_LPF(x, alpha)

firstrun = [] ;
num_msg  = size(x,1) ;

if isempty(firstrun)
    lpf_data(1) = x(1) ;
    firstrun = 1;
end

for i = 2:num_msg
    lpf_data(i) = alpha*lpf_data(i-1) + (1-alpha)*x(i) ;
end
end