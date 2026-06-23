%% 1. 图形与动态参数初始化
clear; clc; close all;

% 强效清理遗留端口与图窗
try pnet('closeall'); catch; end  
close all force;                  

% 基础采样率（Plane SITL 通常为 50Hz）
base_rate = 50;
dt = 1/base_rate;
window = 1000; % 缓存滑动窗口大小

time = -(window - 1) * dt : dt : 0;

% 初始化基础数据缓存
target = nan(1, window);
actual = nan(1, window);
error = nan(1, window);
P_term = nan(1, window);
I_term = nan(1, window);
D_term = nan(1, window);
ff_term = nan(1, window);

%% 2. 构建多图窗遥测界面
% --- Figure 1: 原始 PID 响应曲线 ---
fig1 = figure('Name', '1. Raw PID Response', 'Position', [50, 100, 600, 700]);
ax1_1 = subplot(3, 1, 1); hold on; grid on;
target_plot = plot(ax1_1, time, target, 'DisplayName', 'target');
actual_plot = plot(ax1_1, time, actual, 'DisplayName', 'actual');
error_plot  = plot(ax1_1, time, error,  'DisplayName', 'error');
xlim(ax1_1, [time(1), 0]); ylabel(ax1_1, 'angle (deg)'); legend(ax1_1, 'location', 'eastoutside');

ax1_2 = subplot(3, 1, 2); hold on; grid on;
P_plot  = plot(ax1_2, time, P_term, 'DisplayName', 'P');
I_plot  = plot(ax1_2, time, I_term, 'DisplayName', 'I');
D_plot  = plot(ax1_2, time, D_term, 'DisplayName', 'D');
ff_plot = plot(ax1_2, time, ff_term, 'DisplayName', 'FF');
xlim(ax1_2, [time(1), 0]); legend(ax1_2, 'location', 'eastoutside');

ax1_3 = subplot(3, 1, 3); hold on; grid on;
output_plot = plot(ax1_3, time, P_term, 'DisplayName', 'output');
xlim(ax1_3, [time(1), 0]); xlabel(ax1_3, 'time (s)'); legend(ax1_3, 'location', 'eastoutside');

% --- FIR 滤波器内核计算 ---
k = 200; % 滤波器宽度（必须为偶数）
i = 0.5 - k / 2 : 1 : k/2;
kernel = sin((2 * pi * i) / k) ./ ((2 * pi * i) / k);
kernel(i == 0) = 1; % 修复 Sinc 函数在 0 点的 0/0 型极限值
kernel = kernel / sum(kernel); % 归一化

% 初始化高级分析中间变量
P_filt = nan(1, window - (k/2));
I_filt = nan(1, window - (k/2));
D_filt = nan(1, window - (k/2));
P_zero = nan(1, window - (k/2));
I_zero = nan(1, window - (k/2));
D_zero = nan(1, window - (k/2));
P_env  = nan(1, window - (k/2));
I_env  = nan(1, window - (k/2));
D_env  = nan(1, window - (k/2));

w = 0.01;       % 包络线滤波系数
w_zero = 0.05;  % 零点信号滤波系数
w_freq = 0.01;  % 频率滤波系数

P_zero_filt = nan(1, window - (k/2));
I_zero_filt = nan(1, window - (k/2));
D_zero_filt = nan(1, window - (k/2));
P_freq = nan(1, window - (k/2));
I_freq = nan(1, window - (k/2));
D_freq = nan(1, window - (k/2));
P_filt_freq = nan(1, window - (k/2));
I_filt_freq = nan(1, window - (k/2));
D_filt_freq = nan(1, window - (k/2));

time_sub = time(1 : window - k*0.5);

% --- Figure 2: 高级特性分析（滤波、去直流、过零检测） ---
fig2 = figure('Name', '2. Advanced Oscillation Analysis', 'Position', [660, 100, 750, 700]);
ax2_1 = subplot(3, 4, [1,2]); hold on; P_plot2 = plot(ax2_1, time, P_term); P_filt_plot = plot(ax2_1, time_sub, P_filt); xlim(ax2_1, [time(1), 0]); ylabel(ax2_1, 'P');
ax2_2 = subplot(3, 4, [5,6]); hold on; I_plot2 = plot(ax2_2, time, I_term); I_filt_plot = plot(ax2_2, time_sub, I_filt); xlim(ax2_2, [time(1), 0]); ylabel(ax2_2, 'I');
ax2_3 = subplot(3, 4, [9,10]); hold on; D_plot2 = plot(ax2_3, time, D_term); D_filt_plot = plot(ax2_3, time_sub, D_filt); xlim(ax2_3, [time(1), 0]); ylabel(ax2_3, 'D'); xlabel(ax2_3, 'time (s)');

ax2_4 = subplot(3, 4, 3); hold on; P_zero_plot = plot(ax2_4, time_sub, P_zero); P_zero_filt_plot = plot(ax2_4, time_sub, P_zero_filt, 'k'); P_env_plot = plot(ax2_4, time_sub, P_env); P_cross = scatter(ax2_4, time_sub(end), 0, '*', 'k'); xlim(ax2_4, [time(1), 0]); ylabel(ax2_4, 'P Zero-Centered');
ax2_5 = subplot(3, 4, 7); hold on; I_zero_plot = plot(ax2_5, time_sub, I_zero); I_zero_filt_plot = plot(ax2_5, time_sub, I_zero_filt, 'k'); I_env_plot = plot(ax2_5, time_sub, I_env); I_cross = scatter(ax2_5, time_sub(end), 0, '*', 'k'); xlim(ax2_5, [time(1), 0]); ylabel(ax2_5, 'I Zero-Centered');
ax2_6 = subplot(3, 4, 11); hold on; D_zero_plot = plot(ax2_6, time_sub, D_zero); D_zero_filt_plot = plot(ax2_6, time_sub, D_zero_filt, 'k'); D_env_plot = plot(ax2_6, time_sub, D_env); D_cross = scatter(ax2_6, time_sub(end), 0, '*', 'k'); xlim(ax2_6, [time(1), 0]); ylabel(ax2_6, 'D Zero-Centered'); xlabel(ax2_6, 'Time (s)');

ax2_7 = subplot(3, 4, 4); hold on; P_freq_plot = plot(ax2_7, time_sub, P_freq); P_filt_freq_plot = plot(ax2_7, time_sub, P_filt_freq); xlim(ax2_7, [time(1), 0]); ylabel(ax2_7, 'P Freq (Hz)');
ax2_8 = subplot(3, 4, 8); hold on; I_freq_plot = plot(ax2_8, time_sub, I_freq); I_filt_freq_plot = plot(ax2_8, time_sub, I_filt_freq); xlim(ax2_8, [time(1), 0]); ylabel(ax2_8, 'I Freq (Hz)');
ax2_9 = subplot(3, 4, 12); hold on; D_freq_plot = plot(ax2_9, time_sub, D_freq); D_filt_freq_plot = plot(ax2_9, time_sub, D_filt_freq); xlim(ax2_9, [time(1), 0]); ylabel(ax2_9, 'D Freq (Hz)'); xlabel(ax2_9, 'time (s)');

% --- Figure 3: 震荡状态综合概览 ---
fig3 = figure('Name', '3. Oscillation Summary Dashboard', 'Position', [1420, 300, 450, 400]);
ax3_1 = subplot(1, 2, 1); hold on; grid on; P_env_plot2 = plot(ax3_1, time_sub, P_env); I_env_plot2 = plot(ax3_1, time_sub, I_env); D_env_plot2 = plot(ax3_1, time_sub, D_env); xlim(ax3_1, [time(1), 0]); legend(ax3_1, 'P','I','D','location', 'northwest'); title(ax3_1, 'Envelope'); xlabel(ax3_1, 'time(s)');
ax3_2 = subplot(1, 2, 2); hold on; grid on; P_filt_freq_plot2 = plot(ax3_2, time_sub, P_filt_freq); I_filt_freq_plot2 = plot(ax3_2, time_sub, I_filt_freq); D_filt_freq_plot2 = plot(ax3_2, time_sub, D_filt_freq); xlim(ax3_2, [time(1), 0]); title(ax3_2, 'Filtered Freq (Hz)'); xlabel(ax3_2, 'time (s)');

%% 3. UDP 网络初始化
u = pnet('udpsocket', 9002);
pnet(u, 'setreadtimeout', 0);
bytes_read = 4 * 7; 

frame_count = 0;
P_cross_index = 0; I_cross_index = 0; D_cross_index = 0;

disp('融合版数据分析系统启动！正在实时捕获高级特性... (关闭图窗1即可退出)');

%% 4. 数据实时处理核心循环
while ishandle(fig1)
    % 非阻塞式轻量级轮询
    in_bytes = pnet(u, 'readpacket', bytes_read);
    
    if in_bytes > 0
        frame_count = frame_count + 1;
        
        % 解析流数据
        pid_info = double(pnet(u, 'read', 7, 'SINGLE', 'intel'));
        
        % 压入基础滑动窗口
        target  = [target(2:end),  pid_info(1)];
        actual  = [actual(2:end),  pid_info(2)];
        error   = [error(2:end),   pid_info(3)];
        P_term  = [P_term(2:end),  pid_info(4)];
        I_term  = [I_term(2:end),  pid_info(5)];
        D_term  = [D_term(2:end),  pid_info(6)];
        ff_term = [ff_term(2:end), pid_info(7)];
        
        % 进行 Sinc FIR 滤波运算（解决了上一版中括号放错位置的报错）
        P_filt = [P_filt(2:end), Sinc_FIR_filter(P_term(window - k + 1 : end), kernel)];
        I_filt = [I_filt(2:end), Sinc_FIR_filter(I_term(window - k + 1 : end), kernel)];
        D_filt = [D_filt(2:end), Sinc_FIR_filter(D_term(window - k + 1 : end), kernel)];
        
        % 去除直流分量（实现零均值对比，自动补偿滤波器延迟群偏移体 window - k/2）
        P_zero = [P_zero(2:end), P_term(window - (k/2)) - P_filt(end)];
        I_zero = [I_zero(2:end), I_term(window - (k/2)) - I_filt(end)];
        D_zero = [D_zero(2:end), D_term(window - (k/2)) - D_filt(end)];
        
        % 指数平滑算法提取震荡波形包络（幅值）
        if isnan(P_env(end))
            P_env = [P_env(2:end), abs(P_zero(end))];
            I_env = [I_env(2:end), abs(I_zero(end))];
            D_env = [D_env(2:end), abs(D_zero(end))];
        else
            P_env = [P_env(2:end), P_env(end) * (1 - w) + abs(P_zero(end)) * w];
            I_env = [I_env(2:end), I_env(end) * (1 - w) + abs(I_zero(end)) * w];
            D_env = [D_env(2:end), D_env(end) * (1 - w) + abs(D_zero(end)) * w];
        end
        
        % 对零点平衡信号做二次平滑，以稳定用于过零点检测
        if isnan(P_zero_filt(end))
            P_zero_filt = [P_zero_filt(2:end), P_zero(end)];
            I_zero_filt = [I_zero_filt(2:end), I_zero(end)];
            D_zero_filt = [D_zero_filt(2:end), D_zero(end)];
        else
            P_zero_filt = [P_zero_filt(2:end), P_zero_filt(end) * (1 - w_zero) + P_zero(end) * w_zero];
            I_zero_filt = [I_zero_filt(2:end), I_zero_filt(end) * (1 - w_zero) + I_zero(end) * w_zero];
            D_zero_filt = [D_zero_filt(2:end), D_zero_filt(end) * (1 - w_zero) + D_zero(end) * w_zero];
        end
        
        % --- 基于过零点检测算法实时估算震荡频率 ---
        % P 项频率检测
        if sign(P_zero_filt(end)) ~= sign(P_zero_filt(end-1)) && P_zero_filt(end-1) ~= 0
            P_freq_new = 1 / (2 * (frame_count - P_cross_index) * dt);
            P_cross_index = frame_count;
            set(P_cross, 'YData', 0, 'XData', time_sub(end));
        else
            P_freq_new = P_freq(end);
        end
        
        % I 项频率检测
        if sign(I_zero_filt(end)) ~= sign(I_zero_filt(end-1)) && I_zero_filt(end-1) ~= 0
            I_freq_new = 1 / (2 * (frame_count - I_cross_index) * dt);
            I_cross_index = frame_count;
            set(I_cross, 'YData', 0, 'XData', time_sub(end));
        else
            I_freq_new = I_freq(end);
        end
        
        % D 项频率检测
        if sign(D_zero_filt(end)) ~= sign(D_zero_filt(end-1)) && D_zero_filt(end-1) ~= 0
            D_freq_new = 1 / (2 * (frame_count - D_cross_index) * dt);
            D_cross_index = frame_count;
            set(D_cross, 'YData', 0, 'XData', time_sub(end));
        else
            D_freq_new = D_freq(end);
        end
        
        P_freq = [P_freq(2:end), P_freq_new];
        I_freq = [I_freq(2:end), I_freq_new];
        D_freq = [D_freq(2:end), D_freq_new];
        
        % 频率信号低通平滑滤波
        if isnan(P_filt_freq(end))
            P_filt_freq = [P_filt_freq(2:end), P_freq(end)];
            I_filt_freq = [I_filt_freq(2:end), I_freq(end)];
            D_filt_freq = [D_filt_freq(2:end), D_freq(end)];
        else
            P_filt_freq = [P_filt_freq(2:end), P_filt_freq(end) * (1 - w_freq) + P_freq(end) * w_freq];
            I_filt_freq = [I_filt_freq(2:end), I_filt_freq(end) * (1 - w_freq) + I_freq(end) * w_freq];
            D_filt_freq = [D_filt_freq(2:end), D_filt_freq(end) * (1 - w_freq) + D_freq(end) * w_freq];
        end
        
        %% 5. 高效画布更新渲染控制区
        % 每 5 帧冲刷一次画布，兼顾实时性与极低的 CPU 负载, 10 hz
        % 每 2 帧冲刷一次画布，25 hz
        if mod(frame_count, 2) == 0 
            % --- 图窗 1 数据刷新 ---
            set(target_plot, 'YData', target); set(actual_plot, 'YData', actual); set(error_plot, 'YData', error);
            set(P_plot, 'YData', P_term); set(I_plot, 'YData', I_term); set(D_plot, 'YData', D_term); set(ff_plot, 'YData', ff_term);
            set(output_plot, 'YData', P_term + I_term + D_term + ff_term);
            
            % --- 图窗 2 数据刷新 ---
            set(P_plot2, 'YData', P_term); set(P_filt_plot, 'YData', P_filt);
            set(I_plot2, 'YData', I_term); set(I_filt_plot, 'YData', I_filt);
            set(D_plot2, 'YData', D_term); set(D_filt_plot, 'YData', D_filt);
            
            set(P_zero_plot, 'YData', P_zero); set(P_zero_filt_plot, 'YData', P_zero_filt); set(P_env_plot, 'YData', P_env);
            set(I_zero_plot, 'YData', I_zero); set(I_zero_filt_plot, 'YData', I_zero_filt); set(I_env_plot, 'YData', I_env);
            set(D_zero_plot, 'YData', D_zero); set(D_zero_filt_plot, 'YData', D_zero_filt); set(D_env_plot, 'YData', D_env);
            
            set(P_freq_plot, 'YData', P_freq); set(P_filt_freq_plot, 'YData', P_filt_freq);
            set(I_freq_plot, 'YData', I_freq); set(I_filt_freq_plot, 'YData', I_filt_freq);
            set(D_freq_plot, 'YData', D_freq); set(D_filt_freq_plot, 'YData', D_filt_freq);
            
            % --- 图窗 3 数据刷新 ---
            set(P_env_plot2, 'YData', P_env); set(I_env_plot2, 'YData', I_env); set(D_env_plot2, 'YData', D_env);
            set(P_filt_freq_plot2, 'YData', P_filt_freq); set(I_filt_freq_plot2, 'YData', I_filt_freq); set(D_filt_freq_plot2, 'YData', D_filt_freq);
            
            % 限制刷新率防卡顿
            % drawnow limitrate;
            % 高刷新率
            drawnow nocallbacks;
        end
    end
end

% 退出清理
pnet('closeall');
disp('遥测绘图已平稳停止，网络端口已完全释放。');

%% 5. 局域有限冲激响应滤波器子函数
function out = Sinc_FIR_filter(signal_window, kernel)
    out = sum(signal_window(:) .* kernel(:));
end