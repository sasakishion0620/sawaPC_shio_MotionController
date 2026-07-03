clear;
clc;
close all;

%% =========================
% 設定
%% =========================

Ts = 0.001;      % サンプリング周期 [s]
np = 1;          % 極数：1次遅れ
nz = 0;          % 零点なし
ioDelay = 0;     % 無駄時間なし

%% =========================
% CSVを複数選択
%% =========================

[fileNames, folderPath] = uigetfile("*.csv", ...
    "モデル化に使うCSVを選択してください", ...
    "MultiSelect", "on");

if isequal(fileNames,0)
    disp("CSV選択がキャンセルされました");
    return;
end

if ischar(fileNames)
    fileNames = {fileNames};
end

numFiles = length(fileNames);

fprintf("選択されたCSV数: %d\n", numFiles);

%% =========================
% 結果保存用
%% =========================

u_list = zeros(numFiles,1);
K_list = zeros(numFiles,1);
T_list = zeros(numFiles,1);
L_list = zeros(numFiles,1);

models = cell(numFiles,1);

%% =========================
% 全CSV重ね描画用
%% =========================

figure(100);
hold on;
grid on;

figure(101);   % 実測値だけ
hold on;
grid on;

figure(102);   % モデルだけ
hold on;
grid on;


colors = [
    0.3010 0.7450 0.9330;   % 水色
    0.8500 0.3250 0.0980;   % オレンジ
    0.9290 0.6940 0.1250;   % 黄色
    0.4940 0.1840 0.5560    % 紫
];

%%凡例の名前
legendNames = {
    '121 μs'
    '242 μs'
    '363 μs'
    '484 μs'
};

h100 = gobjects(numFiles,1);
h101 = gobjects(numFiles,1);
h102 = gobjects(numFiles,1);


%% =========================
% 各CSVをモデル化
%% =========================

for i = 1:numFiles

    filename = fileNames{i};
    fullpath = fullfile(folderPath, filename);

    fprintf("\n=============================\n");
    fprintf("%d個目のCSVを処理中: %s\n", i, filename);

    raw = readmatrix(fullpath);

    u = raw(:,3);     % 3列目：入力
    y = raw(:,4);     % 4列目：出力

    %% 入力振幅を取得
    u_initial = mean(u(1:round(0.2*length(u))));
    u_final   = mean(u(round(0.7*length(u)):end));

    u_amp = u_final - u_initial;
    u_list(i) = u_amp;

    %% iddata作成
    data = iddata(y, u, Ts);

    %% 1次遅れモデル同定
    model = tfest(data, np, nz, ioDelay);
    models{i} = model;

    %% パラメータ取得
    K = dcgain(model);

    [~, den] = tfdata(model, "v");

    % 1次遅れモデルが
    % G(s) = K / (T s + 1)
    % ではなく
    % G(s) = b / (s + a)
    % の形で出るので，T = 1/a
    T = 1 / den(2);

    L = model.IODelay;

    K_list(i) = K;
    T_list(i) = T;
    L_list(i) = L;

    fprintf("入力振幅 u = %.6f\n", u_amp);
    fprintf("ゲイン K = %.6f\n", K);
    fprintf("時定数 T = %.6f [s]\n", T);
    fprintf("無駄時間 L = %.6f [s]\n", L);

    %% 実測応答とモデル応答を重ねて表示
    t = (0:length(y)-1)' * Ts;
    y_model = lsim(model, u, t);

    figure;
    plot(t, y, "LineWidth", 1.2);
    hold on;
    plot(t, y_model, "--", "LineWidth", 1.5);
    grid on;

    xlabel("時間 [s]");
    ylabel("出力 y");
    title("実測応答とモデル応答 : " + filename);
    xlim([0 10]);   % ←追加
    ylim([0 16]);
    % legend("実測", "モデル");
    
    %% =========================
% 全CSV比較用にも追加
%% =========================

figure(100);

c = colors(i,:);

h100(i) = plot(t, y, ...
    "Color", c, ...
    "LineWidth", 2, ...
    "DisplayName", legendNames{i});

plot(t, y_model, ...
    "--", ...
    "Color", c, ...
    "LineWidth", 2, ...
    "HandleVisibility","off");

%% 実測値だけ重ね描き
figure(101);

%% 実測値だけ重ね描き
figure(101);

h101(i) = plot(t, y, ...
    "-", ...
    "Color", c, ...
    "LineWidth", 2, ...
    "DisplayName", legendNames{i});

%% モデルだけ重ね描き
figure(102);

h102(i) = plot(t, y_model, ...
    "-", ...
    "Color", c, ...
    "LineWidth", 2, ...
    "DisplayName", legendNames{i});
end


%% =========================
% 全CSV比較グラフ仕上げ
%% =========================
figure(100);

set(gcf, "Position", [100 100 1000 700]);

grid on;

xlabel("時間 [s]", "FontSize", 22);
ylabel("発揮力 x [N]", "FontSize", 22);

ax = gca;
ax.FontSize = 20;
ax.LineWidth = 1.2;

xlim([0 10]);
ylim([0 16]);

legend(h100(end:-1:1), legendNames(end:-1:1), ...
    "Location", "northwest", ...
    "FontSize", 18);

set(gcf, "PaperPositionMode", "auto");
print(gcf, "figure100", "-depsc");

% title("全CSVの実測応答とモデル応答");



%% =========================
% 実測値だけの重ね描きグラフ仕上げ
%% =========================

figure(101);

set(gcf, "Position", [100 100 1000 700]);

grid on;

xlabel("時間 [s]", "FontSize", 22);
ylabel("発揮力 x [N]", "FontSize", 22);

ax = gca;
ax.FontSize = 20;
ax.LineWidth = 1.2;

xlim([0 10]);
ylim([0 16]);

legend(h101(end:-1:1), legendNames(end:-1:1), ...
    "Location", "northwest", ...
    "FontSize", 18);

set(gcf, "PaperPositionMode", "auto");
print(gcf, "figure101", "-depsc");

% title("全CSVの実測応答");


%% =========================
% モデルだけの重ね描きグラフ仕上げ
%% =========================
figure(102);

set(gcf, "Position", [100 100 1000 700]);

grid on;

xlabel("時間 [s]", "FontSize", 22);
ylabel("発揮力 x [N]", "FontSize", 22);

ax = gca;
ax.FontSize = 20;
ax.LineWidth = 1.2;

xlim([0 10]);
ylim([0 16]);

legend(h102(end:-1:1), legendNames(end:-1:1), ...
    "Location", "northwest", ...
    "FontSize", 18);

set(gcf, "PaperPositionMode", "auto");
print(gcf, "figure102", "-depsc");


% title("全CSVのモデル応答");
%% =========================
% K(u) = A exp(-B u) + C で非線形フィット
%% =========================

fitFunc = @(p,u) p(1)*exp(-p(2)*u) + p(3);

% y∞ = K × 入力振幅
yss_model = K_list .* u_list;

p0 = [ ...
   -(max(yss_model)-min(yss_model)), ...
    0.001, ...
    max(yss_model)];

p = lsqcurvefit(fitFunc, p0, u_list, yss_model);

A = p(1);
B = p(2);
C = p(3);

fprintf("\n=============================\n");
fprintf("非線形ゲインモデル\n");
fprintf("y∞(u) = A exp(-B u) + C\n");
fprintf("A = %.6f\n", A);
fprintf("B = %.6f\n", B);
fprintf("C = %.6f\n", C);

%% =========================
% 非線形フィット結果表示
%% =========================

u_fit = linspace(min(u_list), max(u_list), 100);
y_fit = fitFunc(p, u_fit);

figure;

plot(u_list, yss_model, 'o', ...
    'MarkerSize', 8, ...
    'MarkerFaceColor', 'b');
hold on;
plot(u_fit, y_fit, "LineWidth", 2);
grid on;

xlabel("入力パルス幅 u　[µs]", "FontSize", 22);
ylabel("定常値 x_\infty", "FontSize", 22);

ax = gca;
ax.FontSize = 18;      % ←目盛りの数字の大きさ
ax.LineWidth = 1.2;    % ←軸の太さ

set(gcf,"Position",[100 100 800 600]);  % ←図の大きさ

% title("定常値の非線形モデル");
% legend("同定結果", "非線形近似");

%% =========================
% 結果を表にまとめる
%% =========================

resultTable = table( ...
    string(fileNames(:)), ...
    u_list, ...
    K_list, ...
    T_list, ...
    L_list, ...
    'VariableNames', {'File','u_amp','K','T','L'} ...
);

disp(resultTable);

%% =========================
% T平均に使うファイルをクリックで選択
%% =========================

listText = strings(numFiles,1);

for i = 1:numFiles
    listText(i) = sprintf("%d : %s    T = %.6f [s]", ...
        i, fileNames{i}, T_list(i));
end

[useIdx, ok] = listdlg( ...
    "PromptString", "T平均に使うCSVを選択してください", ...
    "SelectionMode", "multiple", ...
    "ListString", listText);

if ok == 0
    disp("T平均の選択がキャンセルされました");
    return;
end

T_mean = mean(T_list(useIdx));
k_mean = 1 / T_mean;

fprintf("\n=============================\n");
fprintf("T平均に使用したCSV番号 : ");
fprintf("%d ", useIdx);
fprintf("\n");

fprintf("T平均 = %.6f [s]\n", T_mean);
fprintf("k = 1/T = %.6f [1/s]\n", k_mean);


%% =========================
% 制御用パラメータ出力
%% =========================

alpha = k_mean;

fprintf("\n=============================\n");
fprintf("制御用パラメータ\n");
fprintf("{\n");
fprintf('  "A": %.2f,\n', A);
fprintf('  "B": %.1g,\n', B);
fprintf('  "C": %.2f,\n', C);
fprintf('  "alpha": %.2f,\n', alpha);
fprintf('  "k": %.2f,\n', k_mean);
fprintf('  "T": %.2f\n', T_mean);
fprintf("}\n");