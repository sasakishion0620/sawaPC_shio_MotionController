clear;
clc;
close all;

%% =========================
% 設定
%% =========================

watchFolder = "/home/tomoya/sawaPC_shio_MotionController/data/2026_06_01";   % CSVを保存するフォルダ
csvPattern  = "*.csv";           % 対象CSV

Ts = 0.001;      % サンプリング周期[s]
np = 1;          % 極数 1次遅れ
nz = 0;          % 零点なし
ioDelay = 0;     % 無駄時間あり

numFiles = 4;    % 4つのCSVがそろったら処理
check_interval = 1.0;

cd(watchFolder);

disp("CSV監視開始");

%% =========================
% CSVが4個そろうまで待つ
%% =========================

while true

    files = dir(csvPattern);%.csv全部とってくる

    if length(files) >= numFiles
        disp("CSVが集まりました");
        break;
    end

    fprintf("現在のCSV数: %d / %d\n", length(files), numFiles);
    pause(check_interval);

end

%% =========================
% 更新日時順に並べる
%% =========================

[~,idx] = sort([files.datenum]);
files = files(idx);

files = files(1:numFiles);

%% =========================
% 結果保存用
%% =========================

u_list = zeros(numFiles,1);   % 入力振幅
K_list = zeros(numFiles,1);   % ゲイン
T_list = zeros(numFiles,1);   % 時定数
L_list = zeros(numFiles,1);   % 無駄時間

models = cell(numFiles,1);    % 各モデル保存用

%% =========================
% 各CSVをモデル化
%% =========================

for i = 1:numFiles

    filename = files(i).name;

    fprintf("\n=============================\n");
    fprintf("%d個目のCSVを処理中: %s\n", i, filename);

    %% CSV読み込み
    raw = readmatrix(filename);

    u = raw(:,3);     % 1列目: 入力
    y = raw(:,4);     % 2列目: 出力

    figure;

subplot(2,1,1);
plot(u);
grid on;
title("入力 u");

subplot(2,1,2);
plot(y);
grid on;
title("出力 y");


    %% 入力振幅を自動取得
    u_initial = mean(u(1:round(0.2*length(u))));
    u_final   = mean(u(round(0.7*length(u)):end));

    u_amp = u_final - u_initial;

    u_list(i) = u_amp;

    %% iddata作成
    data = iddata(y,u,Ts);

    %% 1次遅れ＋無駄時間で同定
    model = tfest(data,np,nz,ioDelay);

    models{i} = model;

    %% パラメータ取得
    K = dcgain(model);

  den = model.Denominator;
  T = 1 / den(2);

    L = model.IODelay;

    K_list(i) = K;
    T_list(i) = T;
    L_list(i) = L;

    %% 結果表示
    fprintf("入力振幅 u = %.6f\n", u_amp);
    fprintf("ゲイン K = %.6f\n", K);
    fprintf("時定数 T = %.6f [s]\n", T);
    fprintf("無駄時間 L = %.6f [s]\n", L);

    %% 比較グラフ
    figure(i);
    compare(data,model);
    title("実測 vs 1次遅れ+無駄時間モデル : " + filename);

end

%% =========================
% K(u) = A exp(-B u) + C で非線形フィット
%% =========================

fitFunc = @(p,u) p(1)*exp(-p(2)*u) + p(3);
yss_model = K_list .* u_list;
p0 = [ ...
   -(max(yss_model)-min(yss_model)), ...
    0.001, ...
    max(yss_model)];
% p = [A, B, C]
% Aは負になることがある
% Kがuとともに増える場合，A<0になる



p = lsqcurvefit(fitFunc,p0,u_list,yss_model);

A = p(1);
B = p(2);
C = p(3);

fprintf("\n=============================\n");
fprintf("非線形ゲインモデル\n");
fprintf("K(u) = A exp(-B u) + C\n");
fprintf("A = %.6f\n", A);
fprintf("B = %.6f\n", B);
fprintf("C = %.6f\n", C);

%% =========================
% フィット結果表示
%% =========================

u_fit = linspace(min(u_list),max(u_list),100);
y_fit = fitFunc(p,u_fit);

figure;
plot(u_list,yss_model,"o","MarkerSize",8);
hold on;
plot(u_fit,y_fit,"LineWidth",2);
grid on;

xlabel("入力振幅 u");
ylabel("定常値 y_\infty")
title("定常値の非線形モデル");
legend("同定結果","非線形近似");

%% =========================
% 結果を表にまとめる
%% =========================

resultTable = table( ...
    string({files.name})', ...
    u_list, ...
    K_list, ...
    T_list, ...
    L_list, ...
    'VariableNames', {'File','u_amp','K','T','L'} ...
);

disp(resultTable);

%% =========================
% T平均に使うファイル選択
%% =========================

disp("----- T一覧 -----");

for i = 1:numFiles
    fprintf("%d : %s  T=%.6f\n", ...
        i, files(i).name, T_list(i));
end

useIdx = input( ...
    '平均に使用する番号を入力してください (例:[1 2 4]) : ');

T_mean = mean(T_list(useIdx));
k_mean = 1/T_mean;

fprintf("\n=============================\n");
fprintf("使用ファイル番号 : ");
fprintf("%d ", useIdx);
fprintf("\n");

fprintf("T平均 = %.6f [s]\n", T_mean);
fprintf("k = %.6f [1/s]\n", k_mean);

%% =========================
% Tの平均値とkを計算
%% =========================

T_mean = mean(T_list);
k_mean = 1 / T_mean;

fprintf("\n=============================\n");
fprintf("Tの平均値 = %.6f [s]\n", T_mean);
fprintf("k = 1/T = %.6f [1/s]\n", k_mean);
%% =========================
% 保存
%% =========================

save("all_models.mat","models","u_list","K_list","T_list","L_list","A","B","C");

writetable(resultTable,"model_results.csv");

param_nonlinear = [A B C];
writematrix(param_nonlinear,"nonlinear_gain_param.txt");

disp("保存完了");