% net_composite.m
rng(pi);
close all;
hiddenLayers = [15];
trainingFcn = "trainbfg";
transferFcn = "tansig";

train_csv = "data_acquisition/data/11.10.2020 00.24.19.csv";
val_csv = "data_acquisition/data/11.09.2020 18.15.14.csv";

%% Loading data
%t, thetaR, thetaE, dOrtoR, dOrtoE, sensors(6:18)
data = readmatrix(train_csv);

sensors = data(:, 6:size(data, 2))';
thetaR = data(:, 2)';
dOrtoR = data(:, 4)';
output = [thetaR; dOrtoR];

%% Network Definition
net = feedforwardnet(hiddenLayers);
net.trainFcn = trainingFcn;
net.layers{1}.transferFcn = transferFcn;
net.trainParam.showWindow=0;
net.trainParam.epochs = 5000;
%view(net)
%% Training
net = train(net, sensors, output);

%% Validation
data_val = readmatrix(val_csv);
sensors_val = data_val(:, 6:size(data_val, 2))';

t = data_val(:, 1);
thetaR_val = data_val(:, 2)';
dOrtoR_val = data_val(:, 4)';

thetaE_val = data_val(:, 3)';
dOrtoE_val = data_val(:, 5)';

y = net(sensors_val);
thetaE = y(1, :);
dOrtoE = y(2, :);

thetaMSENetwork = goodnessOfFit(thetaE, thetaR_val, "MSE");
dOrtoMSENetwork = goodnessOfFit(dOrtoE, dOrtoR_val, "MSE");

thetaMSEGeometric = goodnessOfFit(thetaE_val, thetaR_val, "MSE");
dOrtoMSEGeometric = goodnessOfFit(dOrtoE_val, dOrtoR_val, "MSE");

%% Neural Network Evaluation

figure('Renderer', 'painters', 'Position', [10 10 900 700])
subplot(2, 1, 1);
plot(t, thetaR_val*180/pi, 'b-', 'LineWidth', 2,...
     'DisplayName', "\theta_{R} - Referência"), hold on;

plot(t, thetaE*180/pi, 'r-.', 'LineWidth', 1.5,...
    'DisplayName',...
    sprintf("\\theta_{EN} - Rede Neural, MSE: %.2f", thetaMSENetwork));

title("Ângulo de desvio", 'FontSize', 14);
legend('FontSize', 8);
grid;
xlabel("$t (s)$", "interpreter", "latex", 'FontSize', 14);
ylabel("\theta (º)", 'FontSize', 14);

% Ortogonal Error
subplot(2, 1, 2);
plot(t, dOrtoR_val, 'b-', 'LineWidth', 2,...
     'DisplayName', "\Gamma_{R} - Referência"), hold on;

plot(t, dOrtoE, 'r-.', 'LineWidth', 1.5,...
    'DisplayName',...
    sprintf("\\Gamma_{EN} - Rede Neural, MSE: %.2f", dOrtoMSENetwork));

title("Deslocamento ortogonal", 'FontSize', 14);

legend('FontSize', 8);
grid;
xlabel("$t (s)$", "interpreter", "latex", 'FontSize', 14);
ylabel("\Gamma (m)", 'FontSize', 14);
%sgtitle("Comparação de performance da Rede Neural com saída composta", 'FontSize', 14);

%% Geometric Algorithm Evaluation
figure('Renderer', 'painters', 'Position', [10 10 900 700])
subplot(2, 1, 1);
plot(t, thetaR_val*180/pi, 'b-', 'LineWidth', 2,...
     'DisplayName', "\theta_{R} - Referência"), hold on;

 plot(t, thetaE_val*180/pi, 'r-', 'LineWidth', 1.5,...
    'DisplayName',...
    sprintf("\\theta_{EG} - Algoritmo Geométrico, MSE: %.2f", thetaMSEGeometric));

title("Ângulo de desvio", 'FontSize', 14);
legend('FontSize', 8);
grid;
xlabel("$t (s)$", "interpreter", "latex", 'FontSize', 14);
ylabel("\theta (º)", 'FontSize', 14);

% Ortogonal Error
subplot(2, 1, 2);
plot(t, dOrtoR_val, 'b-', 'LineWidth', 2,...
     'DisplayName', "\Gamma_{R} - Referência"), hold on;

plot(t, dOrtoE_val, 'r-', 'LineWidth', 1.5,...
    'DisplayName',...
    sprintf("\\Gamma_{EG} - Algoritmo Geométrico, MSE: %.2f", dOrtoMSEGeometric));

title("Deslocamento ortogonal", 'FontSize', 14);

legend('FontSize', 8);
grid;
xlabel("$t (s)$", "interpreter", "latex", 'FontSize', 14);
ylabel("\Gamma (m)", 'FontSize', 14);
ylim([-0.5, 0.5]);