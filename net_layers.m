% net_independent.m
rng(pi);
close all;
clear;

scenarios = {};
scenarios{1}.hiddenLayers = [15];
scenarios{2}.hiddenLayers = [15, 15, 15];
scenarios{3}.hiddenLayers = [15, 15, 15, 15, 15];
scenarios{4}.hiddenLayers = [15, 15, 15, 15, 15, 15, 15];

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

%% Network Definition
for index = 1:length(scenarios)
    scenario = scenarios{index};

    net_theta = feedforwardnet(scenario.hiddenLayers);
    net_theta.trainFcn = trainingFcn;
    net_theta.layers{1}.transferFcn = transferFcn;
    net_theta.trainParam.showWindow=0;
    net_theta.trainParam.epochs = 5000;

    net_dOrto = feedforwardnet(scenario.hiddenLayers);
    net_dOrto.trainFcn = trainingFcn;
    net_dOrto.layers{1}.transferFcn = transferFcn;
    net_dOrto.trainParam.showWindow=0;
    net_dOrto.trainParam.epochs = 5000;

    % Training
    net_theta = train(net_theta, sensors, thetaR);
    net_dOrto = train(net_dOrto, sensors, dOrtoR);

    %% Validation
    data_val = readmatrix(val_csv);
    sensors_val = data_val(:, 6:size(data_val, 2))';

    t = data_val(:, 1);
    thetaR_val = data_val(:, 2)';
    dOrtoR_val = data_val(:, 4)';

    thetaE_val = data_val(:, 3)';
    dOrtoE_val = data_val(:, 5)';

    thetaE = net_theta(sensors_val);
    dOrtoE = net_dOrto(sensors_val);

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
end