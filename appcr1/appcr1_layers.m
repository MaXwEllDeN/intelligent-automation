%a. a quantidade de camadas escondidas

rng(pi);
[X,T] = prprob;

scenarios = {};
scenarios{1}.hiddenLayers = [10];
scenarios{2}.hiddenLayers = [10, 10, 10];
scenarios{3}.hiddenLayers = [10, 10, 10, 10, 10];
scenarios{4}.hiddenLayers = [10, 10, 10, 10, 10, 10, 10];

noiseLevels = 0:.05:1;
numLevels = length(noiseLevels);

percErrors_clean = zeros(numLevels, length(scenarios));
percErrors_noisy = zeros(numLevels, length(scenarios));

%% Creating and training Neural Networks

for index = 1:length(scenarios)
    scenario = scenarios{index};

    fprintf("Executando treinamento para o cenário %d", index);        
    net1 = feedforwardnet(scenario.hiddenLayers);
    net1.trainFcn = 'traingdx';
    net1.trainParam.showWindow=0;

    for i = 1:length(scenario.hiddenLayers)
        net1.layers{i}.transferFcn = 'tansig';
    end

    % Training the first Neural Network
    net1 = train(net1, X, T, nnMATLAB);

    numNoise = 30;
    Xn = min(max(repmat(X,1,numNoise)+randn(35,size(T, 2)*numNoise)*0.2,0),1);
    Tn = repmat(T, 1, numNoise);

    net2 = feedforwardnet(scenario.hiddenLayers);
    net2.trainFcn = 'traingdx';
    net2.trainParam.showWindow=0;

    for i = 1:length(scenario.hiddenLayers)
        net2.layers{i}.transferFcn = 'tansig';
    end

    net2 = train(net2,Xn,Tn,nnMATLAB);

    % Testing both Neural Networks
    for i = 1:numLevels
      Xtest = min(max(repmat(X,1,numNoise)+randn(35,size(T, 2)*numNoise)*noiseLevels(i),0),1);
      Y1 = net1(Xtest);
      percErrors_clean(i, index) = sum(sum(abs(Tn-compet(Y1))))/(size(T, 2)*numNoise*2);

      Y2 = net2(Xtest);
      percErrors_noisy(i, index) = sum(sum(abs(Tn-compet(Y2))))/(size(T, 2)*numNoise*2);
    end
end
%% Ploting data
for index = 1:length(scenarios)
    fig = figure('visible', 'off');
    plot(noiseLevels, percErrors_clean(:, index)*100, '-',...
        'DisplayName', 'Treinada com sinal limpo', 'LineWidth', 2);
    hold on;
    plot(noiseLevels, percErrors_noisy(:, index)*100, '-.',...
        'DisplayName', 'Treinada com sinal ruidoso', 'LineWidth', 2);

    xlabel('Nível de ruído', 'FontSize', 14);
    ylabel('Erros', 'FontSize', 14);
    legend('Location','SouthEast', 'FontSize', 14);
    grid;
    title(["Porcentagens de erro de reconhecimento",...
           "para diferentes níveis de ruído"], 'FontSize', 14);
    saveas(fig, sprintf("plots/layers%d.png", index));
end