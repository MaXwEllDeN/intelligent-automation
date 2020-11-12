%d. o tipo de partição de dados (treinamento, teste e validação)

rng(pi);
[X,T] = prprob;

scenarios = {};
scenarios{1}.divideFcn = 'dividerand';
scenarios{2}.divideFcn = 'divideblock';
scenarios{3}.divideFcn = 'divideint';
scenarios{4}.divideFcn = 'divideind';

noiseLevels = 0:.05:1;
numLevels = length(noiseLevels);

percErrors_clean = zeros(numLevels, length(scenarios));
percErrors_noisy = zeros(numLevels, length(scenarios));

%% Creating and training Neural Networks

for index = 1:length(scenarios)
    scenario = scenarios{index};

    fprintf("Executando treinamento para o cenário %d", index);        
    net1 = feedforwardnet([10]);
    net1.trainFcn = 'traingdx';
    net1.trainParam.showWindow=0;
    net1.layers{1}.transferFcn = 'tansig';
    net1.divideFcn = scenario.divideFcn;

    if strcmp(scenario.divideFcn, 'divideind')
        net1.divideParam.trainInd = [1];
        net1.divideParam.trainInd = [12];
        net1.divideParam.valInd = [22];
    else
        net1.divideParam.trainRatio = 0.70;
        net1.divideParam.testRatio = 0.15;
        net1.divideParam.valRatio = 0.15;              
    end
    
    % Training the first Neural Network
    net1 = train(net1, X, T, nnMATLAB);

    numNoise = 30;
    Xn = min(max(repmat(X,1,numNoise)+randn(35,size(T, 2)*numNoise)*0.2,0),1);
    Tn = repmat(T, 1, numNoise);

    net2 = feedforwardnet([10]);
    net2.trainFcn = 'traingdx';
    net2.trainParam.showWindow=0;
    net2.layers{1}.transferFcn = 'tansig';
    net2.divideFcn = scenario.divideFcn;
    
    if strcmp(scenario.divideFcn, 'divideind')
        net2.divideParam.trainInd = [1];
        net2.divideParam.testInd = [12];
        net2.divideParam.valInd = [22];
    else
        net2.divideParam.trainRatio = 0.70;
        net2.divideParam.testRatio = 0.15;
        net2.divideParam.valRatio = 0.15;        
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
    saveas(fig, sprintf("plots/divideFcn%d.png", index));
end