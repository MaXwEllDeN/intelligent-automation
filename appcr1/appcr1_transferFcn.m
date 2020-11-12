%c. o tipo de função de ativação

rng(pi);
[X,T] = prprob;

scenarios = {};
scenarios{1}.transferFcn = 'tansig';
scenarios{2}.transferFcn = 'purelin';
scenarios{3}.transferFcn = 'radbasn';
scenarios{4}.transferFcn = 'satlins';

%scenarios{2}.transferFcn = 'compet';
%scenarios{3}.transferFcn = 'elliotsig';
%scenarios{4}.transferFcn = 'hardlim';
%scenarios{5}.transferFcn = 'hardlims';
%scenarios{6}.transferFcn = 'logsig';
%scenarios{7}.transferFcn = 'netinv';
%scenarios{8}.transferFcn = 'poslin';
%scenarios{9}.transferFcn = 'purelin';
%scenarios{10}.transferFcn = 'radbas';
%scenarios{11}.transferFcn = 'radbasn';
%scenarios{12}.transferFcn = 'satlin';
%scenarios{13}.transferFcn = 'satlins';
%scenarios{14}.transferFcn = 'softmax';
%scenarios{15}.transferFcn = 'tansig';
%scenarios{16}.transferFcn = 'tribas';

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

    net1.layers{1}.transferFcn = scenario.transferFcn;

    % Training the first Neural Network
    net1 = train(net1, X, T, nnMATLAB);

    numNoise = 30;
    Xn = min(max(repmat(X,1,numNoise)+randn(35,size(T, 2)*numNoise)*0.2,0),1);
    Tn = repmat(T, 1, numNoise);

    net2 = feedforwardnet([10]);
    net2.trainFcn = 'traingdx';
    net2.trainParam.showWindow=0;

    net2.layers{1}.transferFcn = scenario.transferFcn;

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
    saveas(fig, sprintf("plots/transferFcn%d.png", index));
end