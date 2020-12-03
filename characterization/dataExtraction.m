%% Motor electrical characteristics
global i_n
global s_n

f = 60;         % Hz
w = 2*pi*f;     % rad/s
c_n = 4.30;     % Nm
i_n = 3.46;     % A - @ 200V
v_n = 100;
s_n = sqrt(3)*v_n * i_n;
%% Genetic Algorithm parameters
global PARAM_MAX_VALUE
PARAM_MAX_VALUE = 4;

popSize = 2;
maxGens = 20;

global genotypeSize
genotypeSize = 44;
probCrossover = 1;          % The probability of crossing over. 
probMutation = 1;           % The mutation probability (per bit)

verboseFlag = 1;            % 1 => display details of each generation
                            % 0 => run quietly

%% Reading data from WEG catalog

global Ixs
global Pxs
global eval_s

Ixs = csvread("B - Corrente.csv");
Cxs = csvread("A - Conjugado.csv");
Pxs = zeros(size(Cxs));

for i = 1:length(Ixs)
   Ixs(i, 1) = (100 - Ixs(i, 1))/100;
   Ixs(i, 2) = Ixs(i, 2);
end

for i = 1:length(Cxs)
   Cxs(i, 1) = (100 - Cxs(i, 1))/100;
   Cxs(i, 2) = Cxs(i, 2) * c_n;
   
   % P = wT
   Pxs(i, 1) = Cxs(i, 1);
   Pxs(i, 2) = w * Cxs(i, 2) / s_n;
end

%eval_s = Ixs(:, 1);
eval_s = 0.01:0.01:1-0.01;
%% Main Scope

pop = (2^44 - 1)*rand(popSize, 1);

eliteIndiv = [];
eliteFitness = -realmax;

avgFitnessHist = zeros(1, maxGens + 1);
maxFitnessHist = zeros(1, maxGens + 1);

rs = 0.127;
x1 = 0.481;
x2 = 2.56;
r2 = 0.163;

yp = power(eval_s, rs, x1, x2, r2);
%yp = current(eval_s, rs, x1, x2, r2);
plot(eval_s, yp, 'DisplayName', 'P(s, \theta)');
hold on;
plot(eval_s, catalogPower(eval_s), 'DisplayName', 'P(s)');
grid on;
legend;

for gen=0:maxGens
    fitnessVals = evalFitness(pop);

    [maxFitnessHist(1, gen + 1), maxIndex] = max(fitnessVals);    
    avgFitnessHist(1, gen + 1) = mean(fitnessVals);

    if eliteFitness < maxFitnessHist(   gen + 1)
        eliteFitness = maxFitnessHist(gen + 1);
        eliteIndiv = pop(maxIndex, :);
    end

    if isinf(maxFitnessHist(gen+1))
        break;
    end

    roulette_p = fitnessVals / sum(fitnessVals);
    parentIndices = randsample([1:popSize], popSize, true, roulette_p);

    % deterimine the first parents of each mating pair
    firstParents = dec2bin(pop(parentIndices(1:popSize/2),:), genotypeSize);

    % determine the second parents of each mating pair
    secondParents = dec2bin(pop(parentIndices(popSize/2+1:end),:), genotypeSize);

    [offspringA, offspringB] = crossover(firstParents, secondParents, probCrossover);
    [offspringA, offspringB] = mutate(offspringA, offspringB, probMutation);

    % display the generation number, the average Fitness of the population,
    % and the maximum fitness of any individual in the population
    if verboseFlag
        display(['gen=' num2str(gen, '%.3d') '   avgFitness=' ...
            num2str(avgFitnessHist(1, gen + 1), '%3.3f') '   maxFitness=' ...
            num2str(maxFitnessHist(1, gen + 1), '%3.3f')]);
    end

    if 1%visualizationFlag
        [rs, x1, x2, r2] = thetaToParameters(dec2bin(eliteIndiv, genotypeSize));
        figure(1);
        hold off;
        plot(eval_s, catalogCurrent(eval_s), 'LineWidth', 2);
        hold on;
        yp = current(eval_s, rs, x1, x2, r2);
        plot(eval_s, yp, '-r*', 'MarkerIndices',1:10:length(yp));
        %hold off;
        %yyaxis right;
        %plot(t,fp, '-rs', 'LineWidth', 0.5, 'MarkerIndices',1:10:length(fp));
        ylabel('Yaw Angle (º)');
        grid on;
        %yyaxis left;
        %legend('x_{p}(t)','y_{p}(t)','\phi_{p}(t)');
        title(['Generation = ' num2str(gen) ', Average Fitness = ' sprintf('%0.3f', avgFitnessHist(1,gen+1))]);
        %ylabel('Position (m)');
        xlabel('s');
        drawnow;
    end    
end


%% Functions

function [offspringA, offspringB] = crossover(parentA, parentB, probCrossover)
    offspringA = parentA;
    offspringB = parentB;

    genN = size(parentA, 2);
    p = rand(1);

    if p <= probCrossover
        n = randi(genN);
        offspringA = [parentA(:, 1:n), parentB(:, n+1:genN)];
        offspringB = [parentB(:, 1:n), parentA(:, n+1:genN)];
    end
end

function [newOffspringA, newOffspringB] = mutate(offspringA, offspringB, probMutation)
    newOffspringA = offspringA;
    newOffspringB = offspringB;

    len = size(offspringA, 2);
    n = size(offspringA, 1);

    for i = 1:len
        for j = 1:n
            pA = rand(1);
            pB = rand(1);
            if pA <= probMutation
                newOffspringA(j, i) = randi([0, 1]);
            end
            
            if pB <= probMutation
                newOffspringB(j, i) = randi([0, 1]);
            end
        end
    end
end

function [rs, x1, x2, r2] = thetaToParameters(theta)
    rs = bitsArrayToNum(theta(1:11));
    x1 = bitsArrayToNum(theta(12:22));
    x2 = bitsArrayToNum(theta(23:33));
    r2 = bitsArrayToNum(theta(34:44));
end

function I = current(s, rs, x1, x2, r2)
    global i_n
    Vs = 220/sqrt(3);
    
    if sum(size(s)) == 2
        A = 1 + (r2/(s*x2))^2;
        I = Vs*(1 + (r2/(s*x2))^2)/sqrt((rs*A + r2/s)^2 + (x1*A + (1 - A)*x2)^2);
        I = I /i_n;
    else
        I = zeros(size(s));
        for i = 1:length(s)
            I(i) = current(s(i), rs, x1, x2, r2);
        end
    end
end

function P = power(s, rs, x1, x2, r2)
    global s_n;
    Vs = 220/sqrt(3);
    if sum(size(s)) == 2
        A = 1 + (r2/(s*x2))^2;
        P = Vs^2*(1 + (r2/(s*x2))^2)*(rs*A + r2/s)/((rs*A + r2/s)^2 + (x1*A + (1 - A)*x2)^2);
        P = P/s_n;
    else
        P = zeros(size(s));
        for i = 1:length(s)
            P(i) = power(s(i), rs, x1, x2, r2);
        end
    end
end

function f = evalFitness(pop)
    global eval_s
    
    f = 1 ./costFunction(eval_s, pop);
end

function J = costFunction(eval_s, pop)
    global genotypeSize
    J = zeros(1, length(pop));

    for i = 1:length(pop)
        [rs, x1, x2, r2] = thetaToParameters(dec2bin(pop(i), genotypeSize));
        Ise = 0;
        Pse = 0;

        for j = 1:length(eval_s)
            s = eval_s(j);
            P = power(s, rs, x1, x2, r2);
            I = current(s, rs, x1, x2, r2);

            Ise = Ise + (catalogCurrent(s) - I)^2;
            Pse = Pse + (catalogPower(s) - P)^2;
        end

        J(i) = Ise + Pse;
    end
end

function I = catalogCurrent(s)
    global Ixs
    I = interp1(Ixs(:, 1), Ixs(:, 2), s, 'spline');
end

function P = catalogPower(s)
    global Pxs
    P = interp1(Pxs(:, 1), Pxs(:, 2), s, 'spline');
end

function num = bitsArrayToNum(bits)
    global PARAM_MAX_VALUE
    num = PARAM_MAX_VALUE * (bin2dec(bits)/2^11);
end
