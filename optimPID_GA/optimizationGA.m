close all;
%
PERFORMANCE_CALC = 'itae';
%

global dt
global iv
global t_span
global params

dt = 0.0357;
iv = [0; 0; pi/4; eps; eps];

% Adding a tilt on the yaw angle
iv(3) = pi;

t_span = 0:dt:12;
params = [2, 2, 0.25, 50]; 

popSize = 60;
maxGens = 300;
tolerance = 1e-4;

paramSize = 4;
genotypeSize = 3*paramSize;
probCrossover = 1;          % The probability of crossing over. 
probMutation = 1;           % The mutation probability (per bit)

verboseFlag = 1;            % 1 => display details of each generation
                            % 0 => run quietly
visualizationFlag = 1;


%%
% generation
avgFitnessHist = zeros(1, maxGens + 1);
maxFitnessHist = zeros(1, maxGens + 1);

pop = rand(popSize, 1);

eliteIndiv = [];
eliteFitness = -realmax;

for gen=0:maxGens

    fitnessVals = zeros(popSize, 1);
    for i=1:popSize
        fitnessVals(i) = iecost(decToQ14(pop(i), genotypeSize), PERFORMANCE_CALC);
    end

    [maxFitnessHist(1, gen + 1), maxIndex] = max(fitnessVals);    
    avgFitnessHist(1, gen + 1) = mean(fitnessVals);

    if eliteFitness < maxFitnessHist(gen + 1)
        eliteFitness = maxFitnessHist(gen + 1);
        eliteIndiv = pop(maxIndex, :);
    end

    if isinf(maxFitnessHist(gen+1))
        break;
    end

    roulette_p = fitnessVals / sum(fitnessVals);
    parentIndices = randsample([1:popSize], popSize, true, roulette_p);

    % deterimine the first parents of each mating pair
    firstParents = pop(parentIndices(1:popSize/2),:);
    firstParents = decToQ14(firstParents, genotypeSize);

    % determine the second parents of each mating pair
    secondParents = pop(parentIndices(popSize/2+1:end),:);
    secondParents = decToQ14(secondParents, genotypeSize);
    [offspringA, offspringB] = crossover(firstParents, secondParents, probCrossover);
    [offspringA, offspringB] = mutate(offspringA, offspringB, probMutation);

    pop = [q14ToDec(offspringA); q14ToDec(offspringB)];

    eliteChromosome = decToQ14(eliteIndiv, genotypeSize);

    if verboseFlag
        fprintf('gen= %.3d,\t avgFitness=%3.3f,\t maxFitness=%3.3f,\t eliteChromosome=%s\n',...
            gen, avgFitnessHist(1, gen + 1), maxFitnessHist(1, gen + 1),...
            eliteChromosome);
    end    
    
    if visualizationFlag
        %% Plotting results        
        [kp, ki, kd] = chromosomeToParams(eliteChromosome);
        [t, x] = ode45(@(t, x)dynamicModelPID(t, x, kp, ki, kd, params), t_span, iv, odeset('RelTol',1e-6));

        % control error
        phi_d = atan2(params(2) - x(:, 2), params(1) - x(:, 1));
        e = phi_d - x(:, 3);
        e = atan2(sin(e),cos(e));        

        xp = x(:, 1);
        yp = x(:, 2);
        fp = x(:, 3) * 180/pi;
        figure(1);
        hold off;
        plot(t, xp, 'LineWidth', 2);
        hold on;
        plot(t, yp, '-b*', 'MarkerIndices',1:10:length(yp));
        hold off;
        yyaxis right;
        plot(t,fp, '-rs', 'LineWidth', 0.5, 'MarkerIndices',1:10:length(fp));
        ylabel('Yaw Angle (º)');
        grid on;
        yyaxis left;
        legend('x_{p}(t)','y_{p}(t)','\phi_{p}(t)');
        title(['Generation = ' num2str(gen) ', Average Fitness = ' sprintf('%0.3f', avgFitnessHist(1,gen+1))]);
        ylabel('Position (m)');
        xlabel('t [s]');
        drawnow;
    end

     
    if iecost(eliteChromosome, PERFORMANCE_CALC) <= tolerance
        break;
    end

end

[kp, ki, kd] = chromosomeToParams(eliteChromosome);

fprintf("Kp=%.3f, Ki=%.3f, Kd=%.3f\n", kp, ki, kd);

%% Functions

function [offspringA, offspringB]=crossover(parentA, parentB, probCrossover)
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

function [newOffspringA, newOffspringB]=mutate(offspringA, offspringB, probMutation)
    newOffspringA = offspringA;
    newOffspringB = offspringB;

    pool = char("0123456789ABCDEF");

    len = size(offspringA, 2);
    n = size(offspringA, 1);

    for i = 1:len
        for j = 1:n
            pA = rand(1);
            pB = rand(1);
            if pA <= probMutation
                newOffspringA(j, i) = randsample(pool, 1);
            end
            
            if pB <= probMutation
                newOffspringB(j, i) = randsample(pool, 1);
            end
        end
    end
end

function J=iecost(individual, idx)
    global iv
    global dt
    global t_span
    global params

    [kp, ki, kd] = chromosomeToParams(individual);
    [t, x] = ode45(@(t, x)dynamicModelPID(t, x, kp, ki, kd, params), t_span, iv, odeset('RelTol',1e-6));

    % control error
    phi_d = atan2(params(2) - x(:, 2), params(1) - x(:, 1));
    e = phi_d - x(:, 3);
    e = atan2(sin(e),cos(e));
   
    % performance calculation
    switch idx
        case 'ise'  % ISE
            J=trapz(t, e.^2);            
        case 'iae'  % IAE
            J=trapz(t, abs(e));
        case 'itse'  % ITSE
            J=trapz(t, t.*(e.^2));
        case 'itae'  % ITAE
            J=trapz(t, t.*abs(e));
    end

    J = 1/J;
end

function [kp, ki, kd] = chromosomeToParams(chromossome)
    kp = q14ToDec(extractBetween(chromossome, 1, 4));
    ki = q14ToDec(extractBetween(chromossome, 5, 8));
    kd = q14ToDec(extractBetween(chromossome, 9, 12));
end
