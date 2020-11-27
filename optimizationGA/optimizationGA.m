close all;
popSize = 60;
maxGens = 500;
error = 1e-3;

genotypeMin = 4;
genotypeSize = 4*3;           % 
probCrossover = 1;          % The probability of crossing over. 
probMutation = 1;           % The mutation probability (per bit)

verboseFlag = 1;            % 1 => display details of each generation
                            % 0 => run quietly
%
a = 2.0;
b = 0.1;
c = 1.0;
f = @(x) a*exp(-b*x).*sin(c*x);
%

% generation
avgFitnessHist = zeros(1, maxGens + 1);
maxFitnessHist = zeros(1, maxGens + 1);

pop = rand(popSize, 1);

eliteIndiv = [];
eliteFitness = -realmax;

for gen=0:maxGens
    fitnessVals = evalFitness(f(pop));
    
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
    secondParents=pop(parentIndices(popSize/2+1:end),:);
    secondParents = decToQ14(secondParents, genotypeSize);
    [offspringA, offspringB] = crossover(firstParents, secondParents, probCrossover);
    [offspringA, offspringB] = mutate(offspringA, offspringB, probMutation);

    pop = [q14ToDec(offspringA); q14ToDec(offspringB)];
    % display the generation number, the average Fitness of the population,
    % and the maximum fitness of any individual in the population
    if verboseFlag
        display(['gen=' num2str(gen, '%.3d') '   avgFitness=' ...
            num2str(avgFitnessHist(1, gen + 1), '%3.3f') '   maxFitness=' ...
            num2str(maxFitnessHist(1, gen + 1), '%3.3f')]);
    end
    
    if abs(f(eliteIndiv)) <= error
        break;
    end

end

x1 = eliteIndiv;

h = 1e-3;
x = (-3*pi:h:3*pi);
y = f(x);

figure(1);
plot(x,y), hold on;
xlabel('x');
grid;
title(['x_{*}=' num2str(x1) ' e f(x_{*})=' num2str(f(x1))]);
plot(x1, f(x1), 'ro');
legend('f(x)', 'x_{*}');
%
   
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

function val = evalFitness(f)
    val = abs(f);
    % In order to find minimum value:
    val = 1 ./ val;
end