% SpeedyGA is a vectorized implementation of a Simple Genetic Algorithm in Matlab
% Version 1.3
% Copyright (C) 2007, 2008, 2009  Keki Burjorjee
% Created and tested under Matlab 7 (R14). 

%  Licensed under the Apache License, Version 2.0 (the "License"); you may
%  not use this file except in compliance with the License. You may obtain 
%  a copy of the License at  
%
%  http://www.apache.org/licenses/LICENSE-2.0 
%
%  Unless required by applicable law or agreed to in writing, software 
%  distributed under the License is distributed on an "AS IS" BASIS, 
%  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
%  See the License for the specific language governing permissions and 
%  limitations under the License. 

%  Acknowledgement of the author (Keki Burjorjee) is requested, but not required, 
%  in any publication that presents results obtained by using this script 

%  Without Sigma Scaling, Stochastic Universal Sampling, and the generation of mask 
%  repositories, SpeedyGA faithfully implements the specification of a simple genetic 
%  algorithm given on pages 10,11 of M. Mitchell's book An Introduction to
%  Genetic Algorithms, MIT Press, 1996). Selection is fitness
%  proportionate.

close all;
clear all;

global len popSize In Cn

len=44;                    % The length of the genomes  
popSize=400;               % The size of the population (must be an even number)
maxGens=400;              % The maximum number of generations allowed in a run
probCrossover=1;           % The probability of crossing over. 
probMutation=0.01;        % The mutation probability (per bit)
sigmaScalingFlag=1;        % Sigma Scaling is described on pg 168 of M. Mitchell's
                           % GA book. It often improves GA performance.
sigmaScalingCoeff=1;       % Higher values => less fitness pressure 

SUSFlag=1;                 % 1 => Use Stochastic Universal Sampling (pg 168 of 
                           %      M. Mitchell's GA book)
                           % 0 => Do not use Stochastic Universal Sampling
                           %      Stochastic Universal Sampling almost always
                           %      improves performance

crossoverType=1;           % 0 => no crossover
                           % 1 => 1pt crossover
                           % 2 => uniform crossover

visualizationFlag=0;       % 0 => don't visualize bit frequencies
                           % 1 => visualize bit frequencies

verboseFlag=1;             % 1 => display details of each generation
                           % 0 => run quietly

useMaskRepositoriesFlag=1; % 1 => draw uniform crossover and mutation masks from 
                           %      a pregenerated repository of randomly generated bits. 
                           %      Significantly improves the speed of the code with
                           %      no apparent changes in the behavior of
                           %      the SGA
                           % 0 => generate uniform crossover and mutation
                           %      masks on the fly. Slower. 
                           

%%
global Vs In Cn

Vs = 220/sqrt(3);
Cn = 4.30;
In = 3.46;
poles = 4;

T = readmatrix('Tptg_s.txt');
I = readmatrix('Iptg_s.txt');
ns = 120*60/poles;

nI =  I(:,2)*120*60/poles;
I(:,2) = (ns-nI)/ns;

nT =  T(:,2)*120*60/poles;
T(:,1) = T(:,1).*T(:,2)*In;  %.*nT/ns;%*2*pi/(60*ns);
T(:,2) = (ns-nT)/ns;

Tfit = fit(T(:,2),T(:,1),'poly5');
Ifit = fit(I(:,2),I(:,1),'poly5');

coeffT = coeffvalues(Tfit);
Pds = @(s) coeffT(1).*s.^5+coeffT(2).*s.^4+ ...
           coeffT(3).*s.^3+coeffT(4).*s.^2+...
           coeffT(5).*s+coeffT(6);
coeffI = coeffvalues(Ifit);
Ids = @(s) coeffI(1).*s.^5+coeffI(2).*s.^4+ ...
           coeffI(3).*s.^3+coeffI(4).*s.^2+...
           coeffI(5).*s+coeffI(6);
%%
% crossover masks to use if crossoverType==0.
mutationOnlycrossmasks=false(popSize,len);

% pre-generate two �repositories� of random binary digits from which the  
% the masks used in mutation and uniform crossover will be picked. 
% maskReposFactor determines the size of these repositories.

maskReposFactor=15;
uniformCrossmaskRepos=rand(popSize/2,(len+1)*maskReposFactor)<0.5;
mutmaskRepos=rand(popSize,(len+1)*maskReposFactor)<probMutation;

% preallocate vectors for recording the average and maximum fitness in each
% generation
avgFitnessHist=zeros(1,maxGens+1);
maxFitnessHist=zeros(1,maxGens+1);

    
eliteIndiv=[];
eliteFitness=-realmax;


% the population is a popSize by len matrix of randomly generated boolean
% values
pop=rand(popSize,len)<.5;

for gen=0:maxGens

    % evaluate the fitness of the population. The vector of fitness values 
    % returned  must be of dimensions 1 x popSize.
    fitnessVals=fitting(pop,Ids,Pds);
     
    [maxFitnessHist(1,gen+1),maxIndex]=max(fitnessVals);    
    avgFitnessHist(1,gen+1)=mean(fitnessVals);
    if eliteFitness<maxFitnessHist(gen+1)
        eliteFitness=maxFitnessHist(gen+1);
        eliteIndiv=pop(maxIndex,:);
    end    
    
    % display the generation number, the average Fitness of the population,
    % and the maximum fitness of any individual in the population    
    [rs,x1,x2,r2] = binArray2dec(eliteIndiv);
    if verboseFlag
        display(['gen=' num2str(gen,'%.3d') '   rs=' ...
            num2str(rs,'%3.3f') '   x1=' ...
            num2str(x1,'%3.3f') '   x2=' ...
            num2str(x2,'%3.3f') '   rs=' ...
            num2str(r2,'%3.3f')]);        
    end
    plotGen = figure(3);
    sN = 0.01:0.01:1-0.01;
    A =  1+(r2./sN/x2).^2;
    I =  (A)./sqrt((rs*A+r2./sN).^2+(x1.*A+(1-A).*x2).^2);
    P =  (A).*(rs*A+r2./sN)./((rs*A+r2./sN).^2+(x1*A+(1-A)*x2).^2);  
    subplot(2,1,1)
    plot(sN,Ids(sN)); hold on;
    plot(sN,I); hold off;
    ylim([0 10]);
    legend("Datasheet WEG", "Genetic algorithm",...
            "Location","Southeast");
    title(['Generation = ' num2str(gen) ', Average Fitness = ' sprintf('%0.3f', avgFitnessHist(1,gen+1))]);
    xlabel('s')
    ylabel('I [A]');
    grid;
    subplot(2,1,2)
    plot(sN,Pds(sN)); hold on;
    plot(sN,P); hold off;
    ylim([0 5]);
    xlabel('s')
    ylabel('P [W]');
    legend("Datasheet WEG", "Genetic algorithm",...
            "Location","Southeast");
    grid;
    % Conditionally perform bit-frequency visualization
    if visualizationFlag
        figure(1)
        set (gcf, 'color', 'w');
        hold off
        bitFreqs=sum(pop)/popSize;
        plot(1:len,bitFreqs, '.');
        axis([0 len 0 1]);
        title(['Generation = ' num2str(gen) ', Average Fitness = ' sprintf('%0.3f', avgFitnessHist(1,gen+1))]);
        ylabel('Frequency of the Bit 1');
        xlabel('Locus');
        drawnow;
    end    
    
    if (any(isinf(fitnessVals)))
        break;
    end
    
    % Conditionally perform sigma scaling 
    if sigmaScalingFlag
        sigma=std(fitnessVals);
        if sigma~=0
            fitnessVals=1+(fitnessVals-mean(fitnessVals))/...
            (sigmaScalingCoeff*sigma);
            fitnessVals(fitnessVals<=0)=0;
        else
            fitnessVals=ones(popSize,1);
        end
    end        
    
    
    % Normalize the fitness values and then create an array with the 
    % cumulative normalized fitness values (the last value in this array
    % will be 1)
    cumNormFitnessVals=cumsum(fitnessVals/sum(fitnessVals))';

    % Use fitness proportional selection with Stochastic Universal or Roulette
    % Wheel Sampling to determine the indices of the parents 
    % of all crossover operations
    if SUSFlag
        markers=rand(1,1)+[1:popSize]/popSize;
        markers(markers>1)=markers(markers>1)-1;
    else
        markers=rand(1,popSize);
    end
    [temp parentIndices]=histc(markers,[0 cumNormFitnessVals]);
    parentIndices=parentIndices(randperm(popSize));    

    % deterimine the first parents of each mating pair
    firstParents=pop(parentIndices(1:popSize/2),:);
    % determine the second parents of each mating pair
    secondParents=pop(parentIndices(popSize/2+1:end),:);
    
    % create crossover masks
    if crossoverType==0
        masks=mutationOnlycrossmasks;
    elseif crossoverType==1
        masks=false(popSize/2, len);
        temp=ceil(rand(popSize/2,1)*(len-1));
        for i=1:popSize/2
            masks(i,1:temp(i))=true;
        end
    else
        if useMaskRepositoriesFlag
            temp=floor(rand*len*(maskReposFactor-1));
            masks=uniformCrossmaskRepos(:,temp+1:temp+len);
        else
            masks=rand(popSize/2, len)<.5;
        end
    end
    
    % determine which parent pairs to leave uncrossed
    reprodIndices=rand(popSize/2,1)<1-probCrossover;
    masks(reprodIndices,:)=false;
    
    % implement crossover
    firstKids=firstParents;
    firstKids(masks)=secondParents(masks);
    secondKids=secondParents;
    secondKids(masks)=firstParents(masks);
    pop=[firstKids; secondKids];
    
    % implement mutation
    if useMaskRepositoriesFlag
        temp=floor(rand*len*(maskReposFactor-1));
        masks=mutmaskRepos(:,temp+1:temp+len);
    else
        masks=rand(popSize, len)<probMutation;
    end
    pop=xor(pop,masks);    
end
if verboseFlag
    figure(2)
    %set(gcf,'Color','w');
    hold off
    plot([0:maxGens],avgFitnessHist,'k-');
    hold 
    plot([0:maxGens],maxFitnessHist,'c-');
    title('Maximum and Average Fitness')
    xlabel('Generation')
    ylabel('Fitness')
end

%%
[rs,x1,x2,r2] = binArray2dec(eliteIndiv);
fprintf("eliteIndiv: rs = %.3f x1 = %.3f x2 = %.3f r2 = %.3f\n",...
        rs,x1,x2,r2);
[~,~,~] = mkdir('data');
save('both.mat','eliteIndiv');

[~,~,~] = mkdir('pics');
[~,~,~] = mkdir('pics/Method-01');
saveas(gca,"pics/Method-01/"+"both"+".png")
%%
function fitness=fitting(pop,Ids,Pds)
    global popSize 
    [rs,x1,x2,r2]=binArray2dec(pop);
    cost = zeros(popSize,1);
    sN = 0.2:0.01:0.8-0.01;
    for i=1:popSize
        A =  1+(r2(i)./sN/x2(i)).^2;
        I =  (A)./sqrt((rs(i)*A+r2(i)./sN).^2+(x1(i).*A+(1-A)*x2(i)).^2);
        P =  (A).*(rs(i)*A+r2(i)./sN)./((rs(i)*A+r2(i)./sN).^2+(x1(i)*A+(1-A)*x2(i)).^2);   
        eI = Ids(sN) - I;
        eP = Pds(sN) - P;
        x = sum(eI.^2)/4^2+sum(eP.^2)/3.25^2;
        if (isnan(x))
            x=inf;
        end   
        cost(i)=x;
    end
    fitness=(1./cost); 
end

function [rs,x1,x2,r2]=binArray2dec(x)
    global len
    rs = hex2dec(binaryVectorToHex(x(:,1:len/4)))/(2^(len/4))+0.001;
    x1 = hex2dec(binaryVectorToHex(x(:,len/4+1:2*len/4)))/(2^(len/4))+0.001;
    x2 = hex2dec(binaryVectorToHex(x(:,2*len/4+1:3*len/4)))/(2^(len/4-2))+4;
    r2 = hex2dec(binaryVectorToHex(x(:,3*len/4+1:len)))/(2^(len/4))+0.001;
end

