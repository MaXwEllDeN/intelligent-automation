% PID Tuning using optimization

maxGen = 100;
A = eye(4);
B = eye(4);
C = eye(4);
D = eye(4);

F = [-0.9999,   -394.8590,  394.8590;
     0,         -1,         0;
     0,          0,         -1
    ];

G = [0, 394.8590, 394.8590]';

H = [1, 0, 0];

% Baseline start: 
for gen = 1:maxGen
    
end

function result = fitnessFunction(e)
    % result = int(e'edt) from 0 to +inf
    result = NaN;
end