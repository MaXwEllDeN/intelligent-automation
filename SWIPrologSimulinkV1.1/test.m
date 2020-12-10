target_overshoot = 10;
target_overshoot_ratio = 0.25;
target_damping = 0.5;
target_period = 10;
target_rise_time = 2;


target_features = [target_overshoot, target_overshoot_ratio,...
                   target_damping, target_period, target_rise_time];
%
weights = [30 20 20 1 1];
%
Kc_signs = [0 1 1 1 1];
Ti_signs = [1 1 1 0 -1];
Td_signs = [1 1 1 -1 1];


%Kc = 15;
%Ti = 6.75;
%Td = 1.69;
%[cf1 cf2 cf3 cf4 cf5] = patternExtractor(Kc, Ti, Td);

