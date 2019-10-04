clear all
tests = 1:500;
fits = zeros(size(tests));
d=[0.125;0.125;0.125]*200;
load('C:\Users\jzerez\Desktop\Formula\Mk4\God Mode\Outputs8\rear-3-1.mat')
ag.curr_aca.endpoints(1).new_region(d);
ag.curr_aca.endpoints(2).new_region(d);
ag.curr_pca.endpoints(1).new_region(d);
ag.curr_pca.endpoints(2).new_region(d);

for test = tests
    ag.curr_aca.endpoints(1).shuffle()
    ag.curr_aca.endpoints(2).shuffle()
    ag.curr_pca.endpoints(1).shuffle()
    ag.curr_pca.endpoints(2).shuffle()
    [static_char, dyn_char] = ag.perform_sweep(7, 0);
    fits(test) = calc_fitness(static_char, dyn_char, desired_static_char, static_char_weights, desired_dyn_char, dyn_char_weights);
end
figure;
disp(std(fits))
plot(fits)
xlabel('Test Case')
ylabel('Fitness')
title('Fitness of Suspension, 0.125" tolerance')
