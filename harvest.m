addpath Classes
pathname = "./Outputs8/";
clear files
files_lame = dir(pathname);
v_dists_lame = zeros([1, length(files_lame)-2]);
names = [];
for iterate_lame = 3:length(files_lame)
    
    load(pathname + files_lame(iterate_lame).name);
    disp(files_lame(iterate_lame).name)
    v_dists_lame(iterate_lame - 2) = n1.location(2) - n11.location(2);
end
disp(v_dists_lame)

