fs = 38.192e6;
f_if = 9.548e6;
f_pass_low  = f_if - 2.0e6;
f_pass_high = f_if + 2.0e6;
[b_saw, a_saw] = butter(4, [f_pass_low, f_pass_high] / (fs/2), 'bandpass');

fprintf('b: '); fprintf('%.16e, ', b_saw); fprintf('\n');
fprintf('a: '); fprintf('%.16e, ', a_saw); fprintf('\n');