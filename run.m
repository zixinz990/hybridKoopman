function run(filename)
addpath("./functions");
addpath("./data");
load(filename);
obs_fun_list = obs_fun_list;
data = data;
data_next = data_next;
data_DT = data_DT;
rbf_center_list = rbf_center_list;
num_rbf = num_rbf;
eps = eps;
obs_fun_list = obs_fun_list;

% Estimate R matrix
R = zeros(num_obs, num_obs);
for i = 1:num_obs
    parfor j = i:num_obs
        fprintf('Estimating R matrix: i = %d, j = %d\n', i, j);
        result = est_inner_product(obs_fun_list{i}, obs_fun_list{j}, data, data_DT);
        R(i, j) = result;
    end
end
R = R + R' - tril(R);

% Estimate Q matrix
Q = zeros(num_obs, num_obs);
for i = 1:num_obs
    parfor j = 1:num_obs
        if j == 5
            disp("")
        end
        fprintf('Estimating Q matrix: i = %d, j = %d\n', i, j);
        result = est_inner_product(obs_fun_list{i}, obs_fun_list{j}, data_next, data_DT);
        Q(i, j) = result;
    end
end

% Get A matrix
A = Q * inv(R);

% save
time_arr = string(clock);
under_line_arr = repmat("_", size(time_arr));
time_arr = [time_arr; under_line_arr];
time_arr = time_arr(:)';
time_arr = time_arr(1:end-2);
file_name = "./data/" + strjoin(time_arr, '') + "bouncing_ball_2_dim_KDE.mat";
save(file_name, "R", "Q", "A", "rbf_center_list", "num_rbf", "eps", "obs_fun_list");
end
