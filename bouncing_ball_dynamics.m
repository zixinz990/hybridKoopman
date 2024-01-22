function x_next = bouncing_ball_dynamics(x, u)
m = 1;
g = -9.81;
dt = 0.01;

h = x(1);
v = x(2);
a = (m * g + u) / m;
h_next = h + v * dt + 0.5 * a * dt^2;

if h_next < 0
    t_jump_1 = (-v - sqrt(v^2-2*a*h)) / a;
    t_jump_2 = (-v + sqrt(v^2-2*a*h)) / a;
    t_jump = max(t_jump_1, t_jump_2);
    v_jump = v + a * t_jump;
    v_next = -v_jump + a * (dt - t_jump);
    h_next = -v_jump * (dt - t_jump) + 0.5 * a * (dt - t_jump)^2;
else
    v_next = v + a * dt;
end
x_next = [h_next; v_next];
end
