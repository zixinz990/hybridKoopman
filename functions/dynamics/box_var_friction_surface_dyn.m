function state_next = box_var_friction_surface_dyn(state, input, options)
arguments
    state(2, 1) {mustBeNumeric} % state = [pos; vel]
    input(1, 1) {mustBeNumeric} % input = external force except friction
    options.mass(1, 1) {mustBeNumeric, mustBePositive}
    options.g(1, 1) {mustBeNumeric, mustBePositive} = 10
    options.mu(3, 1) {mustBeNumeric, mustBePositive}
    options.pos_switch(3, 1) {mustBeNumeric}
    options.dt(1, 1) {mustBeNumeric, mustBePositive} = 0.01
end
pos = state(1);
vel = state(2);

if pos < options.pos_switch(1)
    f = -sign(vel) * options.mu(1) * options.mass * options.g;
elseif pos >= options.pos_switch(1) && pos < options.pos_switch(2)
    f = -sign(vel) * options.mu(2) * options.mass * options.g;
else
    f = -sign(vel) * options.mu(3) * options.mass * options.g;
end

acc = (input + f) / options.mass;
vel_next = vel + acc * options.dt;
pos_next = pos + (vel + vel_next) * options.dt / 2;
state_next = [pos_next; vel_next];

end
