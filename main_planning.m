
close all
clear all
clc

load('fPC_dataset') %Loading of functional Principal Components

%% Initialization starting and final constrain

t_start = 0; %initial time in [s]
t_end = 5; %final time in [s]
starting_position = [-300 500 300]; %initial position [x y z] in [mm]
ending_position = [300 -500 -300]; %final position [x y z] in [mm]


%% Obstacle definition
% The obstacles are represented in a matrix Nx4 where each row [X Y Z R]
% represent a spherical obstacle with radius R and center [X Y Z] all
% expressed in [mm]
% If there are no obstacle, leaves 'obs' as an empty variable

% obs = [];
obs = [0 0 0 150];

offset = 50; %Minimum distance that the trajectory ha to keep from obstacles in [mm]

%% Trajectory computation

direct_trajectory = [];
t_trajectory = [];
viapoint = [];

for i=1:3
    [traiettoria, t] = traj_no_obs(dof{i}, starting_position(i), ending_position(i), t_end);
    direct_trajectory = [direct_trajectory traiettoria];
    t_trajectory = [t_trajectory t];
end

if size(obs,1)~=0
    if traj_check_collision(direct_trajectory, obs, offset)
        [complete_trajectory, t_trajectory, viapoint] = compute_traj(starting_position(1:3),t_start,ending_position(1:3),t_end,obs,offset,dof,zeros(1,3),zeros(1,3));
    else
        complete_trajectory = direct_trajectory;
    end
else
    complete_trajectory = direct_trajectory;
end


%% Plot

close all

figure
plot3(starting_position(1), starting_position(2), starting_position(3),'ok');
hold on
grid on
xlim([-600 600])
ylim([-600 600])
zlim([-600 600])
view(45,45)
plot3(ending_position(1), ending_position(2), ending_position(3), 'ok');
[X,Y,Z] = sphere;
   
for i=1:size(obs,1)
    obsi = surf(obs(i,4)*X+obs(i,1), obs(i,4)*Y+obs(i,2), obs(i,4)*Z+obs(i,3), [1 0 0]);
    set(obsi, 'facec', 'r')
    set(obsi, 'FaceAlpha',0.1)
    set(obsi, 'edgec', [200,0,0]./255)
    plot3(obs(i,1), obs(i,2), obs(i,3), 'ob');
end

if size(viapoint,1)~=0
    for j=1:size(viapoint,1)
        plot3(viapoint(j,1), viapoint(j,2), viapoint(j,3), 'ob');
    end
end

plot3(complete_trajectory(:,1), complete_trajectory(:,2), complete_trajectory(:,3), '-g');

figure
plot(t_trajectory(:,1), complete_trajectory(:,1))
hold on
plot(t_start,starting_position(1),'ko')
plot(t_end,ending_position(1),'ko')
if size(viapoint,1)~=0
    plot(viapoint(4),viapoint(1),'ro')
end
xlabel('t [s]')
ylabel('x [mm]')
grid on

figure
plot(t_trajectory(:,1), complete_trajectory(:,2))
hold on
plot(t_start,starting_position(2),'ko')
plot(t_end,ending_position(2),'ko')
if size(viapoint,1)~=0
    plot(viapoint(4),viapoint(2),'ro')
end
xlabel('t [s]')
ylabel('y [mm]')
grid on

figure
plot(t_trajectory(:,1), complete_trajectory(:,3))
hold on
plot(t_start,starting_position(3),'ko')
plot(t_end,ending_position(3),'ko')
if size(viapoint,1)~=0
    plot(viapoint(4),viapoint(3),'ro')
end
xlabel('t [s]')
ylabel('z [mm]')
grid on
