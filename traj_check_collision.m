function [collision] = traj_check_collision(traj, obs, offset)

n_sample = length(traj(:,1));
collision = false;

for i=1:2:n_sample
    if point_check_collision(traj(i,:), obs, offset)
        collision = true;
        return
    end
end

end