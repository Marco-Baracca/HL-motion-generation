function [collision] = point_check_collision(p, obs, offset)

collision = false;
n_obs = length(obs(:,1));

for i=1:n_obs
    if norm(p-obs(i,1:3))<(obs(i,4)+offset)
        collision = true;
        return
    end
end

end