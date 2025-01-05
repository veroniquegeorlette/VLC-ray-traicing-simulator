import numpy as np
from figures import *
from simulator import *
from utils import *
from rays_math import *


rays = generate_points_normal(50)

z_axis = np.array([0, 0, 1])
bottom_vec = np.array([0, 0, -1])
diag_vec = np.array([0, 1, -1])

bottom_align = align_rays(rays, z_axis, bottom_vec)
diag_align = align_rays(rays, z_axis, diag_vec)

plot_rays(rays, (0, 0, 0))
plot_rays(bottom_align, (0, 0, 0))
plot_rays(diag_align, (0, 0, 0))



#%%
n = 10000
plot_random_vectors(3, n)

#%%

ray_directions = generate_points_normal(20)
ray_directions = align_rays(ray_directions, np.array([0, 0, 1]), np.array([0, 0, -1]))

ray_origins = np.array([[-0.5,-0.5,1]]*20)

triangles = load_triangles_from_file("corner_room")
vertices, normals, _ = get_triangles_values(triangles)

closest_tri_indices, closest_distances = find_first_collision(ray_origins, ray_directions, vertices)

plot_triangles(triangles)
plot_collisions(ray_origins, ray_directions, triangles, closest_tri_indices, closest_distances)


#%%
ray_directions = generate_points_normal(20)
ray_directions = align_rays(ray_directions, np.array([0, 0, 1]), np.array([0, 0, -1]))

ray_origins = np.array([[0,0,1]]*20)

triangles = load_triangles_from_file("octo_empty")


vertices, normals, _ = get_triangles_values(triangles)


closest_tri_indices, closest_distances = find_first_collision(ray_origins, ray_directions, vertices)

plot_triangles(triangles)
plot_collisions(ray_origins, ray_directions, triangles, closest_tri_indices, closest_distances)





#%%
import numpy as np
from figures import *
from simulator import *
from utils import *
from rays_math import *
if __name__ == "__main__":
    # Emitter
    h = 2.15
    theta = 60
    power = 200
    
    # Receiver
    angle = 60
    area = 0.016
    index = 1.5
    
    rho = 0.8
    
    triangles = load_triangles_from_file("Inverted_example_room", rho)
    #plot_triangles(triangles)
    
    emitter_direction = np.array([0, 0, -1])
    emitter_direction = normalize(emitter_direction)
    
    emitter = Emitter(np.array([0, 0, h]), emitter_direction, power, theta)
    receiver = Receiver(0, angle, area, index, (-2.5, 2.5, -2.5, 2.5))
        
    
    simulation = VLCSimulator([emitter], receiver, triangles)
    
    X, Y, mean_power, nlos_mean_power = simulation.do_simulation(10000, 100, 100, 20)
    #print(np.min(mean_power), np.max(mean_power))
    X = X[1:-1, 1:-1]
    Y = Y[1:-1, 1:-1]
    mean_power = mean_power[1:-1, 1:-1]
    nlos_mean_power = nlos_mean_power[1:-1, 1:-1]
    plot_surface(X, Y, mean_power,'Mean power in mW for LoS')
    plot_surface(X, Y, nlos_mean_power, 'Mean power in mW for nLoS')
    
#%%
    plot_surface(X, Y, 10 * np.log10(mean_power+0.0001), 'Mean power in dBm for LoS')
#%%
    plot_surface(X, Y, mean_power + nlos_mean_power,'Sum of both LoS and nLoS in mW')
    plot_surface(X, Y, 10 * np.log10(nlos_mean_power+0.0001), 'Mean power in dBm for nLoS')
    plot_surface(X, Y, 10 * np.log10(mean_power + nlos_mean_power+0.0001),'Sum of both LoS and nLoS in dBm')
 

#%%

import numpy as np
from figures import *
from simulator import *
from utils import *
from rays_math import *

# Emitter
h = 2.15
theta = 12.5
power = 200

# Receiver
angle = 70
area = 1e-4
index = 1.5

emitter1 = Emitter(np.array([-1.25, -1.25, h]), np.array([0, 0, -1]), power, theta)
emitter2 = Emitter(np.array([-1.25, 1.25, h]), np.array([0, 0, -1]), power, theta)
emitter3 = Emitter(np.array([1.25, -1.25, h]), np.array([0, 0, -1]), power, theta)
emitter4 = Emitter(np.array([1.25, 1.25, h]), np.array([0, 0, -1]), power, theta)

receiver = Receiver(0.00001, angle, area, index, (-2.5, 2.5, -2.5, 2.5))
    
triangles = load_triangles_from_file("example_room")

simulation = VLCSimulator([emitter1, emitter2, emitter3, emitter4], receiver, triangles)

X, Y, mean_power, _ = simulation.do_simulation(100000, 10, 0, 25)
print(np.min(mean_power), np.max(mean_power))
plot_surface(X, Y, mean_power,'Mean power in dBm for nLoS, 4 emitters')
plot_surface(X, Y, 10 * np.log10(mean_power),'Mean power in dBm for nLoS, 4 emitters')

#%%
# Emitter
h = 0.9
theta = 60
power = 200

# Receiver
angle = 90
area = 0.016
index = 1.5

emitter_direction = np.array([0, 0, -1])
emitter_direction = normalize(emitter_direction)

emitter = Emitter(np.array([-0.5, -0.5, h]), emitter_direction, power, theta)
receiver = Receiver(0, angle, area, index, (-1, 1, -1, 1))
triangles = load_triangles_from_file("corner_room")

simulation = VLCSimulator([emitter], receiver, triangles)

X, Y, mean_power,nlos_mean_power = simulation.do_simulation(10000, 100, 10, 50)
print(np.min(mean_power), np.max(mean_power))
plot_surface(X, Y, mean_power, 'Mean power in mW for Los in a corner room')
plot_surface(X, Y, 10 * np.log10(mean_power+0.0001),'Mean power in dBm for Los in a corner room')
plot_surface(X, Y, nlos_mean_power, 'Mean power in mW for nLoS in a corner room')
plot_surface(X, Y, 10 * np.log10(nlos_mean_power+0.0001),'Mean power in dBm for nLoS in a corner room')
plot_surface(X, Y, mean_power+nlos_mean_power, 'Mean power in mW for nLoS in a corner room')
plot_surface(X, Y, 10 * np.log10(mean_power+nlos_mean_power+0.0001),'Mean power in dBm for nLoS in a corner room')