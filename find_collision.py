import bpy
import math
from mathutils import Vector

class Agent:
    def __init__(self, location, velocity):
        self.location = Vector(location)
        self.velocity = Vector(velocity)

def calculate_collision_time(agent, collider, width):
    relative_position = collider.location - agent.location
    relative_velocity = collider.velocity - agent.velocity

    a = relative_velocity.length_squared
    b = 2 * relative_position.dot(relative_velocity)
    c = relative_position.length_squared - width**2

    if a == 0: # agents move at same velocity
        if relative_position.length <= width:
            return 0 # currently crashed
        return None

    discriminant = b**2 - 4 * a * c 
    if discriminant < 0:
        return None

    sqrt_discriminant = math.sqrt(discriminant)
    time_to_collision_1 = (-b - sqrt_discriminant) / (2 * a)
    time_to_collision_2 = (-b + sqrt_discriminant) / (2 * a)

    times = []
    for t in (time_to_collision_1, time_to_collision_2):
        if t > 0:
            times.append(t)

    if times:
        gugus = min(times)
        collision_point = Vector(agent.location) + (gugus * agent.velocity)
        print(f"Collision point: {collision_point}")
        return min(times)
    else:
        return None

def calculate_minimum_distance(agent, collider):
    relative_position = collider.location - agent.location
    relative_velocity = collider.velocity - agent.velocity

    a = relative_velocity.length_squared
    b = 2 * relative_position.dot(relative_velocity)
    
    # Find the time of closest approach
    if a > 0:
        t_min_dist = -b / (2 * a)
        if t_min_dist > 0:
            closest_position_agent = agent.location + t_min_dist * agent.velocity
            closest_position_collider = collider.location + t_min_dist * collider.velocity
            min_distance = (closest_position_agent - closest_position_collider).length
            return t_min_dist, min_distance
    return None, relative_position.length  # If agents are static or always diverging

### SCRIPT ###

agent = Agent((0, 0), (1, 0.5))
collider = Agent((6, 0), (-2, 2))
width = 1.00

time_to_collision = calculate_collision_time(agent, collider, width)

if time_to_collision:
    print(f"The agents will collide in {time_to_collision:.2f} seconds.")
else:
    print("The agents will not collide.")
    t_min_dist, min_distance = calculate_minimum_distance(agent, collider)
    if t_min_dist:
        print(f"Closest approach in {t_min_dist:.2f} seconds with a minimum distance of {min_distance:.2f} units.")
    else:
        print("The agents are diverging or static relative to each other.")
