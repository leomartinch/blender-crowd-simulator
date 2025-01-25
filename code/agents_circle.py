import bpy
import math
import mathutils
from mathutils import Vector
import time

import gpu
from gpu_extras.batch import batch_for_shader

from collections import namedtuple


"""IN PROGRESS"""

"""
To-Do:
- move characters to grid
- get list of neighbors

"""





### CLASS ###
"""agent class: create agent object with attributes that define the characteristics of the agent in the simulation"""
class Agent:
    def __init__(self, name, direction, location, velocity, target_list, mass, leg_length):
        self.name = name
        self.direction = Vector(direction)
        self.prev_direction = Vector(direction)
        self.location = Vector(location)
        self.prev_location = Vector(location)
        self.velocity = velocity
        self.prev_velocity = velocity

        self.target = target_list[-1]
        target_loc = bpy.data.objects.get(target_list[-1]).location
        self.target_location = (target_loc.x, target_loc.y) # not sure if it is smart to have the target location be part of the agent, multiple agents can have the same moving target
        self.target_stack = target_list

        self.nearest_neighbor = None
        self.min_length_to_neighbor = (Vector(location) - Vector((target_loc.x, target_loc.y))).length
        self.nearest_neighbor_location = target_loc
        
        self.mass = mass
        self.leg_length = leg_length

        self.is_panic = False

        #assign_to_grid(self)

        agents_directions[name] = direction
        agents_previous_directions[name] = direction
        
        agents_coordinates[name] = location
        agents_previous_coordinates[name] = location

        agents_target_stack[name] = target_list
        all_neighbors.append(self)


    def update_frame_data(self, new_direction, new_location, new_velocity):
        self.prev_direction = self.direction
        self.prev_location = self.location
        self.prev_velocity = self.velocity

        self.direction = new_direction
        self.location = new_location
        self.velocity = new_velocity

    def update_target_stack(self, target_list):
        self.target_stack.append(target_list)

    def update_nearest_neighbor(self, neighbor):
        self.nearest_neighbor = neighbor
        self.nearest_neighbor_location = (Vector(self.location) - Vector(neighbor.location)).length



    

### FUNCTIONS ###

# ignore grid for moment, have to find a better way to check for grid etc

def assign_to_grid(agent):
    x_cell_value = int(agent.location.x // cell_size)
    y_cell_value = int(agent.location.y // cell_size)
    cell_key = (x_cell_value, y_cell_value)
    if cell_key not in grid:
        grid[cell_key] = []
    grid[cell_key].append(agent)

    return cell_key

"""return all objects (neighbors) that are in the field of view of the agent"""
def objects_in_field_of_view(agent): # mit skalarprodukt (dot product)
    visible_neighbors = []
    #visible_enemies = []

    vision_angle = 120
    vision_radius = 20
    vision_angle_radians = math.radians(vision_angle)
    half_field_of_view_cosine = math.cos(vision_angle_radians / 2)
    
    #x_cell_value = int(agent.location.x // cell_size)
    #y_cell_value = int(agent.location.y // cell_size)
    #grid_cell_key = (x_cell_value, y_cell_value)
    
    #all_neighbors = 0

    #for neighbor in grid[grid_cell_key]:
    for neighbor in all_neighbors:
        vec_to_neighbor = Vector(neighbor.location) - Vector(agent.location)
        len_to_neighbor = vec_to_neighbor.length

        if len_to_neighbor <= vision_radius:
            other_direction = vec_to_neighbor.normalized()
            dot_product = agent.direction.dot(other_direction)

            if neighbor != agent and dot_product > half_field_of_view_cosine:# and 'enemy' in neighbor_agent.lower(): # if enemy in field of view
                visible_neighbors.append(neighbor)
            #
            #elif dot_product > half_field_of_view_cosine: # if neighbor in field of view
            #    visible_neighbors.append(neighbor_location)

    #if visible_enemies:
    #    update_agent_target(agent_name, visible_enemies)

    return visible_neighbors

"""check for collisions (between two vectors"""
def collision_detection(agent, collider, optimisation_rotation=None):
    max_frontal_angle = math.radians(1)
    agent_to_collider = Vector(agent.location) - Vector(collider.location).normalized()
    
    # not very pretty solution, to be changed some time
    if optimisation_rotation:  """this function is also used for the optimisation to find the best rotation so this is where the function is adapted for that"""
        cos_theta = math.cos(optimisation_rotation)
        sin_theta = math.sin(optimisation_rotation)
        x_rotated = agent.direction.x * cos_theta - agent.direction.y * sin_theta
        y_rotated = agent.direction.x * sin_theta + agent.direction.y * cos_theta
        agent_direction = Vector((x_rotated, y_rotated))
    else:
        agent_direction = agent.direction

    scalar_product = agent_to_collider.dot(agent_direction.normalized())
    scalar_product = max(min(scalar_product, 1.0), -1.0) # for no floating point errors that will break cos, values [-1,1]
    agent_collider_angle = math.acos(scalar_product)
    if agent_collider_angle > math.radians(90):
        agent_collider_angle = math.radians(180) - agent_collider_angle

    # --- frontal collision 
    if agent_collider_angle < max_frontal_angle: """for when two agents are nearly directly coming towards each other"""
        #print("---[Frontal Collision]")
        relative_speed = abs(agent.velocity) + abs(collider.velocity)
        time_to_impact = agent_to_collider.length / relative_speed

        collision_point = Vector(agent.location) + (time_to_impact * agent_direction * agent.velocity)
        return collision_point
    
    # --- angled collision
    #print("---[Angled Collision]")
    direction_relation = mathutils.Matrix([ # relationship between agent and collider directions in 2d plane
            [agent_direction.x, -collider.direction.x], 
            [agent_direction.y, -collider.direction.y]
    ])
    location_difference = Vector(collider.location) - Vector(agent.location)
    times_to_impact = direction_relation.inverted() @ location_difference  # [t1, t2] = A^-1 * B
    agent_time_to_impact, collider_time_to_impact = times_to_impact.x, times_to_impact.y
    
    if agent_time_to_impact < 0 or collider_time_to_impact < 0: # collision is in past, have to change the 0 to be a bit minus because there has to be a bit of wiggle room
        return 0 
    
    collision_point = Vector(agent.location) + agent_time_to_impact * agent_direction
    return collision_point """return where the collision will take place"""

"""return the probability of a collision happening"""
def get_probability_of_collision(agent, collider, collision_point):
    target_to_agent = collision_point - Vector(agent.location)
    agent_time_to_collision = target_to_agent.dot(agent.direction) / agent.velocity

    target_to_collider = collision_point - Vector(collider.location)
    collider_time_to_collision = target_to_collider.dot(collider.direction) / collider.velocity

    if agent_time_to_collision < -0.3 and collider_time_to_collision < -0.3: # collision is in past
        return None
    
    agent_time_to_collision = abs(agent_time_to_collision)
    collider_time_to_collision = abs(collider_time_to_collision)

    probability_of_collision = min(collider_time_to_collision, agent_time_to_collision) / max(collider_time_to_collision, agent_time_to_collision)

    return probability_of_collision


def get_urgency():
    print()

"""kind of based on the PLEdestrians paper, calculating the effort exerted for a given path"""
# big question, how do i handle zero velocity, because the effort would be 0
def get_agent_effort(delta_velocity, delta_rotation, agent):
    gravitational_constant = 9.81
    biological_efficiency = 0.25

    length_path_to_target = (Vector(agent.location) - Vector(agent.target_location)).length
    meters_pro_velocity = 1
    if delta_rotation: # path is curved
        third_length = length_path_to_target / 3
        height = (math.tan(delta_rotation) * length_path_to_target / 2) + 0.5
        length_path_to_target = 2*math.sqrt(third_length**2 + height**2) + third_length
        meters_pro_velocity = length_path_to_target / delta_velocity

    #print("length to target:", length_path_to_target)

    mass_factor = agent.mass * gravitational_constant / math.pi
    leg_spring_factor = math.sqrt(3 * gravitational_constant * agent.leg_length / 2)
    parameter_0 = math.pi**2 * delta_velocity**2
    parameter_1 = 6 * gravitational_constant * agent.leg_length
    shape_factor = 1 - math.sqrt(1 - (parameter_0 / parameter_1))

    effort_while_walking = mass_factor * leg_spring_factor * shape_factor

    effort_for_rest_of_path = meters_pro_velocity * effort_while_walking
    immediate_effort = 1 / biological_efficiency # for things like stopping, increase velocity and the energy needed to turn
    
    return immediate_effort + effort_for_rest_of_path


# goal is to minimize this function
def get_collision_and_effort_cost(delta_velocity, delta_rotation, agent, collider):
    # constants that have to be found (could be generated by the character individuality)
    w1 = 0.5
    w2 = 0.5

    collision_probability = None

    target_location = collision_detection(agent, collider, delta_rotation) #optimisation_direction

    if target_location:
        collision_probability = get_probability_of_collision(agent, collider, target_location)
        #print("collision:", target_location)
    else:
        collision_probability = 0
        
    #print("probability:", collision_probability)
    #effort = get_agent_effort(delta_velocity, delta_rotation, agent)
    #print("effort:", effort)

    #total = (w1 * collision_probability * effort) + (w2 * effort)
    effort = 1
    return collision_probability, effort


"""because i didnt want to use mathematical gradients to find the best rotation and velocity change i made a simple function that checks a given amount of rotations and then finds the best out of it"""
def optimisation_function(agent, collider): # does not work really
    min_collision_probability = 0.9
    min_effort = float('inf')

    best_velocity = None
    best_rotation = 0

    velocity_increment = 0.05
    rotation_increment = math.radians(5)

    range_velocity = 5
    range_rotation = 10

    # first check speed
    
    for step in range(0, range_rotation + 1):
        
        delta_rotation = step * rotation_increment
        #print("rotation", math.degrees(delta_rotation))
        #print("delta rotation:", math.degrees(delta_rotation))
        collision_probability, effort = get_collision_and_effort_cost(agent.velocity, delta_rotation, agent, collider)
        #print("probability:", collision_probability)
        #print("--min effort:", min_effort)
        if collision_probability < min_collision_probability:
            #min_effort = (effort / collision_probability)
            min_collision_probability = collision_probability
            best_rotation = delta_rotation
                
    print("stage 2:", best_rotation)

    return best_velocity, best_rotation

def step_location(agent):
    print()

def agent_new_location(agent):
    return agent.location + (agent.velocity * agent.direction.normalized())



### SCRIPT-INITIATION ###

cell_size = 40

agent_names = [
        "agent_0",
        "agent_1",
        "agent_2",
        "agent_3",
        "agent_4",
        "agent_5",
        "agent_6",
        "agent_7"
]
target_names = [ """give targets for each agent, must be in same index as the agent"""
        "target_0",
        "target_1",
        "target_2",
        "target_3",
        "target_4",
        "target_5",
        "target_6",
        "target_7"
]
agent_speed = [
    0.09,
    0.04,
    0.06,
    0.08,
    0.05,
    0.06,
    0.04,
    0.07
]

grid = {}
agents_coordinates = {}
agents_previous_coordinates = {}
agents_directions = {}
agents_previous_directions = {}
agents_target_stack = {}
agents_individuality = {}
agents_dictionary = {}
targets_dictionary = {}

all_path_coords = []
shaders_rgb = []
all_neighbors = []

if bpy.context.mode != 'OBJECT':
    bpy.ops.object.mode_set(mode='OBJECT')

num_agents = len(agent_names)
shader_growth_index = 6/num_agents
shader_index = 0

for index, agent_name in enumerate(agent_names):
    target_name = target_names[index]
    agent_loc = bpy.data.objects.get(agent_name).location
    location = (agent_loc.x, agent_loc.y)
    target_loc = bpy.data.objects.get(target_name).location
    target_vec = (target_loc.x, target_loc.y)
    direction = (Vector(target_vec) - Vector(location)).normalized()

    agents_dictionary[agent_name] = Agent(agent_name, direction, location, agent_speed[index], [target_name], 70, 1)
    first_location = (agent_loc.x, agent_loc.y, 0)
    agent_coords_list = [first_location]
    all_path_coords.append(agent_coords_list)
    targets_dictionary[target_name] = target_name

    shader_index += shader_growth_index
    """to have a variety of colors for the drawing of the paths"""
    if shader_index <= 1:       # index is in range 0-1 
            shaders_rgb.append((1, 0, shader_index, 1))
    elif 1 < shader_index <= 2: # index is in range 1-2 
            shaders_rgb.append((1-(shader_index-1), 0, 1, 1))
    elif 2 < shader_index <= 3: # index is in range 2-3 
            shaders_rgb.append((0, (shader_index-2), 1, 1))
    elif 3 < shader_index <= 4: # index is in range 3-4 
            shaders_rgb.append((0, 1, 1-(shader_index-3), 1))
    elif 4 < shader_index <= 5: # index is in range 4-5 
            shaders_rgb.append(((shader_index-4), 1, 0, 1))
    elif shader_index > 5:      # index is in range 5-6 
            shaders_rgb.append((1, 1-(shader_index-5), 0, 1))



### SCRIPT ###

end_frame = 80

"""simulation loop"""
for frame in range(1, end_frame + 1):
    bpy.context.scene.frame_set(frame)
    bpy.context.view_layer.update()
    print("------------------------------------")
    print(f"###[Frame {frame}]")
    time_start = time.time()

    for index, agent_name in enumerate(agents_dictionary):
        agent = agents_dictionary[agent_name]
        agent_bpy = bpy.data.objects.get(agent_name)
    
        neighbors = objects_in_field_of_view(agent) """get all the neighbors of the agent in a list"""
        
        for neighbor in neighbors: """go through each neighbor and check which one is the closest one"""
            collision_probability, effort = get_collision_and_effort_cost(agent.velocity, None, agent, neighbor)
            print(f"{agent.name} will collide with {neighbor.name} with a chance of {collision_probability}")

            length_agent_to_neighbor = (Vector(agent.location) - Vector(neighbor.location)).length
            print("length agent to neighbor:", length_agent_to_neighbor)
            if length_agent_to_neighbor < agent.min_length_to_neighbor:
                agent.min_length_to_neighbor = length_agent_to_neighbor
                agent.update_nearest_neighbor(neighbor)
        
        if agent.nearest_neighbor: """if there is a nearest neighbor then find the rotation and velocity change and update them"""
            print(f"{agent.name} nearest neighbor is {agent.nearest_neighbor.name}")
            best_velocity, best_rotation = optimisation_function(agent, agent.nearest_neighbor)
            print(f"{agent.name}==best_velocity: {best_velocity}")
            print(f"{agent.name}==best_rotation: {math.degrees(best_rotation)}")
            
            cos_theta = math.cos(best_rotation)
            sin_theta = math.sin(best_rotation)
            x_rotated = agent.direction.x * cos_theta - agent.direction.y * sin_theta
            y_rotated = agent.direction.x * sin_theta + agent.direction.y * cos_theta
            agent_direction = Vector((x_rotated, y_rotated))
            agent.update_frame_data(agent_direction, agent.location, agent.velocity) # causing problems
            new_location = agent_new_location(agent)
            #agent.update_frame_data(agent_direction, new_location, agent.velocity)
        else:
            print(f"seems like there is no nearest_neighbor for {agent.name}")
            new_location = agent_new_location(agent)

        #target_loc

        #calculate rotation urgency
        agent.min_length_to_neighbor

        #new_location = agent_new_location(agent)
        target_locus = bpy.data.objects.get(agent.target).location
        agent_direction = target_locus - Vector((agent.location.x, agent.location.y, 0))
        agent_direction = Vector((agent_direction.x, agent_direction.y)).normalized()
        if new_location and agent_direction:
            agent.update_frame_data(agent_direction, new_location, agent.velocity)

        three_dimensional_location_vector = (new_location[0], new_location[1], 0) # for drawing in blender (testing)

        
        all_path_coords[index].append(three_dimensional_location_vector)

    print("---Calculations: %.4f sec" % (time.time() - time_start))

        

        









step_coords = [(0,0,0), (4,4,4)]

print()


### MODAL-OPERATOR ###
"""blender modal operators used for drawing the paths in blender, based on a default preset from blender"""
def create_keyframes(name, coordinates):
    max_frame = len(coordinates)
    for frame in range(1, max_frame + 1):
        obj = bpy.data.objects.get(name)
        obj.location = coordinates[frame - 1]
        obj.keyframe_insert(data_path="location", frame=frame)

batches = [] # Define the batches list
shader = gpu.shader.from_builtin('UNIFORM_COLOR')

for coords in all_path_coords:
    batch = batch_for_shader(shader, 'LINE_STRIP', {"pos": coords})
    batches.append(batch)

#batch = batch_for_shader(shader, 'LINE_STRIP', {"pos": step_coords})
#batches.append(batch)
#shaders_rgb.append((1,1,0,1))

def draw():
    for batch,color_rgb in zip(batches, shaders_rgb):
        shader.uniform_float("color", color_rgb)
        batch.draw(shader)

my_draw_handler = bpy.types.SpaceView3D.draw_handler_add(draw, (), 'WINDOW', 'POST_VIEW')

def remove_draw_handler():
    bpy.types.SpaceView3D.draw_handler_remove(my_draw_handler, 'WINDOW')
    print("Draw handler removed")

class ModalOperator(bpy.types.Operator):
    bl_idname = "wm.modal_operator"
    bl_label = "Modal Operator"
    
    _timer = None
    last_location = None
    last_frame = None
    
    def modal(self, context, event):
        scene = bpy.context.scene

        if event.type == 'TIMER':
            current_frame = scene.frame_current


            if self.last_frame is None or current_frame != self.last_frame:
                    
                self.last_frame = current_frame

                context.area.tag_redraw() # update the viewport drawing of the lines
 
        if event.type == 'ESC': # cancel script
            remove_draw_handler()
            return {'FINISHED'}
        
        if event.type == 'K': # add keyframes
            for index, name in enumerate(agents_dictionary):
                create_keyframes(name, all_path_coords[index])
            return {'RUNNING_MODAL'}
        
        if event.type == 'R': # remove keyframes (does not work yet)
            for index, name in enumerate(agents_dictionary):
                create_keyframes(name, all_path_coords[index])
            return {'RUNNING_MODAL'}

        return {'PASS_THROUGH'}


    def invoke(self, context, event):
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

# Register the modal operator
def register():
    bpy.utils.register_class(ModalOperator)

# Unregister the modal operator
def unregister():
    bpy.utils.unregister_class(ModalOperator)

# Run the modal operator
register()
bpy.ops.wm.modal_operator('INVOKE_DEFAULT')
