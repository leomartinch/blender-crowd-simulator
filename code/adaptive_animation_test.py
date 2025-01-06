import bpy
import math
import mathutils
from mathutils import Vector, Euler
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
        self.armature_name = "armature_" + name
        self.stance_width = 0.16
        self.R_leg_end_frame = 1
        self.L_leg_end_frame = 12
        self.hierarchy = [True, False]

        self.R_leg_coordinates = []
        self.L_leg_coordinates = [Vector((0.50546,0.724783,0))] * 12
        self.R_arm_coordinates = []
        self.L_arm_coordinates = []


        self.bone_all_location_dictionary = {}
        armature = bpy.data.objects.get(self.armature_name)
        for bone in armature.pose.bones:
            self.bone_all_location_dictionary[bone.name] = bone.tail.copy()

        bone_order = [
            "Hips", "Spine_01", "Spine_02", "Spine_03", 
            "R_Up_Leg", "R_Leg", 
            "L_Up_Leg", "L_Leg", 
            "R_Arm", "R_Fore_Arm", 
            "L_Arm", "L_Fore_Arm"
        ]

        # Initialize the dictionary
        self.bone_location_dictionary = {}

        # Iterate over the ordered bone names
        for bone_name in bone_order:
            # Check if the bone exists in the armature
            if bone_name in armature.pose.bones:
                bone = armature.pose.bones[bone_name]
                # Store the bone's tail location in the dictionary
                self.bone_location_dictionary[bone_name] = bone.tail.copy()

    def update_frame_data(self, new_direction, new_location, new_velocity):
        self.prev_direction = self.direction
        self.prev_location = self.location
        self.prev_velocity = self.velocity

        self.direction = new_direction
        self.location = new_location
        self.velocity = new_velocity

    def update_bone_location(self):
        for bone in armature.pose.bones:
            self.bone_all_location_dictionary[bone.name] = bone.tail.copy()

    def update_target_stack(self, target_list):
        self.target_stack.append(target_list)

    def update_nearest_neighbor(self, neighbor):
        self.nearest_neighbor = neighbor
        self.nearest_neighbor_location = (Vector(self.location) - Vector(neighbor.location)).length


    def rotate_all_bones(self, frame):
        armature = bpy.data.objects.get(self.armature_name)

        #for bone_name in self.bone_location_dictionary:
        for bone_name in ["R_Up_Leg", "R_Leg", "L_Up_Leg", "L_Leg"]:
            bpy.context.view_layer.update()
            bone = armature.pose.bones[bone_name]

            head_to_target = self.bone_location_dictionary[bone_name] - bone.head
            direction = head_to_target.normalized()

            current_direction = (bone.tail - bone.head).normalized()
            rotation_axis = current_direction.cross(direction)
            rotation_angle = current_direction.angle(direction)

            rotation_matrix = mathutils.Matrix.Rotation(rotation_angle, 4, rotation_axis)
            bone.matrix = rotation_matrix @ bone.matrix

            bone.keyframe_insert(data_path="rotation_euler", frame=frame)
            




    

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


def collision_detection(agent, collider, optimisation_rotation=None):
    max_frontal_angle = math.radians(1)
    agent_to_collider = Vector(agent.location) - Vector(collider.location).normalized()
    
    # not very pretty solution, to be changed some time
    if optimisation_rotation: 
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
    if agent_collider_angle < max_frontal_angle:
        relative_speed = abs(agent.velocity) + abs(collider.velocity)
        time_to_impact = agent_to_collider.length / relative_speed

        collision_point = Vector(agent.location) + (time_to_impact * agent_direction * agent.velocity)
        return collision_point
    
    # --- angled collision
    direction_relation = mathutils.Matrix([ # relationship between agent and collider directions in 2d plane
            [agent_direction.x, -collider.direction.x], 
            [agent_direction.y, -collider.direction.y]
    ])
    location_difference = Vector(collider.location) - Vector(agent.location)
    times_to_impact = direction_relation.inverted() @ location_difference  # [t1, t2] = A^-1 * B
    agent_time_to_impact, collider_time_to_impact = times_to_impact.x, times_to_impact.y
    
    if agent_time_to_impact < -0.1 or collider_time_to_impact < -0.1: # collision is in past, have to change the 0 to be a bit minus because there has to be a bit of wiggle room
        return 0 
    
    collision_point = Vector(agent.location) + agent_time_to_impact * agent_direction
    return collision_point


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


def optimisation_function(agent, collider): # does not work really
    min_collision_probability = 0.9
    min_effort = float('inf')

    best_velocity = None
    best_rotation = 0

    velocity_increment = 0.05
    rotation_increment = math.radians(5)

    range_velocity = 5
    range_rotation = 10

    for step in range(0, range_rotation + 1):
        delta_rotation = step * rotation_increment
        collision_probability, effort = get_collision_and_effort_cost(agent.velocity, delta_rotation, agent, collider)

        if collision_probability < min_collision_probability:
            min_collision_probability = collision_probability
            best_rotation = delta_rotation
                
    print("stage 2:", best_rotation)

    return best_velocity, best_rotation


def agent_new_location(agent):
    return agent.location + (agent.velocity * agent.direction.normalized())

"""###################################################################################################"""

def get_new_foot_target(agent, foot_side, gait_length):
    width = agent.stance_width / 2
    length = agent.velocity * gait_length # of one gait cicle, left foot to left foot again
    #length = agent.velocity *
    # some more code for curves

    if foot_side == 0: # left foot
        width_vec = Vector((-agent.direction.y, agent.direction.x)) * width
    else:
        width_vec = Vector((agent.direction.y, -agent.direction.x)) * width

    new_pos = (agent.direction * length) + width_vec
    foot_location_coordinates = new_pos + agent.location

    return foot_location_coordinates


#################################################################################################
### bone animation part

def rotate_bone_to_target(bone, target):
    head_to_target = target - bone.head

    direction = head_to_target.normalized()

    current_direction = (bone.tail - bone.head).normalized()
    rotation_axis = current_direction.cross(direction)
    rotation_angle = current_direction.angle(direction)

    rotation_matrix = mathutils.Matrix.Rotation(rotation_angle, 4, rotation_axis)
    bone.matrix = rotation_matrix @ bone.matrix


def character_inverse_kinematics(side_a_length, side_c_length, home_point, target_point):
    #for i, sublist in enumerate(controll_point_names):
    #    if current_point in sublist:
    #        index_sublist = i

    """Need to know the length of the sides, home point and target point as vectors"""

    #armature = bpy.data.objects[agent.armature_name]

    # arms
    #if index_sublist < 1.5:
    #side_a = (armature.pose.bones["L_Leg"].tail - armature.pose.bones["L_Leg"].head).length
    #side_c = (armature.pose.bones["L_Up_Leg"].head - armature.pose.bones["L_Up_Leg"].tail).length

    # legs
    #elif index_sublist > 1.5:
    #    side_a = side_a_leg
    #    side_c = side_c_leg

    #direction_point = #bpy.data.objects.get(controll_point_names[index_sublist][2]).location
    #home_point = #armature.pose.bones[controll_point_names[index_sublist][4]].head
    
    #bone_side_c = armature.pose.bones[controll_point_names[index_sublist][4]] # get side c bone
    #bone_side_a = armature.pose.bones[controll_point_names[index_sublist][5]] # get side a bone

    #direction_point = Vector((10,0,1)) # for testing
    direction_point = bpy.data.objects.get("target_0").location * 20

    side_b = (home_point - target_point).length

    bone_positions = [0,0,0]

    home_to_target = (home_point - target_point) # home point to target point
    home_to_direction_point = (home_point - direction_point) # home point to extra point (to form a plane)

    if (abs(side_c_length - side_a_length)) <= side_b <= (side_c_length + side_a_length): # if target inside bounds
        axis_of_rotation = home_to_target.cross(home_to_direction_point)
    
        alpha_top = side_b**2 + side_c_length**2 - side_a_length**2
        alpha_bot = 2 * side_b * side_c_length
        alpha = math.acos(alpha_top / alpha_bot)
        alpha = -(math.radians(180) - alpha)

        quaternion_vec = mathutils.Quaternion(axis_of_rotation.normalized(), alpha)

        c_side_vec = side_c_length * home_to_target.normalized()

        point_B = quaternion_vec @ c_side_vec
        point_B = point_B + home_point

        bone_positions[0] = home_point
        bone_positions[1] = point_B
        bone_positions[2] = target_point

    else:
        home_to_target = (target_point - home_point)
        home_to_direction_point = (direction_point - home_point)

        c_side_vec = side_c_length * home_to_target.normalized()
        c_side_vec += home_point

        if side_b < (abs(side_c_length - side_a_length)): # if target inside bounds
            c_side_target_vec = (c_side_vec - target_point)
            a_side_target = side_a_length * c_side_target_vec.normalized()
        else:
            a_side_target = (side_a_length + side_c_length) * home_to_target.normalized()

        a_side_target += home_point

        bone_positions[0] = home_point
        bone_positions[1] = c_side_vec
        bone_positions[2] = a_side_target

        point_B = c_side_vec
        target_point = a_side_target
    """
    bpy.ops.mesh.primitive_cube_add(size=0.1, location=(0, 0, 0))
    obj = bpy.context.active_object
    obj.name = f"home"
    obj.location = bone_positions[0]
    bpy.ops.mesh.primitive_cube_add(size=0.1, location=(0, 0, 0))
    obj = bpy.context.active_object
    obj.name = f"knee"
    obj.location = bone_positions[1]
    bpy.ops.mesh.primitive_cube_add(size=0.1, location=(0, 0, 0))
    obj = bpy.context.active_object
    obj.name = f"target"
    obj.location = bone_positions[2]
    """
    return bone_positions

    #rotate_bone_to_target(bone_side_c, point_B)
    #bpy.context.view_layer.update()
    #rotate_bone_to_target(bone_side_a, target_point)
    #bpy.context.view_layer.update()


def stretch_and_scale_curve(agent, home, target, bezier_vectors):
    local_bezier_vectors = bezier_vectors.copy()

    original_base = local_bezier_vectors[0] - local_bezier_vectors[-1]
    new_base = home - target
    scale_value = new_base.length / original_base.length

    axis_of_rotation = new_base.cross(original_base)
    angle = new_base.angle(original_base)
    quaternion_vector = mathutils.Quaternion(axis_of_rotation.normalized(), -angle)

    retargeted_control_points = []
    retargeted_control_points.append(home)

    for vector in local_bezier_vectors[1:]:

        vec_to_base_angle = vector.angle(original_base)
        vec_projection_on_base = math.cos(vec_to_base_angle) * vector.length

        rotated_vector = quaternion_vector @ vector
        translated_vector = home + rotated_vector
        scale_adjustement = new_base.normalized() * ((vec_projection_on_base * (scale_value - 1)))

        control_point_vector = translated_vector + scale_adjustement
        retargeted_control_points.append(control_point_vector)

    return retargeted_control_points


def bezier_curve(progress, control_points):
    x_coord = (
            (1-progress)**3 * control_points[0][0] + 
            3 * progress * (1-progress)**2 * control_points[1][0] +
            3 * progress**2 * (1-progress) * control_points[2][0] +
            progress**3 * control_points[3][0]
        )
    y_coord = (
            (1-progress)**3 * control_points[0][1] + 
            3 * progress * (1-progress)**2 * control_points[1][1] +
            3 * progress**2 * (1-progress) * control_points[2][1] +
            progress**3 * control_points[3][1]
        )
    z_coord = (
            (1-progress)**3 * control_points[0][2] + 
            3 * progress * (1-progress)**2 * control_points[1][2] +
            3 * progress**2 * (1-progress) * control_points[2][2] +
            progress**3 * control_points[3][2]
        )

    return Vector((x_coord, y_coord, z_coord))



def create_animation_path(animation_length, control_point_list):
    local_bezier_curve_coordinates = []

    time_progress = 0
    interval = 2/animation_length

    for i in range(animation_length):
        if time_progress <= 1:
            local_bezier_curve_coordinates.append(bezier_curve(time_progress, control_point_list[:4])) 
            time_progress += interval

        elif time_progress > 1:
            local_bezier_curve_coordinates.append(bezier_curve((time_progress - 1), control_point_list[3:7])) 
            time_progress += interval

    local_bezier_curve_coordinates.append(bezier_curve(1, control_point_list[3:7])) 
            
    return local_bezier_curve_coordinates





### SCRIPT-INITIATION ###

foot_bezier_vec = [Vector((0.0, 0.0, 0.0)), Vector((0.15840864181518555, -0.004654914140701294, 0.24591614305973053)), Vector((0.519545316696167, -0.010119594633579254, 0.2040533423423767)), Vector((1.0203156471252441, 0.003473609685897827, 0.0861692950129509)), Vector((1.5210859775543213, 0.01706681400537491, -0.031714752316474915)), Vector((1.741530418395996, 0.004816815257072449, 0.13751499354839325)), Vector((1.841078519821167, 0.0, 0.0))]




locuun = character_inverse_kinematics(2, 4, Vector((0,0,0)), Vector((3,0,0)))
print("lcoaminini:", locuun)






cell_size = 40

agent_names = [
        "agent_0"
]
target_names = [
        "target_0"
]
agent_speed = [
    0.08
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


R_step_coords = []
L_step_coords = []



def execute_locomotion(agent, hierarchy, frame):

    local_frame = (frame - agent.R_leg_end_frame) + 22 - 1

    if hierarchy[0]: # right leg is swinging
        R_local_target = agent.R_leg_coordinates[local_frame]
        locations = character_inverse_kinematics(0.445212, 0.445783, agent.bone_all_location_dictionary["R_Hip"], R_local_target)
        agent.bone_location_dictionary["R_Up_Leg"] = locations[1]
        agent.bone_location_dictionary["R_Leg"] = locations[2]

    elif hierarchy[1]: # left leg is swinging
        L_local_target = agent.L_leg_coordinates[local_frame]
        locations = character_inverse_kinematics(0.445212, 0.445783, agent.bone_all_location_dictionary["L_Hip"], L_local_target)
        agent.bone_location_dictionary["L_Up_Leg"] = locations[1]
        agent.bone_location_dictionary["L_Leg"] = locations[2]








### SCRIPT ###

end_frame = 40








for frame in range(1, end_frame + 1):
    bpy.context.scene.frame_set(frame)
    bpy.context.view_layer.update()
    print("------------------------------------")
    #print(f"###[Frame {frame}]")
    time_start = time.time()

    for index, agent_name in enumerate(agents_dictionary):
        agent = agents_dictionary[agent_name]
        agent_bpy = bpy.data.objects.get(agent_name)

        neighbors = objects_in_field_of_view(agent)
        width = agent.stance_width / 2

        if frame == agent.R_leg_end_frame: # find new point for right leg
            new_loca = get_new_foot_target(agent, 1, 22)
            #bpy.ops.mesh.primitive_cube_add(size=0.1, location=(0, 0, 0))
            #obj = bpy.context.active_object
            #obj.name = f"foot_2"
            #obj.location = Vector((new_loca[0], new_loca[1], 0))

            width_vec = Vector((agent.direction.y, -agent.direction.x, 0)) * width
            locus = Vector((agent.location.x, agent.location.y, 0)) + width_vec
            bezier_control_points = stretch_and_scale_curve(0, locus, Vector((new_loca[0], new_loca[1], 0)), foot_bezier_vec)
            local_path = create_animation_path(22, bezier_control_points) # now i have the local path for the leg of length x (22)
            agent.R_leg_end_frame = frame + 22
            agent.R_leg_coordinates = local_path
            R_step_coords.extend(local_path) # only for testing
            agent.hierarchy = [True, False]

        if frame == agent.L_leg_end_frame: # find new point for left leg
            new_loca = get_new_foot_target(agent, 0, 22)
            #bpy.ops.mesh.primitive_cube_add(size=0.1, location=(0, 0, 0))
            #obj = bpy.context.active_object
            #obj.name = f"foot_2"
            #obj.location = Vector((new_loca[0], new_loca[1], 0))

            width_vec = Vector((-agent.direction.y, agent.direction.x, 0)) * width
            locus = Vector((agent.location.x, agent.location.y, 0)) + width_vec
            bezier_control_points = stretch_and_scale_curve(0, locus, Vector((new_loca[0], new_loca[1], 0)), foot_bezier_vec)
            local_path = create_animation_path(22, bezier_control_points) # now i have the local path for the leg of length x (22)
            agent.L_leg_end_frame = frame + 22
            agent.L_leg_coordinates = local_path
            L_step_coords.extend(local_path) # only for testing
            agent.hierarchy = [False, True]



        #find agents
        for neighbor in neighbors:
            collision_probability, effort = get_collision_and_effort_cost(agent.velocity, None, agent, neighbor)
            print(f"{agent.name} will collide with {neighbor.name} with a chance of {collision_probability}")

            length_agent_to_neighbor = (Vector(agent.location) - Vector(neighbor.location)).length
            print("length agent to neighbor:", length_agent_to_neighbor)
            if length_agent_to_neighbor < agent.min_length_to_neighbor:
                agent.min_length_to_neighbor = length_agent_to_neighbor
                agent.update_nearest_neighbor(neighbor)
        
        if agent.nearest_neighbor:
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
            #print(f"seems like there is no nearest_neighbor for {agent.name}")
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
        armature = bpy.data.objects[agent.armature_name]

        #if armature.mode != 'POSE':
            #bpy.context.view_layer.objects.active = armature
            #bpy.ops.object.mode_set(mode='POSE')
        #bpy.ops.object.mode_set(mode='POSE')
        boneg = armature.pose.bones["Hips"]
        boneg.location.x = new_location[0]
        boneg.location.y = 0
        boneg.location.z = -new_location[1]
        boneg.keyframe_insert(data_path="location", frame=frame)
        angle_radians = agent.direction.angle(Vector((0,-1)))
        boneg.rotation_euler = Euler((0, angle_radians, 0), 'XYZ')
        boneg.keyframe_insert(data_path="rotation_euler", frame=frame)
        bpy.context.view_layer.update()
        #agent.bone_all_location_dictionary["Hips"] = bone.tail.copy()
        agent.update_bone_location()
        #agent.move_armature
        

        execute_locomotion(agent, [True, False], frame)

        agent.rotate_all_bones(frame)


        #bpy.ops.object.mode_set(mode='OBJECT')

        
        all_path_coords[index].append(three_dimensional_location_vector)

    print("---Calculations: %.4f sec" % (time.time() - time_start))

        

        

print()


### MODAL-OPERATOR ###

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

batch = batch_for_shader(shader, 'LINE_STRIP', {"pos": R_step_coords})
batches.append(batch)
shaders_rgb.append((1,1,0,1))

batch = batch_for_shader(shader, 'LINE_STRIP', {"pos": L_step_coords})
batches.append(batch)
shaders_rgb.append((1,1,0,1))

#batch = batch_for_shader(shader, 'LINE_STRIP', {"pos": locuun})
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
        
        #if event.type == 'R': # remove keyframes (does not work yet)
        #    for index, name in enumerate(agents_dictionary):
        #        create_keyframes(name, all_path_coords[index])
        #    return {'RUNNING_MODAL'}

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
