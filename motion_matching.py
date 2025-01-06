import bpy
import math
import mathutils
from mathutils import Vector, Euler
import time
import json

import gpu
from gpu_extras.batch import batch_for_shader



"""IN PROGRESS"""

"""
To-Do:
- move characters to grid
- get list of neighbors

AAAAAAAAAAA
get the fucking blending to work, the function does everything right but the fucking output does not contain the blended rotations

"""


### CLASS ###

class Agent:
    def __init__(self, name, direction, location, velocity, target_list, mass, leg_length):
        self.name = name
        self.velocity = Vector(direction).normalized() * velocity
        self.prev_velocity = Vector(direction).normalized() * velocity
        self.location = Vector(location)
        self.prev_location = Vector(location)
        #self.velocity = velocity
        #self.prev_velocity = velocity

        self.target = target_list[-1]
        target_loc = bpy.data.objects.get(target_list[-1]).location
        self.target_location = (target_loc.x, target_loc.y) # not sure if it is smart to have the target location be part of the agent, multiple agents can have the same moving target
        self.target_stack = target_list

        self.nearest_neighbor = None
        self.min_length_to_neighbor = (Vector(location) - Vector((target_loc.x, target_loc.y))).length
        self.nearest_neighbor_location = target_loc
        
        self.mass = mass
        self.leg_length = leg_length

        agents_directions[name] = direction
        agents_previous_directions[name] = direction
        
        agents_coordinates[name] = location
        agents_previous_coordinates[name] = location

        agents_target_stack[name] = target_list
        all_neighbors.append(self)
        self.armature_name = "armature_" + name
        self.stance_width = 0.16

        self.active_foot = 1 # right foot

        self.bone_all_location_dictionary = {}
        armature = bpy.data.objects.get(self.armature_name)
        for bone in armature.pose.bones:
            self.bone_all_location_dictionary[bone.name] = bone.tail.copy()




        self.active_feet = [False, True] # for most people their strong foot is the right one

        self.bone_groups = {
            "R_Leg": ["R_Up_Leg", "R_Leg"],
            "L_Leg": ["L_Up_Leg", "L_Leg"],
            "R_Arm": ["R_Arm", "R_Fore_Arm"],
            "L_Arm": ["L_Arm", "L_Fore_Arm"]
        }

        bone_order = [
            "Hips", "Spine_01", "Spine_02", "Spine_03", 
            "R_Up_Leg", "R_Leg", 
            "L_Up_Leg", "L_Leg", 
            "R_Arm", "R_Fore_Arm", 
            "L_Arm", "L_Fore_Arm"
        ]

        # Initialize the dictionary
        self.bone_location_dictionary = {}

        self.bone_length = {
            "R_Up_Leg": None,
            "R_Leg": None,
            "L_Up_Leg": None,
            "L_Leg": None,
            "R_Arm": None,
            "R_Fore_Arm": None,
            "L_Arm": None,
            "L_Fore_Arm": None
        }

        self.adaptive_bones = { # add the arms
            "R_Up_Leg": None,
            "R_Leg": None,
            "L_Up_Leg": None,
            "L_Leg": None
        }
        self.non_adaptive_bones = {
            "Hips": None,
            "Spine_0": None,
            "Spine_1": None,
            "Spine_2": None,
            "Neck": None,
            "Head": None,
            "R_Shoulder": None,
            "L_Shoulder": None,
            "R_Arm": None,
            "R_Fore_Arm": None,
            "L_Arm": None,
            "L_Fore_Arm": None
        }
        self.blend_non_adaptive_bones = {
            "Hips": None,
            "Spine_0": None,
            "Spine_1": None,
            "Spine_2": None,
            "Neck": None,
            "Head": None,
            "R_Shoulder": None,
            "L_Shoulder": None,
            "R_Arm": None,
            "R_Fore_Arm": None,
            "L_Arm": None,
            "L_Fore_Arm": None
        }

        self.hip_motion_coordinates = []
        self.hip_forward_axis = None
        self.hip_speed = None

        armature = bpy.data.objects.get(self.armature_name)
        hip_bone = armature.pose.bones["Hips"]
        self.global_hip_position = hip_bone.tail.copy()

        self.global_animation_start_frame = 1
        self.global_animation_end_frame = 1

        self.R_foot_posi_list = []
        self.L_foot_posi_list = []

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


    def get_data_for_leg_animation(self):
        leg_targets = {}

        if self.active_feet[0]: # left foot was active (so right will be next)
            static_leg_home = self.adaptive_bones["L_Leg"][-1] # left foot is static
            active_foot_home = self.adaptive_bones["R_Leg"][-1]
        else:
            static_leg_home = self.adaptive_bones["R_Leg"][-1] # right foot is static
            active_foot_home = self.adaptive_bones["L_Leg"][-1]
        
        active_foot_home = Vector((active_foot_home[0], active_foot_home[1], active_foot_home[2]))
        static_leg_home = Vector((static_leg_home[0], static_leg_home[1], static_leg_home[2]))

        return active_foot_home, static_leg_home



    def get_new_foot_target(self): #, gait_length)
        width = self.stance_width / 2
        #length = agent.velocity * gait_length # of one gait cicle, left foot to left foot again
        length = 1.87
        #length = agent.velocity *
        # some more code for curves
        direction = self.velocity.normalized()

        if self.active_feet[0]: # left foot
            width_vec = Vector((-direction.y, direction.x)) * width
        else:
            width_vec = Vector((direction.y, -direction.x)) * width

        new_pos = (direction * length) + width_vec
        foot_location_coordinates = new_pos + self.location
        print("foot_next pos:", foot_location_coordinates)

        return Vector((foot_location_coordinates.x, foot_location_coordinates.y, 0))


    def stretch_and_scale_curve(self, home_point, target_point, path_coordinates):
        local_path_coordinates = path_coordinates
        print("len pathh:", len(path_coordinates))
        original_base = Vector(local_path_coordinates[0]) - Vector(local_path_coordinates[-1])
        #print(home_point, type(home_point))
        #print(target_point, type(target_point))
        new_base = home_point - target_point
        scale_value = new_base.length / original_base.length

        axis_of_rotation = new_base.cross(original_base)
        angle = new_base.angle(original_base)
        #if axis_of_rotation.length == 0:
            #quaternion_vector = mathutils.Quaternion((1, 0, 0, 0))
        #else:
        quaternion_vector = mathutils.Quaternion(axis_of_rotation.normalized(), -angle)

        retargeted_control_points = []
        retargeted_control_points.append(home_point)

        for vector in local_path_coordinates[1:]:
            vector = Vector(vector)

            vec_to_base_angle = vector.angle(original_base)
            vec_projection_on_base = math.cos(vec_to_base_angle) * vector.length

            rotated_vector = quaternion_vector @ vector
            translated_vector = home_point + rotated_vector
            scale_adjustement = new_base.normalized() * ((vec_projection_on_base * (scale_value - 1)))

            control_point_vector = translated_vector + scale_adjustement
            retargeted_control_points.append(control_point_vector)

        return path_coordinates[:-1]#retargeted_control_points


    def set_leg_animation(self, legs_animation, current_frame, current_target_point):
        self.global_animation_start_frame = current_frame 
        self.global_animation_end_frame = current_frame + (legs_animation.end_frame - 1)
        #self.animation_index = end_frame

        #dynamic_foot = target_list[0]
        #static_foot = target_list[1]

        ##### OVERVIEW
        # this function only sets the leg vectors and rotates the spine

        active_foot_home, static_leg_home = self.get_data_for_leg_animation()


        blend_threshold = 5

        print("animation start:", self.global_animation_start_frame)
        print("animation end:", self.global_animation_end_frame)
        feet_parent = ["R_Leg", "L_Leg"]

        # adaptive part
        
        if self.active_feet[0]: # left foot is active
            current_bone_path = legs_animation.adaptive_bones.get("L_Leg", [])
            retargeted_bone_path = self.stretch_and_scale_curve(active_foot_home, current_target_point, current_bone_path)
            self.adaptive_bones["L_Leg"] = retargeted_bone_path
            self.adaptive_bones["R_Leg"] = [static_leg_home] * len(retargeted_bone_path)

        elif self.active_feet[1]:
            current_bone_path = legs_animation.adaptive_bones.get("R_Leg", [])
            retargeted_bone_path = self.stretch_and_scale_curve(active_foot_home, current_target_point, current_bone_path)
            self.adaptive_bones["R_Leg"] = retargeted_bone_path
            self.adaptive_bones["L_Leg"] = [static_leg_home] * len(retargeted_bone_path)
            

        self.R_foot_posi_list.extend(self.adaptive_bones["R_Leg"]) # for testing

        self.L_foot_posi_list.extend(self.adaptive_bones["L_Leg"]) # for testing


        self.hip_motion_coordinates = legs_animation.hip_motion_coordinates
        self.hip_speed = legs_animation.hip_speed

        nimi = ["L_Arm", "L_Fore_Arm"]
        for bone_name in legs_animation.non_adaptive_bones:
            #self.non_adaptive_bones[bone_name] = legs_animation.non_adaptive_bones[bone_name][blend_threshold:]

            blend_quaternions = self.blend_non_adaptive_bones.get(bone_name, []) # from old animation
            animation_quaternions = legs_animation.non_adaptive_bones.get(bone_name, [])
            if bone_name in nimi:
                if blend_quaternions:
                    #print(dict1)
                    #print(f"transition for {bone_name}")
                    #print(len(blend_quaternions), "and", len(animation_quaternions[:blend_threshold])) # checks out
                    blended_quaternions = self.dynamic_blend_rotations(blend_quaternions, animation_quaternions[:blend_threshold], blend_threshold)
                    #print("blended:", blended_quaternions[0], blended_quaternions[1], blended_quaternions[2], blended_quaternions[3], blended_quaternions[4])
                    #print("normal:", animation_quaternions[2])
                    blended_quaternions.extend(legs_animation.non_adaptive_bones[bone_name][blend_threshold:])
                    #print("in big list:", blended_quaternions[5])
                    #print("in big list:", len(blended_quaternions))
                    #print("animation:", len(animation_quaternions))
                    self.non_adaptive_bones[bone_name] = blended_quaternions
                self.blend_non_adaptive_bones[bone_name] = legs_animation.non_adaptive_bones[bone_name][:blend_threshold]
            else:
                #print(f"normal for {bone_name}")
                self.non_adaptive_bones[bone_name] = animation_quaternions

            #self.non_adaptive_bones[bone_name] = blended_quaternions
            


    def set_arm_animation(self, arms_animation, current_frame):
        self.global_animation_start_frame = current_frame 
        self.global_animation_end_frame = current_frame + (arms_animation.end_frame - 1)
        #self.animation_index = end_frame

        print("animation start:", self.global_animation_start_frame)
        print("animation end:", self.global_animation_end_frame)

        for bone_name in arms_animation.adaptive_bones:
            self.adaptive_bones[bone_name] = arms_animation.adaptive_bones[bone_name]


        for bone_name in arms_animation.non_adaptive_bones:
            self.non_adaptive_bones[bone_name] = arms_animation.non_adaptive_bones[bone_name]




    def dynamic_blend_rotations(self, blend_quaternions, animation_quaternions, blend_threshold):
        blended_list = []

        for frame in range(blend_threshold):

            blend_factor = 1.0 - frame / blend_threshold
            #print("blend factor:", blend_factor)

            # Perform SLERP blending between the two rotations
            original_animation_quaternion = mathutils.Quaternion(blend_quaternions[frame])
            new_animation_quaternion = mathutils.Quaternion(animation_quaternions[frame])

            # the blended quaternion has to be 
            blended_quat = original_animation_quaternion.slerp(new_animation_quaternion, 1.0 - blend_factor)  # Use 1.0 - blend_factor
            blended_list.append(list(blended_quat))

        return blended_list



    def rotate_all_bones(self, current_frame):
        armature = bpy.data.objects.get(self.armature_name)
        current_index = current_frame - self.global_animation_start_frame 
        #print("current index:", current_index)

        for bone_name in self.non_adaptive_bones:
            if not self.non_adaptive_bones[bone_name]:
                continue
            #bpy.context.view_layer.update()
            bone = armature.pose.bones[bone_name]
            if bone_name == "Hips":
                direction = self.velocity.normalized()
                print("####################### Directionj:", direction)
                angle_in_radians = math.atan2(direction.y, direction.x)
                anglus = Vector((0,-1)).angle(direction) + math.radians(180)

                #quaternion = mathutils.Quaternion((0, 0, 1), angle_in_radians)
                #bone.rotation_quaternion = quaternion
                bone.rotation_mode = 'XYZ'  # Set the rotation mode to Euler XYZ
                bone.rotation_euler = mathutils.Euler((0, anglus, 0), 'XYZ')
                bone.keyframe_insert(data_path="rotation_euler", frame=current_frame)


            else:
                bone.rotation_quaternion = self.non_adaptive_bones[bone_name][current_index] 
                bone.keyframe_insert(data_path="rotation_quaternion", frame=current_frame)

        bone = armature.pose.bones["Hips"]
        #bone_location = self.hip_motion_coordinates[current_index]
        bone.location = self.hip_motion_coordinates[current_index]

        current_speed = self.hip_speed[current_index]
        direction = self.velocity.normalized()
        global_direction = Vector((direction.x, direction.y, 0))
        #print("bone location:", bone.location)


        ### x and z axis have to be added to the global location, y does not matter
        print(self.location)
        
        oba = self.hip_motion_coordinates[current_index]
        #oba[2] = 
        #loga = (global_direction * current_speed)
        print("global directon", oba)

        self.global_hip_position += Vector((oba[0], oba[2], current_speed))
        x_value = self.location.x + oba[0]
        forward_value = self.location.y + current_speed
        bone.location = Vector((x_value, oba[1], forward_value))

        bone.keyframe_insert(data_path="location", frame=current_frame)


        # find the rotation in radians towards the goal
        angle_in_radians = math.atan2(direction.y, direction.x)

        #quaternion = mathutils.Quaternion((0, 0, 1), angle_in_radians)
        #z_axis_rotation = mathutils.Quaternion((quaternion.w, quaternion.y, quaternion.z, quaternion.w))
        #print("anglus:", z_axis_rotation)
    
        # Combine the new Z-axis rotation with the bone's existing rotation
        #bone.rotation_quaternion = z_axis_rotation @ bone.rotation_quaternion
        #bone.keyframe_insert(data_path="rotation_quaternion", frame=current_frame)

        self.location += self.velocity



        for bone_name in self.adaptive_bones:
            #print(f"adaptive bones {bone_name}")
            continue
            #bpy.context.view_layer.update()
            side_a_bone_name = self.bone_groups[bone_name][0] ############ IMPORTANT: Still have to check if the reihenfolge is correct
            side_c_bone_name = self.bone_groups[bone_name][1]
            home_point = 1 ########################
            target_position = Vector(self.adaptive_bones[bone_name][current_index])
            ik_vectors = self.get_inverse_kinematics_vectors(self.bone_length[side_a_bone_name], self.bone_length[side_c_bone_name], home_point, target_position)
            self.convert_vector_to_quaternion(ik_vectors)
            bone = armature.pose.bones[self.bone_groups[bone_name][0]]
            bone.keyframe_insert(data_path="rotation_quaternion", frame=current_frame)
            bone = armature.pose.bones[self.bone_groups[bone_name][1]]
            bone.keyframe_insert(data_path="rotation_quaternion", frame=current_frame)







    def convert_vector_to_quaternion(self, ik_coordinates):
        armature = bpy.context.object
        if armature.type != 'ARMATURE':
            raise ValueError("Active object must be an armature")

        # Access the pose bones
        pose_bones = armature.pose.bones

        # Iterate through the path and calculate orientations
        for coordinate_index in range(len(ik_coordinates) - 1):
            start = ik_coordinates[coordinate_index]
            end = ik_coordinates[coordinate_index + 1]
            direction = (end - start).normalized()

            # Assuming bones are named in order like 'Bone.001', 'Bone.002', etc.
            bone_name = f"Bone.{coordinate_index + 1:03d}"
            pose_bone = pose_bones.get(bone_name)
            if pose_bone is None:
                continue
            
            # Define a default up vector (usually Z-axis)
            up_vector = Vector((0, 0, 1))

            # Compute a quaternion to align the bone's +Y axis to the direction vector
            quat = direction.to_track_quat('Y', 'Z')
        quaternion_rotations = []

        return quaternion_rotations
    
    
    def get_inverse_kinematics_vectors(side_a_length, side_c_length, home_point, target_point):
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
        parent_vector = c_side_vec - home_point
        child_vector = a_side_target - c_side_vec

        return [parent_vector, child_vector]




class Animation:
    def __init__(self, animation_name, animation_file_path):
        self.name = animation_name
        
        self.adaptive_bones = {}
        self.non_adaptive_bones = {}
        self.hip_motion_coordinates = []
        
        with open(animation_file_path, 'r') as file:
            animation_data = json.load(file)

        self.adaptive_bone_names = animation_data.get("adaptive_bone_names", [])
        self.non_adaptive_bone_names = animation_data.get("non_adaptive_bone_names", [])
        self.end_frame = animation_data.get("end_frame")
        bones_data = animation_data.get("bones", [])
        
        for bone_data in bones_data:
            bone_name = bone_data["name"]
            #print(bone_name)
            
            if bone_name in self.adaptive_bone_names:
                #print("adaptive:", bone_name)
                coordinates = bone_data.get("coordinates", ())
                self.adaptive_bones[bone_name] = coordinates

            elif bone_name in self.non_adaptive_bone_names:
                #print("non adaptive:", bone_name)
                rotations = bone_data.get("rotations", [])
                self.non_adaptive_bones[bone_name] = rotations
            
            elif bone_name == "Hip_Motion":
                hip_up_down_locations = bone_data.get("up_down", [])
                hip_front_speed = bone_data.get("front", [])
                self.hip_motion_coordinates = hip_up_down_locations
                self.hip_speed = hip_front_speed

                #print("obama:", self.hip_motion_coordinates)

        #print("ini function", self.adaptive_bones)



    
    
### FUNCTIONS ###






def get_best_leg_animation(agent, next_rotation, next_velocity):
    if agent.active_feet[0]:
        agent.active_feet = [False, True]
        animation_name = "legs_R_walking"
    else:
        agent.active_feet = [True, False]
        animation_name = "legs_L_walking"
    
    return animation_name
        


def get_best_arm_animation(next_rotation, next_velocity):
    animation_name = ""
    return animation_name







### SCRIPT-INITIATION ###

animation_links = {
    "legs_R_walking": "G:/Softwares/legs_L_walking.json",
    "legs_L_walking": "G:/Softwares/legs_L_walking.json",
    #"fast_walking": "G:/Softwares/fast_walking.json",
    #"slow_walking": "G",
    #"slow_running": "G",
    #"medium_running": "H",
    #"fast_running": "H"
}

animations_dictionary = {}
    
for animation_name in animation_links:
    animations_dictionary[animation_name] = Animation(animation_name, animation_links[animation_name])


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
Foot_coords = []

############################################################################################
### SCRIPT ###

simulation_start_frame = 1
simulation_end_frame = 20

#R_step_coords.extend(local_path) # only for testing

for frame in range(simulation_start_frame, simulation_end_frame + 1):
    bpy.context.scene.frame_set(frame)
    bpy.context.view_layer.update()
    print("------------------------------------")
    print(f"###[Frame {frame}]")
    time_start = time.time()

    for index, agent_name in enumerate(agents_dictionary):
        agent = agents_dictionary[agent_name]
        agent_bpy = bpy.data.objects.get(agent_name)

        if frame != agent.global_animation_end_frame: # still in animation cycle
            agent.rotate_all_bones(frame)
            #print("normal frame rotate")
            ## stuff
            continue

        ### find next animation
        """
        neighbors = objects_in_field_of_view(agent)
        width = agent.stance_width / 2

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
        """
        next_rotation = 1
        next_velocity = 1
        #print("hippies:", agent.hip_motion_coordinates)
        ### find new animation

        leg_animation_name = get_best_leg_animation(agent, next_rotation, next_velocity)
        arm_animation_name = get_best_arm_animation(next_rotation, next_velocity)
        print(arm_animation_name)

        leg_animation_object = animations_dictionary[leg_animation_name]

        leg_targets = {}
        #dynamic_foot = agent.get_new_foot_target(agent.active_feet)
        if agent.adaptive_bones["R_Leg"] or agent.adaptive_bones["L_Leg"]:
            #static_foot = agent.adaptive_bones["L_Leg"][-1]
            #active_foot_home = agent.adaptive_bones["R_Leg"][-1]
            #leg_targets["L_Leg"] = static_foot
            #leg_targets["R_Leg"] = active_foot_home
            #active_foot_home, leg_targets = agent.get_data_for_leg_animation()
            
            active_foot_target = agent.get_new_foot_target()
            print("active foot target", active_foot_target)
        else:
            #static_foot = agent.adaptive_bones
            #active_foot_home = 
            active_foot_target = agent.get_new_foot_target()
            print("dirs foot target", active_foot_target)
            agent.adaptive_bones["L_Leg"] = [Vector((-0.2,0,0))]
            #leg_targets["R_Leg"] = self.get_new_foot_target()
            agent.adaptive_bones["R_Leg"] = [Vector((0.2,0,0))]
            #leg_targets["L_Leg"] = static_foot
            #leg_targets["R_Leg"] = Vector((2, 1, 0))
            #print("activues", active_foot_target)

        agent.set_leg_animation(leg_animation_object, frame, active_foot_target)

        if arm_animation_name:
            #print()
            arm_animation_object = animations_dictionary[arm_animation_name]
            agent.set_arm_animation(arm_animation_object, frame)
        else:
            #print("normal")
            agent.set_arm_animation(leg_animation_object, frame)




        

        

        agent.rotate_all_bones(frame)
        #all_path_coords[index].append(three_dimensional_location_vector)

    print("---Calculations: %.4f sec" % (time.time() - time_start))

        

        

print()
####################################################################################################

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

agentus = agents_dictionary["agent_0"]


batch = batch_for_shader(shader, 'LINE_STRIP', {"pos": agentus.R_foot_posi_list})
batches.append(batch)
shaders_rgb.append((1,1,0,1))

#batch = batch_for_shader(shader, 'LINE_STRIP', {"pos": agentus.L_foot_posi_list})
#batches.append(batch)
#shaders_rgb.append((0,1,1,1))

batch = batch_for_shader(shader, 'LINE_STRIP', {"pos": L_step_coords})
batches.append(batch)
shaders_rgb.append((1,0,1,1))


batch = batch_for_shader(shader, 'LINE_STRIP', {"pos": Foot_coords})
batches.append(batch)
shaders_rgb.append((0,1,1,1))

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





############################################################################
#################################################

"""

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

#####################################################################################

def get_new_foot_target(agent, foot_side): #, gait_length)
    width = agent.stance_width / 2
    #length = agent.velocity * gait_length # of one gait cicle, left foot to left foot again
    length = 1.87
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




def stretch_and_scale_curve(agent, home, target, bezier_vectors):
    local_bezier_vectors = bezier_vectors.copy()

    original_base = Vector(local_bezier_vectors[0]) - Vector(local_bezier_vectors[-1])
    new_base = home - target
    scale_value = new_base.length / original_base.length

    axis_of_rotation = new_base.cross(original_base)
    angle = new_base.angle(original_base)
    quaternion_vector = mathutils.Quaternion(axis_of_rotation.normalized(), -angle)

    retargeted_control_points = []
    retargeted_control_points.append(home)

    for vector in local_bezier_vectors[1:]:
        vector = Vector(vector)

        vec_to_base_angle = vector.angle(original_base)
        vec_projection_on_base = math.cos(vec_to_base_angle) * vector.length

        rotated_vector = quaternion_vector @ vector
        translated_vector = home + rotated_vector
        scale_adjustement = new_base.normalized() * ((vec_projection_on_base * (scale_value - 1)))

        control_point_vector = translated_vector + scale_adjustement
        retargeted_control_points.append(control_point_vector)

    return retargeted_control_points
"""