import bpy
import math
import mathutils
from mathutils import Vector, Euler
import time
import json
import heapq
import random

import gpu
from gpu_extras.batch import batch_for_shader



"""IN PROGRESS"""

"""
To-Do:
- move characters to grid
- get list of neighbors

AAAAAAAAAAA
get the ****** blending to work, the function does everything right but the ******* output does not contain the blended rotations

"""


### CLASS ###

class Agent:
    def __init__(self, name, direction, location, velocity, target_location, mass, leg_length, agent_path):
        self.name = name
        self.velocity = Vector(direction).normalized() * velocity
        self.schnelligkeit = velocity
        self.prev_velocity = Vector(direction).normalized() * velocity
        self.location = Vector(location)
        self.prev_location = Vector(location)
        self.path = agent_path


        #self.target = target_list[-1]
        #target_loc = bpy.data.objects.get(target_list[-1]).location
        self.target_location = target_location #(target_loc.x, target_loc.y) # not sure if it is smart to have the target location be part of the agent, multiple agents can have the same moving target
        #self.target_stack = target_list

        self.nearest_neighbor = None
        self.min_length_to_neighbor = (Vector(location) - Vector((target_loc.x, target_loc.y))).length
        self.nearest_neighbor_location = target_loc
        
        self.mass = mass
        self.leg_length = leg_length

        agents_directions[name] = direction
        agents_previous_directions[name] = direction
        
        agents_coordinates[name] = location
        agents_previous_coordinates[name] = location

        #agents_target_stack[name] = target_list
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
            """here the rotation values of a cycle are stored which will be used to blend with the following animation cycle"""
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

        numiii = random.randint(0,16)
        self.global_animation_start_frame = numiii
        self.global_animation_end_frame = numiii
        self.at_target = False

        self.R_foot_posi_list = []
        self.L_foot_posi_list = []
        
        self.radius = 1.5# has to be made random
        self.rotation_smoothnes = 25
        self.path_find_radius = 1
        self.width = 1.4
        self.end_target_destination_radius = 0.4
        self.current_path_target = self.target_location
        self.best_rotation = None
        self.smooth_rotation_list = []

    def update_frame_data(self):
        self.prev_location = self.location

        # reset velocity

        target_loc = self.current_path_target
        target_vec = (target_loc[0], target_loc[1])
        new_direction = (Vector(target_vec) - Vector(self.location)).normalized()

        self.velocity = new_direction * self.schnelligkeit
        #self.velocity = agent_direction * self.schnelligkeit
        #print(new_direction)



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

    def get_best_rotation(self):
        return self.best_rotation

    def write_best_rotation(self, best_rotation):
        #print("best rotation", best_rotation, self.name)
        if len(self.smooth_rotation_list) > self.rotation_smoothnes:
            self.smooth_rotation_list.pop(0)
        self.smooth_rotation_list.append(best_rotation)
        #self.best_rotation = best_rotation
        gesamt = 0
        for i in self.smooth_rotation_list:
            gesamt += i
        self.best_rotation = gesamt / len(self.smooth_rotation_list)



    def ffind_closest_target(self):
        in_radius = {}
        for point in self.path:
            length = (self.location - Vector(point)).length
            #print(self.location)
            if length <= self.path_find_radius:
                #print("in radius")
                in_radius[point] = (Vector(point) - Vector(self.target_location)).length
            #else:
                #print("not in radius", length)

        smallest_len = float('inf')
        best_target = None
        #print(f"{self.name} {in_radius}")
        #to_target = (Vector(self.location) - Vector(self.target_location)).length
        #print(f"{to_target}")
        #print(self.path)
        for leng in in_radius:
            if in_radius[leng] < smallest_len:
                smallest_len = in_radius[leng]
                best_target = leng

        #if smallest_len < 1: # arrived at target
            #print()
            #print("at target")
            # do st to make the character stop, also make the stop animation

        self.current_path_target = best_target
        #return best_target

    def find_closest_target(self):
        avodi = (self.location - Vector(self.target_location)).length
        if avodi < self.end_target_destination_radius:
            self.at_target = True
            print("amogus")
            return
        in_radius = {}
        for point in self.path:
            length = (self.location - Vector(point)).length
            #print(self.location)
            if length <= self.path_find_radius:
                #print("in radius")
                in_radius[point] = (Vector(point) - Vector(self.target_location)).length
            #else:
                #print("not in radius", length)

        smallest_len = float('inf')
        best_target = None
        #print(f"{self.name} {in_radius}")
        #to_target = (Vector(self.location) - Vector(self.target_location)).length
        #print(f"{to_target}")
        #print(self.path)
        if not in_radius: # no closest
            for point in self.path:
                length = (self.location - Vector(point)).length
                in_radius[point] = length

        for leng in in_radius:
            if in_radius[leng] < smallest_len:
                smallest_len = in_radius[leng]
                best_target = leng

        #if smallest_len < 1: # arrived at target
            #print()
            #print("at target")
            # do st to make the character stop, also make the stop animation

        self.current_path_target = best_target
        #return best_target
        
        
    """a bit missleading, this function sets the rotation values for the legs and arms"""
    def set_leg_animation(self, legs_animation, current_frame, current_target_point):
        self.global_animation_start_frame = current_frame 
        self.global_animation_end_frame = current_frame + (legs_animation.end_frame - 1)
        #self.animation_index = end_frame

        #dynamic_foot = target_list[0]
        #static_foot = target_list[1]

        ##### OVERVIEW
        # this function only sets the leg vectors and rotates the spine

        active_foot_home, static_leg_home = 1,2#self.get_data_for_leg_animation()


        blend_threshold = 5

        #print("animation start:", self.global_animation_start_frame)
        #print("animation end:", self.global_animation_end_frame)
        feet_parent = ["R_Leg", "L_Leg"]

        # adaptive part
        
        #if self.active_feet[0]: # left foot is active
        #    current_bone_path = legs_animation.adaptive_bones.get("L_Leg", [])
        #    #retargeted_bone_path = self.stretch_and_scale_curve(active_foot_home, current_target_point, current_bone_path)
        #    #self.adaptive_bones["L_Leg"] = retargeted_bone_path
        #    #self.adaptive_bones["R_Leg"] = [static_leg_home] * len(retargeted_bone_path)
#
        #elif self.active_feet[1]:
        #    current_bone_path = legs_animation.adaptive_bones.get("R_Leg", [])
        #    #retargeted_bone_path = self.stretch_and_scale_curve(active_foot_home, current_target_point, current_bone_path)
        #    #self.adaptive_bones["R_Leg"] = retargeted_bone_path
        #    #self.adaptive_bones["L_Leg"] = [static_leg_home] * len(retargeted_bone_path)
        #    

        #self.R_foot_posi_list.extend(self.adaptive_bones["R_Leg"]) # for testing

        #self.L_foot_posi_list.extend(self.adaptive_bones["L_Leg"]) # for testing


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
            

    """is not used, the arms are still moved in the leg function above for simplicity in testing"""
    def set_arm_animation(self, arms_animation, current_frame):
        self.global_animation_start_frame = current_frame 
        self.global_animation_end_frame = current_frame + (arms_animation.end_frame - 1)
        #self.animation_index = end_frame

        #print("animation start:", self.global_animation_start_frame)
        #print("animation end:", self.global_animation_end_frame)

        for bone_name in arms_animation.adaptive_bones:
            self.adaptive_bones[bone_name] = arms_animation.adaptive_bones[bone_name]


        for bone_name in arms_animation.non_adaptive_bones:
            self.non_adaptive_bones[bone_name] = arms_animation.non_adaptive_bones[bone_name]



    """blends between two given lists of bone rotations, works perfectly but somehow the bones then dont get the blended list, but if values are printed out and manually put it in, the values are correct"""
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


    """rotate all bones of the character (rotating is a bit missleading because the hip bone (parent bone of the armature) is changed in translation and rotation). its just the function that is called on every frame to update the bone movements"""
    def rotate_all_bones(self, current_frame):
        armature = bpy.data.objects.get(self.armature_name)
        current_index = current_frame - self.global_animation_start_frame 
        #print(current_frame)
        #print("current index:", current_index)

        for bone_name in self.non_adaptive_bones: 
            """iterate through all the bones that are not rotated with the inverse kinematics and rotate them according to the rotation values for the bone"""
            if not self.non_adaptive_bones[bone_name]:
                continue
            #bpy.context.view_layer.update()
            bone = armature.pose.bones[bone_name]
            if bone_name == "Hips": 
                """change hip location"""
                direction = self.velocity.normalized()

                best_rotation = self.get_best_rotation()
                if best_rotation:
                    cos_theta = math.cos(best_rotation)
                    sin_theta = math.sin(best_rotation)
                    x_rotated = direction.x * cos_theta - direction.y * sin_theta
                    y_rotated = direction.x * sin_theta + direction.y * cos_theta
                    direction = Vector((x_rotated, y_rotated))

                forward = Vector((0,-1))
                anglus = forward.angle(direction)
                cross = forward.cross(direction)

                # In Blender's coordinate system, the Z-axis often determines "up".
                # Check the sign of the Z component of the cross product to decide if the angle should be negative
                if cross < 0:
                    anglus = -anglus
                
                #rotation_quat = Vector((0,-1,0)).rotation_difference(Vector((direction[0], direction[1], 0)))
                #bone.rotation_mode = 'QUATERNION'
                #bone.rotation_quaternion = rotation_quat
                bone.rotation_mode = 'XYZ'  # Set the rotation mode to Euler XYZ
                bone.rotation_euler = mathutils.Euler((0, anglus, 0), 'XYZ')
                bone.keyframe_insert(data_path="rotation_euler", frame=current_frame)
                #bone.keyframe_insert(data_path="rotation_quaternion", frame=current_frame)


            else:
                bone.rotation_quaternion = self.non_adaptive_bones[bone_name][current_index] 
                bone.keyframe_insert(data_path="rotation_quaternion", frame=current_frame)

        bone = armature.pose.bones["Hips"]
        #bone_location = self.hip_motion_coordinates[current_index]
        bone.location = self.hip_motion_coordinates[current_index]

        current_speed = self.hip_speed[current_index]
        #
        direction = self.velocity.normalized()
        best_rotation = self.get_best_rotation()
        if best_rotation:
            cos_theta = math.cos(best_rotation)
            sin_theta = math.sin(best_rotation)
            x_rotated = direction.x * cos_theta - direction.y * sin_theta
            y_rotated = direction.x * sin_theta + direction.y * cos_theta
            direction = Vector((x_rotated, y_rotated))
            #print((x_rotated, y_rotated))
            #print("best rotation", best_rotation)
        #else:
            
            #print("not best rotation")
        
        #global_direction = Vector((direction.x, direction.y, 0))

        #oba = self.hip_motion_coordinates[current_index]

        #global_direction = Vector((direction.x, direction.y, 0))

        # Update the bone location using the correct direction
        #bone.location = self.hip_motion_coordinates[current_index]  # Starting location
        current_speed = self.hip_speed[current_index]  # Movement magnitude
        #print((direction))
        # Compute new global position based on direction and speed
        #new_x = self.location.x + (direction.x * current_speed)
        #new_y = self.location.y + (direction.y * current_speed)
        new_x = (direction.x * current_speed)
        new_y = (direction.y * current_speed)

        # Apply the new position
        #bone.location = Vector((new_x, 0, -new_y))
        bone.location = Vector((self.location.x + new_x, 0, -(self.location.y + new_y)))
        #print((new_x, 0, -new_y))
        #self.global_hip_position += Vector((oba[0], oba[2], current_speed))
        #x_value = self.location.x + oba[0]
        #forward_value = self.location.y + current_speed
        #bone.location = Vector((x_value, 0, forward_value))

        bone.keyframe_insert(data_path="location", frame=current_frame)


        # find the rotation in radians towards the goal
        angle_in_radians = math.atan2(direction.y, direction.x)

        self.R_foot_posi_list.append(Vector((self.location.x, self.location.y, 0)))
        self.location += Vector((new_x, new_y))
        #self.write_best_rotation(None)
        #self.update_frame_data()










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




class Heatmap:
    def __init__(self, grid_size, cell_size, heatmap_origin, static_obstacles):
        number_of_cells = int(grid_size / cell_size) # along one axis
        self.cell_size = cell_size
        self.grid_rows = number_of_cells
        self.grid_columns = number_of_cells
        self.heatmap_origin = heatmap_origin
        
        self.grid = []
        for row in range(self.grid_rows):
            current_row = [1] * self.grid_columns
            self.grid.append(current_row)

        # create the heatmap
        for current_row in range(self.grid_rows):
            for current_column in range(self.grid_columns):
                current_cell_value = self.get_cell_value(current_column, current_row, static_obstacles)
                self.grid[current_row][current_column] = current_cell_value
        #print("finish heatmap cration")


    def is_cell_in_grid(self, x_position, y_position):  # is cell real (within the grid boundaries)
        return 0 <= x_position < self.grid_columns and 0 <= y_position < self.grid_rows

    def is_cell_on_target(self, x_position, y_position, target):  # Check if a cell matches the target
        return (x_position, y_position) == target

    def get_line_length(self, x_position, y_position, target_position):  # Calculate heuristic using Euclidean distance
        return math.sqrt((x_position - target_position[0])**2 + (y_position - target_position[1])**2)

    def convert_global_coords_to_local(self, global_x, global_y):
        local_x = global_x / self.cell_size + (self.cell_size / 2) # point is at center of cell
        local_y = global_y / self.cell_size + (self.cell_size / 2)
        return (int(local_x), int(local_y))

    def convert_local_coords_to_global(self, current_x, current_y):
        global_x = current_x * self.cell_size + (self.cell_size / 2) # point is at center of cell
        global_y = current_y * self.cell_size + (self.cell_size / 2)
        global_x = global_x + self.heatmap_origin[0]
        global_y = global_y + self.heatmap_origin[1]
        return global_x, global_y

    def reconstruct_path(self, parent_map, target):
        #print("does reconstruct")
        original_path = [] # for testing (draw map)
        path = []
        current_x, current_y = target
        while (current_x, current_y) in parent_map:
            original_path.append((current_x, current_y))
            global_x, global_y = self.convert_local_coords_to_global(current_x, current_y)
            path.append((global_x, global_y))
            current_x, current_y = parent_map[(current_x, current_y)]
        path.reverse()
        original_path.reverse()
        #if path:
            #print("obama is in the house")
        self.path = path
        self.original_path = original_path
        return path#, original_path

    # implementation of the A* search algorithm
    def shortest_path_search(self, start_position, target_position):
        self.start_position = self.convert_global_coords_to_local(start_position[0], start_position[1])
        self.target_position = self.convert_global_coords_to_local(target_position[0], target_position[1])

        #self.start_position = start_position
        #self.target_position = target_position
        #print(self.start_position, self.target_position)

        open_cells = []
        heapq.heappush(open_cells, (0, 0, self.start_position[0], self.start_position[1]))  # (f_cost, g_cost, x_position, y_position)
        parent_map = {}
        cost_from_start = {self.start_position: 0}  # Tracks g_cost for each cell
        closed_set = set() # cells that have already been visited

        movement_directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while open_cells:
            current_f_cost, current_g_cost, current_x, current_y = heapq.heappop(open_cells)

            if self.is_cell_on_target(current_x, current_y, self.target_position):
                #print("obarlimim")
                return self.reconstruct_path(parent_map, self.target_position)
                #return

            closed_set.add((current_x, current_y)) # Mark the cell as explored
            is_direction_blocked = {
                (0,1): False,
                (0,-1): False,
                (1,0): False,
                (-1,0): False
            }

            # explore all valid and unblocked neighboring cells
            for delta_x, delta_y in movement_directions:
                neighbor_x = current_x + delta_x
                neighbor_y = current_y + delta_y

                # if neighbor cell is not in grid, if is blocked or has already been visited, then continue
                if not self.is_cell_in_grid(neighbor_x, neighbor_y) or not self.grid[neighbor_y][neighbor_x] > 0 or (neighbor_x, neighbor_y) in closed_set:
                    if delta_x == 0 or delta_y == 0:
                        is_direction_blocked[(delta_x, delta_y)] = True
                    continue

                cell_cost = self.grid[neighbor_y][neighbor_x] # get the cell value from the heatmap grid
                if delta_x != 0 and delta_y != 0:
                    if is_direction_blocked[(delta_x, 0)] and is_direction_blocked[(0, delta_y)]: # diagonal path is blocked by two frontal blocks
                        continue
                    move_cost = math.sqrt(2) # diagonal movement
                else:
                    move_cost = 1 # forward movement
                adjusted_cost = move_cost / cell_cost  # Higher values are preferred

                estimated_g_cost = current_g_cost + adjusted_cost

                # Check if the neighbor is either unvisited or has a lower tentative cost
                if (neighbor_x, neighbor_y) not in cost_from_start or estimated_g_cost < cost_from_start[(neighbor_x, neighbor_y)]:
                    cost_from_start[(neighbor_x, neighbor_y)] = estimated_g_cost

                    f_cost = estimated_g_cost + self.get_line_length(neighbor_x, neighbor_y, self.target_position)

                    heapq.heappush(open_cells, (f_cost, estimated_g_cost, neighbor_x, neighbor_y))
                    parent_map[(neighbor_x, neighbor_y)] = (current_x, current_y)
                    #print(is_direction_blocked)

        print("Failed to find the target.")
        return None # second none only for testing

    def get_cell_value(self, grid_x_position, grid_y_position, obstacle_list):
        x_cell_center = self.heatmap_origin[0] + (grid_x_position + 0.5) * self.cell_size
        y_cell_center = self.heatmap_origin[1] + (grid_y_position + 0.5) * self.cell_size
        cell_center = Vector((x_cell_center, y_cell_center, 20))
        ray_direction = Vector((0, 0, -1))
        cell_value = 1

        for obstacle in obstacle_list:
            depsgraph = bpy.context.view_layer.depsgraph # blender dependency graph
            evaluated_obstacle = obstacle.evaluated_get(depsgraph) # to get the obstacle with all the modifiers etc 
            hit, _, _, _ = evaluated_obstacle.ray_cast(cell_center, ray_direction)
            if hit and obstacle_list[obstacle] == 1:
                return 0
            elif hit:
                cell_value -= obstacle_list[obstacle]

        return max(cell_value, 0.01)
    

    def vector_values_in_grid(self, start, end, num_samples):
        #num_samples = 50

        x0, y0 = self.convert_global_coords_to_local(start.x, start.x)
        x1, y1 = self.convert_global_coords_to_local(end.x, end.x)
        #start = startt[0]
        #end = startt[1]


        # Generate sample points along the vector
        #x0, y0 = start
        #x1, y1 = end

        x_samples = []
        for i in range(num_samples):
            x_samples.append(x0 + (x1 - x0) * i / (num_samples - 1))

        y_samples = []
        for i in range(num_samples):
            y_samples.append(y0 + (y1 - y0) * i / (num_samples - 1))

        # Accumulate the length based on grid values
        total_value = 0.0
        for x, y in zip(x_samples, y_samples):
            #print(x, y)
            # Get the integer coordinates for the grid cell
            grid_x, grid_y = int(x), int(y)

            # Check if the point is within the grid bounds
            if 0 <= grid_x < len(self.grid[0]) and 0 <= grid_y < len(self.grid):
                total_value += (1 - self.grid[grid_y][grid_x])  # Add the grid value at this point
        total_value = total_value / num_samples
        return total_value
    

    def draw_grid(self):
        # to render scene in terminal
        #print()
        for path_point in self.original_path:
            self.grid[path_point[1]][path_point[0]] = 9
        self.grid[self.start_position[1]][self.start_position[0]] = 6 # start position
        self.grid[self.target_position[1]][self.target_position[0]] = 7 # target position

        reversed_list = []
        for item in self.grid:
            reversed_list.insert(0, item)

        for row in reversed_list:
            row_str = ""
            for char in row:
                if char == 1:
                    row_str += "  "
                elif 0.01 <= char <= 0.99:
                    formatted = f"{(1-char):.1f}"
                    decimal_digit = formatted.split(".")[1]
                    row_str += f"{decimal_digit} "
                    #row_str += "* "
                elif char == 0:
                    row_str += "# "
                elif char == 6:
                    row_str += "O "
                elif char == 9:
                    row_str += "@ "
                elif char == 7:
                    row_str += "X "
            print(row_str)

        #print("path:", self.path)



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
    direction = agent.velocity.normalized()

    #for neighbor in grid[grid_cell_key]:
    for neighbor in all_neighbors:
        vec_to_neighbor = Vector(neighbor.location) - Vector(agent.location)
        len_to_neighbor = vec_to_neighbor.length

        if len_to_neighbor <= vision_radius:
            other_direction = vec_to_neighbor.normalized()
            dot_product = direction.dot(other_direction)

            if neighbor != agent and dot_product > half_field_of_view_cosine:# and 'enemy' in neighbor_agent.lower(): # if enemy in field of view
                visible_neighbors.append(neighbor)
            #
            #elif dot_product > half_field_of_view_cosine: # if neighbor in field of view
            #    visible_neighbors.append(neighbor_location)

    #if visible_enemies:
    #    update_agent_target(agent_name, visible_enemies)

    return visible_neighbors






def collision_detection(agent, collider, optimisation_rotation=None):
    if optimisation_rotation:  
        """this function is also used for the optimisation to find the best rotation so this is where the function is adapted for that"""
        cos_theta = math.cos(optimisation_rotation)
        sin_theta = math.sin(optimisation_rotation)
        direction = agent.velocity.normalized()
        x_rotated = direction.x * cos_theta - direction.y * sin_theta
        y_rotated = direction.x * sin_theta + direction.y * cos_theta
        agent_velocity = Vector((x_rotated, y_rotated)) * agent.schnelligkeit
    else:
        agent_velocity = agent.velocity


    relative_position = collider.location - agent.location
    relative_velocity = collider.velocity - agent_velocity

    a = relative_velocity.length_squared
    b = 2 * relative_position.dot(relative_velocity)
    c = relative_position.length_squared - agent.width**2

    if a == 0: # agents move at same velocity
        if relative_position.length <= agent.width:
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
        collision_point = Vector(agent.location) + (gugus * agent_velocity)
        #print(f"Collision point: {collision_point}")
        return collision_point
    else:
        return None






"""return the probability of a collision happening"""
def get_probability_of_collision(agent, collider, collision_point):
    target_to_agent = collision_point - Vector(agent.location)
    agent_time_to_collision = target_to_agent.dot(agent.velocity.normalized()) / agent.schnelligkeit

    target_to_collider = collision_point - Vector(collider.location)
    collider_time_to_collision = target_to_collider.dot(collider.velocity.normalized()) / collider.schnelligkeit

    if agent_time_to_collision < -0.3 and collider_time_to_collision < -0.3: # collision is in past
        return None
    
    agent_time_to_collision = abs(agent_time_to_collision)
    collider_time_to_collision = abs(collider_time_to_collision)

    probability_of_collision = min(collider_time_to_collision, agent_time_to_collision) / max(collider_time_to_collision, agent_time_to_collision)

    return probability_of_collision



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
def gget_collision_and_effort_cost(delta_velocity, delta_rotation, agent, collider):
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



def ggget_collision_and_effort_cost(delta_velocity, delta_rotation, agent, collider):
    # Weights for collision probability and effort
    w1 = 4  # Higher weight for collision avoidance
    w2 = 2  # Lower weight for effort

    # Predict future positions based on delta_velocity and delta_rotation
    future_agent_position = agent.location + delta_velocity * agent.velocity.normalized()
    future_collider_position = collider.location + collider.schnelligkeit * collider.velocity.normalized()

    # Calculate the distance between future positions
    distance_to_collider = (future_agent_position - future_collider_position).length

    # Define a collision threshold (e.g., sum of agent and collider radii)
    collision_threshold = agent.width + collider.width

    # Calculate collision probability based on distance
    if distance_to_collider < collision_threshold:
        collision_probability = 1.0  # Certain collision
    else:
        # Use an inverse relationship between distance and collision probability
        collision_point = collision_detection(agent, collider, delta_rotation)
        if collision_point:
            collision_probability = get_probability_of_collision(agent, collider, collision_point)
        else:
            collision_probability = 0
    #print(collision_probability)
    # Calculate effort based on delta_velocity and delta_rotation
    effort = get_agent_effort(delta_velocity, delta_rotation, agent)

    # Penalize large deviations from the original path
    original_direction = (Vector(agent.target_location) - agent.location).normalized()
    new_direction = (future_agent_position - agent.location).normalized()
    deviation_angle = original_direction.angle(new_direction)
    deviation_penalty = 1 + deviation_angle  # Penalize larger deviations

    # Adjust effort with deviation penalty
    effort *= deviation_penalty

    # Calculate total cost
    total_cost = (w1 * collision_probability) + (w2 * effort)

    return total_cost





def get_collision_and_effort_cost(delta_velocity, delta_rotation, agent, collider):
    # Weights for collision probability and effort
    w1 = 5  # Higher weight for collision avoidance
    w2 = 1  # Lower weight for effort

    # Predict future positions based on delta_velocity and delta_rotation
    future_agent_position = agent.location + delta_velocity * agent.velocity.normalized()
    future_collider_position = collider.location + collider.schnelligkeit * collider.velocity.normalized()

    # Calculate the distance between future positions
    distance_to_collider = (future_agent_position - future_collider_position).length

    # Define a collision threshold (e.g., sum of agent and collider radii)
    collision_threshold = agent.width + collider.width

    # Calculate collision probability based on distance
    if distance_to_collider < collision_threshold:
        collision_probability = 0.5  # Certain collision
    else:
        # Use an inverse exponential relationship between distance and collision probability
        collision_probability = math.exp(-distance_to_collider / collision_threshold) / 2

    num_samples = 50
    more_future_agent_position = agent.location + (agent.velocity.normalized() * 30 * agent.schnelligkeit)
    collision_probability += (global_heatmap.vector_values_in_grid(agent.location, more_future_agent_position, num_samples) / 2*num_samples)
    #print(agent.location, more_future_agent_position, "with probability:", collision_probability)
    # Calculate effort based on delta_velocity and delta_rotation
    effort = get_agent_effort(delta_velocity, delta_rotation, agent)

    # Normalize effort to a range of [0, 1] to prevent it from dominating the cost function
    max_effort = 100  # Define a reasonable maximum effort value
    normalized_effort = min(effort / max_effort, 1.0)

    # Penalize large deviations from the original path
    original_direction = (Vector(agent.target_location) - agent.location).normalized()
    new_direction = (future_agent_position - agent.location).normalized()
    deviation_angle = original_direction.angle(new_direction)
    deviation_penalty = 1 + (deviation_angle / math.pi)  # Normalize deviation to [1, 2]

    # Adjust effort with deviation penalty
    normalized_effort *= deviation_penalty

    # Calculate total cost
    total_cost = (w1 * collision_probability) + (w2 * normalized_effort)

    return total_cost



"""because i didnt want to use mathematical gradients to find the best rotation and velocity change i made a simple function that checks a given amount of rotations and then finds the best out of it"""
def ooptimisation_function(agent, collider): # does not work really
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
        collision_probability, effort = get_collision_and_effort_cost(agent.schnelligkeit, delta_rotation, agent, collider)
        #print("probability:", collision_probability)
        #print("--min effort:", min_effort)
        if collision_probability and collision_probability < min_collision_probability:
            #min_effort = (effort / collision_probability)
            min_collision_probability = collision_probability
            best_rotation = delta_rotation
                
    #print("stage 2:", best_rotation)

    return best_velocity, best_rotation




def optimisation_function(agent, collider):
    # Define ranges for rotation and velocity changes
    rotation_increment = math.radians(5)  # 5-degree steps
    velocity_increment = 0.1  # 0.1 m/s steps

    # Constraints
    max_rotation = math.radians(45)  # Maximum rotation (45 degrees)
    min_speed = 1.0  # Minimum walking speed (m/s)
    max_speed = 2.0  # Maximum walking speed (m/s)

    # Initialize best values
    best_rotation = 0
    best_velocity = agent.schnelligkeit  # Start with current speed
    min_cost = float('inf')

    # Grid search over rotation and velocity changes
    for delta_rotation in range(-int(max_rotation / rotation_increment), int(max_rotation / rotation_increment) + 1):
        delta_rotation_rad = delta_rotation * rotation_increment
        for delta_velocity in range(int((min_speed - agent.velocity.length) / velocity_increment),
                                   int((max_speed - agent.velocity.length) / velocity_increment) + 1):
            delta_velocity_magnitude = delta_velocity * velocity_increment
            new_velocity = agent.schnelligkeit + delta_velocity_magnitude

            # Calculate cost for this combination
            #print(new_velocity)
            cost = get_collision_and_effort_cost(new_velocity, delta_rotation_rad, agent, collider)

            # Update best values if this combination is better
            if cost < min_cost:
                min_cost = cost
                best_rotation = delta_rotation_rad
                best_velocity = new_velocity

    return best_rotation, best_velocity



def find_closest_target(current_location, path_coordinates, target):

    radius = 3 # in blender units

    in_radius = {}
    for point in path_coordinates:
        length = (Vector(current_location) - Vector(point)).length
        if length <= radius:
            in_radius[point] = (Vector(point) - Vector(target)).length

    smallest_len = float('inf')
    best_target = None
    for leng in in_radius:
        if in_radius[leng] < smallest_len:
            smallest_len = in_radius[leng]
            best_target = leng


    return best_target




grid_size = 100.0 # in blender units
cell_size = 0.1 # in blender units
heatmap_origin = (0, 0)  # Bottom-left corner of the heatmap in world coordinates

static_obstacle_names = [
        'molo1', 

]
static_dangerzone_names = [
        'Cube.001',

]
dangerzone_danger_values = {
    "Cube.001": 0.1,

}

static_obstacles = {}
for obj in bpy.data.objects:
    if obj.type == 'MESH' and obj.name in static_obstacle_names:
        static_obstacles[obj] = 1
    elif obj.type == 'MESH' and obj.name in static_dangerzone_names:
        static_obstacles[obj] = dangerzone_danger_values[obj.name]







### FUNCTIONS ###

#print("o bam")

#gugu.shortest_path_search(start_position, target_position)


"""the logic of the motion matching would have been housed here, for testing i only returned the opposite leg, so if the left leg just walked then the right will be next"""
def get_best_leg_animation(agent, next_rotation, next_velocity):
    if agent.active_feet[0]:
        agent.active_feet = [False, True]
        animation_name = "legs_R_walking"
    else:
        agent.active_feet = [True, False]
        animation_name = "legs_L_walking"
    
    return animation_name
        

"""here i would have put a function that chooses which is the best upper body animation"""
def get_best_arm_animation(next_rotation, next_velocity):
    animation_name = ""
    return animation_name


### SCRIPT-INITIATION ###
"""import animations saved in json files, in this case for simplicity in testing the two same ones"""
animation_links = {
    "legs_R_walking": "G:\Sync\documents\maturaarbeit\matura-presentation\code/full-walking-1.json",
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




agent_names = [
        "agent.000",
        "agent.001",
        "agent.002",
        "agent.003",
        "agent.004",
        "agent.005",
        "agent.006",
        "agent.007",
        "agent.008",
        "agent.009",
        "agent.010",
        "agent.011",
        "agent.012",
        "agent.013",
        "agent.014",
        "agent.015",
        "agent.016",
        "agent.017",
        "agent.018",
        "agent.019",
        "agent.020",
        "agent.021",
        "agent.022",
        "agent.023",
        "agent.024",
        "agent.025",
        "agent.026",
        "agent.027",
        "agent.028",
        "agent.029",
        "agent.030",
        "agent.031",
        "agent.032",
        "agent.033",
        "agent.034",
        "agent.035",
        "agent.036",
        "agent.037",
        "agent.038",
        "agent.039",
        "agent.040",
        "agent.041",
        "agent.042",
        "agent.043",
        "agent.044",
        "agent.045",
        "agent.046",
        "agent.047",
        "agent.048",
        "agent.049",
        "agent.050",
        "agent.051",
        "agent.052",
        "agent.053",
        "agent.054",
        "agent.055",
        "agent.056",
        "agent.057",
        "agent.058",
        "agent.059",
]
target_names = [
        "target.000",
        "target.001",
        "target.002",
        "target.003",
        "target.004",
        "target.005",
        "target.006",
        "target.007",
        "target.008",
        "target.009",
        "target.010",
        "target.011",
        "target.012",
        "target.013",
        "target.014",
        "target.015",
        "target.016",
        "target.017",
        "target.018",
        "target.019",
        "target.020",
        "target.021",
        "target.022",
        "target.023",
        "target.024",
        "target.025",
        "target.026",
        "target.027",
        "target.028",
        "target.029",
        "target.030",
        "target.031",
        "target.032",
        "target.033",
        "target.034",
        "target.035",
        "target.036",
        "target.037",
        "target.038",
        "target.039",
        "target.040",
        "target.041",
        "target.042",
        "target.043",
        "target.044",
        "target.045",
        "target.046",
        "target.047",
        "target.048",
        "target.049",
        "target.050",
        "target.051",
        "target.052",
        "target.053",
        "target.054",
        "target.055",
        "target.056",
        "target.057",
        "target.058",
        "target.059",
]
agent_speed = [
    0.05,
    0.06,
    0.05,
    0.04,
    0.05,
    0.06,
    0.05,
    0.04,
    0.05,
    0.06,
    0.05,
    0.04,
    0.05,
    0.06,
    0.05,
    0.04,
    0.05,
    0.06,
    0.05,
    0.04,
    0.05,
    0.06,
    0.05,
    0.04,
    0.05,
    0.06,
    0.05,
    0.04,
    0.05,
    0.06,
    0.05,
    0.04,
    0.05,
    0.06,
    0.05,
    0.04,
    0.05,
    0.06,
    0.05,
    0.04,
    0.05,
    0.06,
    0.05,
    0.04,
    0.05,
    0.06,
    0.05,
    0.04,
    0.05,
    0.06,
    0.05,
    0.04,
    0.05,
    0.06,
    0.05,
    0.04,
    0.05,
    0.06,
    0.05,
    0.04,
    0.05
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


late_start_agent_list = []


if bpy.context.mode != 'OBJECT':
    bpy.ops.object.mode_set(mode='OBJECT')

num_agents = len(agent_names)
shader_growth_index = 6/num_agents
shader_index = 0

global_heatmap = Heatmap(grid_size, cell_size, heatmap_origin, static_obstacles)


for index, agent_name in enumerate(agent_names):
    target_name = target_names[index]
    agent_loc = bpy.data.objects.get(agent_name).location
    agent_location = (agent_loc.x, agent_loc.y)

    target_loc = bpy.data.objects.get(target_name).location
    target_vec = (target_loc.x, target_loc.y)
    direction = (Vector(target_vec) - Vector(agent_location)).normalized()
    #print(agent_location, target_vec)

    


    
    #bpy.data.objects.get(target_list[-1]).location


    path_of_agent = global_heatmap.shortest_path_search(agent_location, target_vec)
    #print("sigma sigma:", path_of_agent)
    #gugu.draw_grid()


    agents_dictionary[agent_name] = Agent(agent_name, direction, agent_location, agent_speed[index], target_vec, 70, 1, path_of_agent)
    first_location = (agent_loc.x, agent_loc.y, 0)
    agent_coords_list = [first_location]
    all_path_coords.append(agent_coords_list)
    targets_dictionary[target_name] = target_name
    late_start_agent_list.append(agent_name)



    shader_index += shader_growth_index

    """small if else tree i made to give each agent path a different color to differ them better from each other"""
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
simulation_end_frame = 450

#R_step_coords.extend(local_path) # only for testing










"""start of the animation loop, that runs from start to end frame"""
for frame in range(simulation_start_frame, simulation_end_frame + 1):
    bpy.context.scene.frame_set(frame)
    bpy.context.view_layer.update()
    #print("------------------------------------")
    print(f"###[Frame {frame}]")
    #time_start = time.time()

    """go through all the agents on each frame to update them"""
    for index, agent_name in enumerate(agents_dictionary):
        agent = agents_dictionary[agent_name]
        agent_bpy = bpy.data.objects.get(agent_name)
        neighbors = objects_in_field_of_view(agent)

        if agent.at_target:
            print("obama", frame)
            #del agents_dictionary[agent_name] 
            # fucking shit does not work
            continue

        if frame < 24:
            if frame == agent.global_animation_end_frame:
                late_start_agent_list.remove(agent_name)


        if agent_name in late_start_agent_list:
            continue
        

        for neighbor in neighbors: 
            """go through each neighbor and check which one is the closest one"""
            #collision_probability, effort = get_collision_and_effort_cost(agent.velocity, None, agent, neighbor)
            #collision_point = collision_detection(agent, neighbor)
            #if collision_point:
            #    chance = get_probability_of_collision(agent, neighbor, collision_point)
            #print(f"{agent.name} will collide with {neighbor.name} with a chance of {collision_probability}")

            length_agent_to_neighbor = (Vector(agent.location) - Vector(neighbor.location)).length
            #print("length agent to neighbor:", length_agent_to_neighbor)
            if length_agent_to_neighbor < agent.min_length_to_neighbor:
                agent.min_length_to_neighbor = length_agent_to_neighbor
                agent.update_nearest_neighbor(neighbor)


        if agent.nearest_neighbor: 
            """if there is a nearest neighbor then find the rotation and velocity change and update them"""
            #print(f"{agent.name} nearest neighbor is {agent.nearest_neighbor.name}")
            best_velocity, best_rotation = optimisation_function(agent, agent.nearest_neighbor)
            if best_rotation:
                agent.write_best_rotation(best_rotation)
            else:
                agent.write_best_rotation(0)

        else:
            agent.write_best_rotation(0)
            #print(f"seems like there is no nearest_neighbor for {agent.name}")


        ### find next animation
        next_rotation = 1
        next_velocity = 1

        leg_animation_name = "legs_R_walking"#get_best_leg_animation(agent, next_rotation, next_velocity)
        
        leg_animation_object = animations_dictionary[leg_animation_name]

        leg_targets = {}

        active_foot_target = 1

        if frame == agent.global_animation_end_frame: # not anymore in animation cycle
            agent.set_leg_animation(leg_animation_object, frame, active_foot_target)

        agent.find_closest_target()
        agent.update_frame_data()
        agent.rotate_all_bones(frame)

    #print("---Calculations: %.4f sec" % (time.time() - time_start))

        

      
print() 
"""empty print to have a space between each execution of the script in the terminal"""
