import bpy
import math
import time
import mathutils
from mathutils import Vector

import gpu
from gpu_extras.batch import batch_for_shader



"""WORKS AS IS"""

"""
--- WHAT WORKS:
the calculations are superfast, they take 0.010 sec


--- WHAT HAS TO BE FIXED:






"""


R_leg_coords_target = []
L_leg_coords_target = []
hips_coordinate_target = []

R_arm_coords_target = []
L_arm_coords_target =[]


R_arm_pos = [(0,0,0), (0,0,0), (0,0,0)]
L_arm_pos = [(0,0,0), (0,0,0), (0,0,0)]
R_leg_pos = [(0,0,0), (0,0,0), (0,0,0)]
L_leg_pos = [(0,0,0), (0,0,0), (0,0,0)]








controll_point_names = [
        ['R_arm_target', 'R_arm_home', 'R_arm_direction', 'R_arm_pos', 'R_Arm', 'R_Fore_Arm', 'NONE'],  # R_arm
        ['L_arm_target', 'L_arm_home', 'L_arm_direction', 'L_arm_pos', 'L_Arm', 'L_Fore_Arm', 'NONE'],  # L_arm
        ['R_leg_target', 'R_leg_home', 'R_leg_direction', 'R_leg_pos', 'R_Up_Leg', 'R_Leg', 'R_leg_coords_target'],  # R_leg
        ['L_leg_target', 'L_leg_home', 'L_leg_direction', 'L_leg_pos', 'L_Up_Leg', 'L_Leg', 'L_leg_coords_target']   # L_leg
]

# dictionary with the lists of the positions of each ik part
pos_list_dict = {
    "R_arm_pos": R_arm_pos,
    "L_arm_pos": L_arm_pos,
    "R_leg_pos": R_leg_pos,
    "L_leg_pos": L_leg_pos,
}

targets_dict = {
    "R_arm_coords_target": R_arm_coords_target,
    "L_arm_coords_target": L_arm_coords_target,
    "R_leg_coords_target": R_leg_coords_target,
    "L_leg_coords_target": L_leg_coords_target
}

#############################

### FUNCTIONS ###

def rotate_bone_to_target(bone, target):
    head_to_target = target - bone.head

    direction = head_to_target.normalized()

    current_direction = (bone.tail - bone.head).normalized()
    rotation_axis = current_direction.cross(direction)
    rotation_angle = current_direction.angle(direction)

    rotation_matrix = mathutils.Matrix.Rotation(rotation_angle, 4, rotation_axis)
    bone.matrix = rotation_matrix @ bone.matrix


def character_inverse_kinematics(current_point, target_point):
    for i, sublist in enumerate(controll_point_names):
        if current_point in sublist:
            index_sublist = i

    # arms
    if index_sublist < 1.5:
        side_a = side_a_arm
        side_c = side_c_arm
    # legs
    elif index_sublist > 1.5:
        side_a = side_a_leg
        side_c = side_c_leg

    extra_point = bpy.data.objects.get(controll_point_names[index_sublist][2]).location
    home_point = armature.pose.bones[controll_point_names[index_sublist][4]].head
    
    bone_side_c = armature.pose.bones[controll_point_names[index_sublist][4]] # get side c bone
    bone_side_a = armature.pose.bones[controll_point_names[index_sublist][5]] # get side a bone

    side_b = (home_point - target_point).length

    current_pos_list = pos_list_dict[controll_point_names[index_sublist][3]]

    home_target_vec = (home_point - target_point) # home point to target point
    home_extra_vec = (home_point - extra_point) # home point to extra point (to form a plane)

    if (abs(side_c - side_a)) <= side_b <= (side_c + side_a): # if target inside bounds
        axis_of_rotation = home_target_vec.cross(home_extra_vec)
    
        alpha_top = side_b**2 + side_c**2 - side_a**2
        alpha_bot = 2 * side_b * side_c
        alpha = math.acos(alpha_top / alpha_bot)
        alpha = -(math.radians(180) - alpha)

        quaternion_vec = mathutils.Quaternion(axis_of_rotation.normalized(), alpha)

        c_side_vec = side_c * home_target_vec.normalized()

        point_B = quaternion_vec @ c_side_vec
        point_B = point_B + home_point

        current_pos_list[0] = home_point
        current_pos_list[1] = point_B
        current_pos_list[2] = target_point

    else:
        home_target_vec = (target_point - home_point)
        home_extra_vec = (extra_point - home_point)

        c_side_vec = side_c * home_target_vec.normalized()
        c_side_vec += home_point

        if side_b < (abs(side_c - side_a)): # if target inside bounds
            cside_target_vec = (c_side_vec - target_point)
            a_side_target = side_a * cside_target_vec.normalized()
        else:
            a_side_target = (side_a + side_c) * home_target_vec.normalized()

        a_side_target += home_point

        current_pos_list[0] = home_point
        current_pos_list[1] = c_side_vec
        current_pos_list[2] = a_side_target

        point_B = c_side_vec
        target_point = a_side_target

    rotate_bone_to_target(bone_side_c, point_B)
    bpy.context.view_layer.update()
    rotate_bone_to_target(bone_side_a, target_point)
    bpy.context.view_layer.update()

### FUNCTIONS ###

def retarget_control_points(home, target, bezier_vectors):
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



### VARIABLES ###

armature = bpy.data.objects["Universal_Rig"]



animation_length = 24

R_targets = [
    "R_target1",
    "R_target2",
    "R_target3",
    "R_target4"
]

L_targets = [
    "L_target1",
    "L_target2",
    "L_target3",
    "L_target4"
]

hips_targets = [
    "hip_target1",
    "hip_target2",
    "hip_target3",
    "hip_target4"
]

target_dic = {
    "R_leg": R_targets,
    "L_leg": L_targets,
    "hips": hips_targets
}



coords_list_dic = {
    "R_leg": R_leg_coords_target,
    "L_leg": L_leg_coords_target,
    "hips": hips_coordinate_target
}


bezier_vectors = [Vector((0.0, 0.0, 0.0)), Vector((0.0, -0.18000608682632446, 0.1318873018026352)), Vector((0.0, -0.3945286273956299, 0.197976753115654)), Vector((0.0, -0.6145839095115662, 0.23324468731880188)), Vector((0.0, -0.8346391916275024, 0.2685126066207886)), Vector((0.0, -0.8794701099395752, 0.14181092381477356)), Vector((0.0, -0.90260910987854, 0.0))]

foot_bezier_vec = [Vector((0.0, 0.0, 0.0)), Vector((0.15840864181518555, -0.004654914140701294, 0.24591614305973053)), Vector((0.519545316696167, -0.010119594633579254, 0.2040533423423767)), Vector((1.0203156471252441, 0.003473609685897827, 0.0861692950129509)), Vector((1.5210859775543213, 0.01706681400537491, -0.031714752316474915)), Vector((1.741530418395996, 0.004816815257072449, 0.13751499354839325)), Vector((1.841078519821167, 0.0, 0.0))]

hips_bezier_vec = [Vector((0.0, 0.0, 0.0)), Vector((0.6732587218284607, 0.023146092891693115, -0.10721433162689209)), Vector((0.7991347908973694, 0.02969646453857422, 0.012429952621459961)), Vector((0.9971082806587219, 0.03605186939239502, 0.012429952621459961)), Vector((1.1950818300247192, 0.04240727424621582, 0.012429952621459961)), Vector((1.3664488792419434, 0.027531206607818604, -0.16209757328033447)), Vector((1.843002200126648, 0.0, 0.0))]


bezier_vec_dic = {
    "R_leg": foot_bezier_vec,
    "L_leg": foot_bezier_vec,
    "hips": hips_bezier_vec
}

### SCRIPT ###

side_a_leg = (armature.pose.bones["L_Leg"].tail - armature.pose.bones["L_Leg"].head).length
side_c_leg = (armature.pose.bones["L_Up_Leg"].head - armature.pose.bones["L_Up_Leg"].tail).length

side_a_arm = (armature.pose.bones["L_Fore_Arm"].head - armature.pose.bones["L_Fore_Arm"].tail).length
side_c_arm = (armature.pose.bones["L_Arm"].head - armature.pose.bones["L_Arm"].tail).length




for body_part in target_dic:
    print(body_part)
    target_coords = []    
    for target in target_dic[body_part]:
        target_location = bpy.data.objects[target].location
        target_coords.append(target_location) 


    for index in range(len(target_coords) - 1):
        current_home = target_coords[index]
        current_target = target_coords[index + 1]

        bezier_control_points = retarget_control_points(current_home, current_target, bezier_vec_dic[body_part])
        local_path = create_animation_path(animation_length, bezier_control_points)
        coords_list_dic[body_part].extend(local_path)







# Define the Shaders for Armature once
R_arm_shader = gpu.shader.from_builtin('UNIFORM_COLOR')
L_arm_shader = gpu.shader.from_builtin('UNIFORM_COLOR')
R_leg_shader = gpu.shader.from_builtin('UNIFORM_COLOR')
L_leg_shader = gpu.shader.from_builtin('UNIFORM_COLOR')
#head_shader = gpu.shader.from_builtin('UNIFORM_COLOR')
#spine_shader = gpu.shader.from_builtin('UNIFORM_COLOR')
L_path_sh = gpu.shader.from_builtin('UNIFORM_COLOR')
R_path_sh = gpu.shader.from_builtin('UNIFORM_COLOR')
hip_path_sh = gpu.shader.from_builtin('UNIFORM_COLOR')

def update_draw(self, context):

    # Define Color of shaders
    R_arm_shader.uniform_float("color", (1, 1, 1, 1))
    L_arm_shader.uniform_float("color", (1, 1, 1, 1))
    R_leg_shader.uniform_float("color", (1, 0.5, 0, 1))
    L_leg_shader.uniform_float("color", (1, 0.5, 0, 1))
    #head_shader.uniform_float("color", (1, 0, 0, 1))
    #spine_shader.uniform_float("color", (1, 0, 0, 1))
    L_path_sh.uniform_float("color", (1, 0, 0, 1))
    R_path_sh.uniform_float("color", (1, 0, 0, 1))
    hip_path_sh.uniform_float("color", (1, 0, 0, 1))

    # Define the batch for each part
    R_arm_batch = batch_for_shader(R_arm_shader, 'LINE_STRIP', {"pos": R_arm_pos})
    L_arm_batch = batch_for_shader(L_arm_shader, 'LINE_STRIP', {"pos": L_arm_pos})
    R_leg_batch = batch_for_shader(R_leg_shader, 'LINE_STRIP', {"pos": R_leg_pos})
    L_leg_batch = batch_for_shader(L_leg_shader, 'LINE_STRIP', {"pos": L_leg_pos})
    #head_batch = batch_for_shader(head_shader, 'LINE_STRIP', {"pos": head_pos})
    #spine_batch = batch_for_shader(spine_shader, 'LINE_STRIP', {"pos": spine_pos})
    L_path = batch_for_shader(L_path_sh, 'LINE_STRIP', {"pos": L_leg_coords_target})
    R_path = batch_for_shader(R_path_sh, 'LINE_STRIP', {"pos": R_leg_coords_target})
    hip_path = batch_for_shader(hip_path_sh, 'LINE_STRIP', {"pos": hips_coordinate_target})

    # Draw the batch
    R_arm_batch.draw(R_arm_shader)
    L_arm_batch.draw(L_arm_shader)
    R_leg_batch.draw(R_leg_shader)
    L_leg_batch.draw(L_leg_shader)
    #head_batch.draw(head_shader)
    #spine_batch.draw(spine_shader)
    L_path.draw(L_path_sh)
    R_path.draw(R_path_sh)
    hip_path.draw(hip_path_sh)



class ModalOperator(bpy.types.Operator):
    bl_idname = "wm.modal_operator"
    bl_label = "Modal Operator"
    
    _timer = None
    last_location = None
    last_frame = None
    
    def modal(self, context, event):
        scene = bpy.context.scene

        if event.type == 'TIMER':
            # Get the current frame
            current_frame = scene.frame_current

            #current_location = bpy.context.object.location.copy()
            if self.last_frame is None or current_frame != self.last_frame:
                    


                #time_start = time.time()
                bpy.context.view_layer.objects.active = armature
                bpy.ops.object.mode_set(mode='POSE')
                boneg = armature.pose.bones["Hips"]
                boneg.location.x = hips_coordinate_target[current_frame - 1][0]
                boneg.location.z = -hips_coordinate_target[current_frame - 1][1]
                boneg.location.y = hips_coordinate_target[current_frame - 1][2] -1
                #bpy.context.view_layer.update()

                for index, value in enumerate(controll_point_names):
                    if controll_point_names[index][6] == "NONE":
                        target_point = bpy.data.objects.get(controll_point_names[index][0]).location

                    elif controll_point_names[index][1] == "R_leg_home":
                        current_coor_list = targets_dict[controll_point_names[index][6]]
                        target_point = current_coor_list[current_frame - 1]

                    elif controll_point_names[index][1] == "L_leg_home":
                        current_coor_list = targets_dict[controll_point_names[index][6]]
                        target_point = current_coor_list[current_frame + 13]
                    
                    #else:
                    #    current_coor_list = targets_dict[controll_point_names[index][6]]
                    #    target_point = current_coor_list[current_frame - 1]

                    character_inverse_kinematics(controll_point_names[index][0], target_point)
                

                # set hip position
                
                #armature.location.x = 1
                    
                #target_point = coords_target[current_frame - 1]

                #character_inverse_kinematics(controll_point, target_point)

                bpy.ops.object.mode_set(mode='OBJECT')
                #print("Bone Calculation took: %.4f sec" % (time.time() - time_start))
                    
                self.last_frame = current_frame

                context.area.tag_redraw() # update the viewport drawing of the lines
        
        # if ESC key is pressed cancel script    
        if event.type == 'ESC':
            wm = context.window_manager
            wm.event_timer_remove(self._timer)
            bpy.types.SpaceView3D.draw_handler_remove(self._draw_handler, 'WINDOW')
            print("programm terminated")
            print()
            return {'CANCELLED'}
        
        '''
        # if Q key is pressed rotate direction left
        if event.type == 'Q':
            wm = context.window_manager
            wm.event_timer_remove(self._timer)
            bpy.types.SpaceView3D.draw_handler_remove(self._draw_handler, 'WINDOW')
            return {'CANCELLED'}
        
        # if W key is pressed rotate direction left
        if event.type == 'W' and event.value == 'PRESS': # press checks that it has to be pressed meaning it can only work once while pressed
            print("W is pressed")
            
            return {'RUNNING_MODAL'}
        '''
                    
        return {'PASS_THROUGH'}

    def execute(self, context):
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.01, window=context.window)
        wm.modal_handler_add(self)

        # Add the draw handler
        self._draw_handler = bpy.types.SpaceView3D.draw_handler_add(update_draw, (self, context), 'WINDOW', 'POST_VIEW')

        return {'RUNNING_MODAL'}

    def cancel(self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
        bpy.types.SpaceView3D.draw_handler_remove(self._draw_handler, 'WINDOW')
        return {'CANCELLED'}
    
    def invoke(self, context, event):
        return self.execute(context)

def register():
    bpy.utils.register_class(ModalOperator)

def unregister():
    bpy.utils.unregister_class(ModalOperator)

if __name__ == "__main__":
    register()

    # Test call
    bpy.ops.wm.modal_operator()
