import bpy
import json

"""CHECKLIST:"""
"""
- check if all the bones are correct in the armature (retarget animation to universal armature)
- check if armature name is correct
- check if start and end frames are set correctly
- set filename

- set the animation specific adaptive and non adaptive bones
DEFAULTS:
- leg_animation = ["R_Up_Leg", "R_Leg", "L_Up_Leg", "L_Leg"], ["Hips", "Spine_0", "Spine_1", "Spine_2"]
- arm_animation = ["Hips", "Spine_0", "Spine_1", "Spine_2", "Neck", "Head", "R_Shoulder", "L_Shoulder", "R_Arm", "R_Fore_Arm", "L_Arm", "L_Fore_Arm"]

"""


### VARIABLES ###

# for the arms we change the direction point to be behind, for the knees we make him be in front
animation_adaptive_bones = ["R_Up_Leg", "R_Leg", "L_Up_Leg", "L_Leg"]
animation_non_adaptive_bones = ["Hips", "Spine_0", "Spine_1", "Spine_2", "Neck", "Head", "R_Shoulder", "L_Shoulder", "R_Arm", "R_Fore_Arm", "L_Arm", "L_Fore_Arm"]


armature_name = "Universal_Armature"
start_frame = 1
end_frame = 34
filename = "oborin"

### FUNCTIONS ###

def export_bone_data(filepath):
    armature = bpy.data.objects.get(armature_name)

    animation_data = {
        "end_frame": (end_frame - start_frame),
        "adaptive_bone_names": animation_adaptive_bones,
        "non_adaptive_bone_names": animation_non_adaptive_bones,
        "bones": []
    }

    obj = bpy.data.objects.get(armature_name)
    for bone_name in animation_adaptive_bones: 
        bone = armature.pose.bones[bone_name]
        coordinates_list = []

        bone_data = {
            "name": bone_name,
            "coordinates": None
        }

        for frame in range(start_frame, end_frame + 1):
            bpy.context.scene.frame_set(frame)
            depsgraph = bpy.context.evaluated_depsgraph_get()
            eval_obj = obj.evaluated_get(depsgraph)
            tail_local = bone.tail
            bone_tail_position = eval_obj.matrix_world @ tail_local

            coordinates_list.append([bone_tail_position.x, bone_tail_position.y, bone_tail_position.z])
        bone_data["coordinates"] = coordinates_list
        animation_data["bones"].append(bone_data)

    for bone_name in animation_non_adaptive_bones:
        bone = armature.pose.bones[bone_name]

        rotation_list = []
        bone_data = {
            "name": bone_name,
            "rotations": None
        }

        for frame in range(start_frame, end_frame + 1):
            bpy.context.scene.frame_set(frame)
            rotation = bone.rotation_quaternion
            rotation_list.append([rotation.w, rotation.x, rotation.y, rotation.z])
        bone_data["rotations"] = rotation_list
        animation_data["bones"].append(bone_data)

    bone = armature.pose.bones["Hips"]
    hip_up_down_locations = []
    hip_front_speed = []
    bone_data = {
        "name": "Hip_Motion",
        "up_down": None,
        "front": None
    }
    last_speed_value = None
    # such a mess because of blenders bone axis, x=x, y=z, z=y 
    for frame in range(start_frame, end_frame + 1):
        bpy.context.scene.frame_set(frame)
        location = bone.location.copy()
        if last_speed_value:
            speed = location.z - last_speed_value
        else:
            speed = 0.08
        last_speed_value = location.z
        hip_front_speed.append(speed)
        location = [location.x, location.y, 0]
        hip_up_down_locations.append(location)
        
    bone_data["up_down"] = hip_up_down_locations
    bone_data["front"] = hip_front_speed
    animation_data["bones"].append(bone_data)


    with open(filepath, 'w') as file:
        json.dump(animation_data, file, indent=None)


### SCRIPT ###
export_bone_data(f"G:/Softwares/{filename}.json")
