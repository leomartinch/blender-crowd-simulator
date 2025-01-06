import bpy
import numpy as np
import heapq
import math
import time
from mathutils import Vector

import gpu
from gpu_extras.batch import batch_for_shader


"""CHECKLIST:"""
"""
- All collision objects in the static collision list need to have their origin point (apply location)

"""


"""PROBLEMS:"""
"""
- The grids seem to be working fine
"""



### FUNCTIONS ###

def is_cell_in_grid(x_position, y_position):  # is cell real (within the grid boundaries)
    return 0 <= x_position < grid_columns and 0 <= y_position < grid_rows

def is_cell_on_target(x_position, y_position, target):  # Check if a cell matches the target
    return (x_position, y_position) == target

def get_line_length(x_position, y_position, target_position):  # Calculate heuristic using Euclidean distance
    return math.sqrt((x_position - target_position[0])**2 + (y_position - target_position[1])**2)

def convert_global_coords_to_local(global_x, global_y):
    local_x = global_x / cell_size + (cell_size / 2) # point is at center of cell
    local_y = global_y / cell_size + (cell_size / 2)
    return int(local_x), int(local_y)

def convert_local_coords_to_global(current_x, current_y):
    global_x = current_x * cell_size + (cell_size / 2) # point is at center of cell
    global_y = current_y * cell_size + (cell_size / 2)
    global_x = global_x + heatmap_origin[0]
    global_y = global_y + heatmap_origin[1]
    return global_x, global_y

def reconstruct_path(parent_map, target):
    original_path = [] # for testing (draw map)
    path = []
    current_x, current_y = target
    while (current_x, current_y) in parent_map:
        original_path.append((current_x, current_y))
        global_x, global_y = convert_local_coords_to_global(current_x, current_y)
        path.append((global_x, global_y))
        current_x, current_y = parent_map[(current_x, current_y)]
    path.reverse()
    original_path.reverse()
    return path, original_path

# Implementation of the A* search algorithm
def shortest_path_search(grid, start_position, target_position):
    open_cells = []
    heapq.heappush(open_cells, (0, 0, start_position[0], start_position[1]))  # (f_cost, g_cost, x_position, y_position)
    parent_map = {}
    cost_from_start = {start_position: 0}  # Tracks g_cost for each cell
    closed_set = set() # cells that have already been visited

    movement_directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    while open_cells:
        current_f_cost, current_g_cost, current_x, current_y = heapq.heappop(open_cells)

        if is_cell_on_target(current_x, current_y, target_position):
            return reconstruct_path(parent_map, target_position)

        closed_set.add((current_x, current_y)) # Mark the cell as explored
        is_direction_blocked = {
            (0,1): False,
            (0,-1): False,
            (1,0): False,
            (-1,0): False
        }

        # Explore all valid and unblocked neighboring cells
        for delta_x, delta_y in movement_directions:
            neighbor_x = current_x + delta_x
            neighbor_y = current_y + delta_y

            # if neighbor cell is not in grid, if is blocked or has already been visited, then continue
            if not is_cell_in_grid(neighbor_x, neighbor_y) or not grid[neighbor_y][neighbor_x] > 0 or (neighbor_x, neighbor_y) in closed_set:
                if delta_x == 0 or delta_y == 0:
                    is_direction_blocked[(delta_x, delta_y)] = True
                continue

            cell_cost = grid[neighbor_y][neighbor_x] # get the cell value from the heatmap grid
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
    
                f_cost = estimated_g_cost + get_line_length(neighbor_x, neighbor_y, target_position)

                heapq.heappush(open_cells, (f_cost, estimated_g_cost, neighbor_x, neighbor_y))
                parent_map[(neighbor_x, neighbor_y)] = (current_x, current_y)
                print(is_direction_blocked)

    print("Failed to find the target.")
    return None, None # second none only for testing

def get_cell_value(grid_x_position, grid_y_position, obstacle_list):
    x_cell_center = heatmap_origin[0] + (grid_x_position + 0.5) * cell_size
    y_cell_center = heatmap_origin[1] + (grid_y_position + 0.5) * cell_size
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


####################################
### VARIABLES ###

grid_size = 4.0 # in blender units
cell_size = 0.25 # in blender units
heatmap_origin = (0, 0)  # Bottom-left corner of the heatmap in world coordinates

static_obstacle_names = [
        'Circle', 
        'Cube.002'
]
static_dangerzone_names = [
        'Cube.001',
        'Cube.004'
]
dangerzone_danger_values = {
    "Cube.001": 0.4,
    "Cube.004": 0.1
}

#start_position = (0, 0)
#target_position = (4, 10)
global_start_position = bpy.data.objects.get("start").location
global_target_position = bpy.data.objects.get("target").location


### SCRIPT ###

print()
print("#########################################################")

number_of_cells = int(grid_size / cell_size) # along one axis
grid_rows = number_of_cells
grid_columns = number_of_cells
simulation_container = 1

global_start_position = global_start_position - Vector((heatmap_origin[0], heatmap_origin[1], 0))
global_target_position = global_target_position - Vector((heatmap_origin[0], heatmap_origin[1], 0))
print("Start:", global_start_position)
print("target:", global_target_position)
start_position = convert_global_coords_to_local(global_start_position.x, global_start_position.y)
target_position = convert_global_coords_to_local(global_target_position.x, global_target_position.y)
print("Start:", start_position)
print("target:", target_position)

heatmap = []
for row in range(grid_rows):
    current_row = [1] * grid_columns
    heatmap.append(current_row)

static_obstacles = {}

for obj in bpy.data.objects:
    if obj.type == 'MESH' and obj.name in static_obstacle_names:
        static_obstacles[obj] = 1
    elif obj.type == 'MESH' and obj.name in static_dangerzone_names:
        static_obstacles[obj] = dangerzone_danger_values[obj.name]

# create the heatmap
for current_row in range(grid_rows):
    for current_column in range(grid_columns):
        current_cell_value = get_cell_value(current_column, current_row, static_obstacles)
        heatmap[current_row][current_column] = current_cell_value


time_start = time.time()
path, original_path = shortest_path_search(heatmap, start_position, target_position)
print("My Script Finished: %.4f sec" % (time.time() - time_start))


# to render scene in terminal
for path_point in original_path:
    heatmap[path_point[1]][path_point[0]] = 9
heatmap[start_position[1]][start_position[0]] = 6 # start position
heatmap[target_position[1]][target_position[0]] = 7 # target position

reversed_list = []
for item in heatmap:
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

print("path:", path)



### MODAL-OPERATOR ###

"""
def create_keyframes(name, coordinates):
    max_frame = len(coordinates)
    for frame in range(1, max_frame + 1):
        obj = bpy.data.objects.get(name)
        obj.location = coordinates[frame - 1]
        obj.keyframe_insert(data_path="location", frame=frame)
"""

batches = []
shaders_rgb = []
shader = gpu.shader.from_builtin('UNIFORM_COLOR')

batch = batch_for_shader(shader, 'LINE_STRIP', {"pos": path})
batches.append(batch)
shaders_rgb.append((1,1,0,1))


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
        
        #if event.type == 'K': # add keyframes
        #    for index, name in enumerate(agents_dictionary):
        #        create_keyframes(name, all_path_coords[index])
        #    return {'RUNNING_MODAL'}

        return {'PASS_THROUGH'}

    def invoke(self, context, event):
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

def register():
    bpy.utils.register_class(ModalOperator)

def unregister():
    bpy.utils.unregister_class(ModalOperator)

register()
bpy.ops.wm.modal_operator('INVOKE_DEFAULT')
