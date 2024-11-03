import numpy as np
import os
import scipy.stats as stats

num_boxes = 60
box_size = 0.03
rgba = "0.9 0 0 1"
bounds = {"x": 0.35, "y":0.35, "z":1}	# Stack region boundaries
origin = {"x": -0.7, "y":0, "z":0.2}	# Stack position
sigma = bounds["z"]	# Truncated normal distribution variance

table_width, table_length = bounds["x"]/2 + box_size, bounds["y"]/2 + box_size
table_height = (origin["z"] - box_size)/2
table_rgba = "0 0 0 0.5"
table_x, table_y, table_z = origin["x"], origin["y"], table_height

# Generate box positions in MJCF format
def create_positions(num_boxes, origin, bounds, box_size):
    positions = []
    fails = 0
    while (len(positions) < num_boxes):
        valid = True
        # Truncated normal distribution for z position to have more boxes spawn at the bottom of the stack
        z = round(origin["z"] + float(stats.truncnorm.rvs(0, bounds["z"]/sigma, loc=0, scale=sigma, size=1)), 2)
        # Shrink the x and y bounds closer to the top of the stack
        bound_x = bounds["x"] * (1 - (z-origin["z"])/bounds["z"])
        bound_y = bounds["y"] * (1 - (z-origin["z"])/bounds["z"])
        # uniform random x and y positions
        x = round(origin["x"] + np.random.uniform(-bound_x/2, bound_x/2), 2)
        y = round(origin["y"] + np.random.uniform(-bound_y/2, bound_y/2), 2)
        
		# If bottom is too full, go back to uniform random distribution
        if fails > 10000:
            x = round(origin["x"] + np.random.uniform(-bounds["x"]/2, bounds["x"]/2), 2)
            y = round(origin["y"] + np.random.uniform(-bounds["y"]/2, bounds["y"]/2), 2)
            z = round(origin["z"] + np.random.uniform(0, bounds["z"]), 2)
            
        # Make sure boxes aren't intersecting
        for pos in positions:
            dx = x - pos[0]
            dy = y - pos[1]
            dz = z - pos[2]
            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            min_dist = 2*np.sqrt(3*(box_size**2))
            if (distance < min_dist):
                valid = False
                if z - origin["z"] < 4*box_size:
                    fails += 1
        if valid:
            positions.append([x, y, z])

	# Put in string format for MJCF
    pos_strings = [f"{pos[0]} {pos[1]} {pos[2]}" for pos in positions]

    return pos_strings

# Add table to the scene 
def add_header(xml, table_pos, table_size, rgba, box_size):
    pos = f"{table_pos[0]} {table_pos[1]} {table_pos[2]}"
    size = f"{table_size[0]} {table_size[1]} {table_size[2]}"
    wall_1_pos, wall_1_size = f"{table_size[0]} 0 {table_size[2]+box_size}"	, f"0.005 {table_size[1]} {box_size}"
    wall_2_pos, wall_2_size = f"{-table_size[0]} 0 {table_size[2]+box_size}"	, f"0.005 {table_size[0]} {box_size}"
    wall_3_pos, wall_3_size = f"0 {table_size[1]} {table_size[2]+box_size}"	, f"{table_size[0]} 0.005 {box_size}"
    wall_4_pos, wall_4_size = f"0 {-table_size[1]} {table_size[2]+box_size}"	, f"{table_size[0]} 0.005 {box_size}"
    header = f"""\
<mujoco>
    <worldbody>
		<body pos="{pos}">
            <geom type="box" size="{size}" rgba="{rgba}"/>
            <geom pos="{wall_1_pos}" type="box" size="{wall_1_size}" rgba="{rgba}"/>
            <geom pos="{wall_2_pos}" type="box" size="{wall_2_size}" rgba="{rgba}"/>
            <geom pos="{wall_3_pos}" type="box" size="{wall_3_size}" rgba="{rgba}"/>
            <geom pos="{wall_4_pos}" type="box" size="{wall_4_size}" rgba="{rgba}"/>
        </body>
"""
    xml.write(header)

# Close out MJCF file
def add_footer(xml):
    footer = """\
    </worldbody>
</mujoco>\
"""
    xml.write(footer)

# Add a box to the scene given parameters
def add_box(xml, pos, box_size, rgba):
    size = f"{box_size} {box_size} {box_size}"
    entry = f"""\
        <body pos="{pos}">
            <joint type="free"/>
            <geom type="box" solimp=".999 .999 .0001" solref=".0001 1" size="{size}" rgba="{rgba}"/>
        </body>
"""
    xml.write(entry)

positions = create_positions(num_boxes, origin, bounds, box_size)

# Open XML file to write to
xml = open(os.path.abspath(os.path.join(os.path.dirname( __file__ ), "boxes.xml")), "w")

table_pos = [table_x, table_y, table_z]
table_size = [table_width, table_length, table_height]
# Generate MJCF tags and write to XML
add_header(xml, table_pos, table_size, table_rgba, box_size)
for pos in positions:
    add_box(xml, pos, box_size, rgba)
add_footer(xml)

xml.close()