import numpy as np
import os
import scipy.stats as stats

box_size = 0.03
rgba = "0.9 0 0 1"
base_dim = 4	# Num of boxes along each side of the pyramid
spacing = 0.03	# Space between each box
fuzz = 0.01
origin = {"x": -0.6, "y":-0.11, "z":0.25}	# Position of first box in stack
center_x = origin["x"]-((base_dim-1)/2 * (spacing+2*box_size)) +(2*box_size+spacing)
center_y = origin["y"]
center = {"x": center_x, "y": center_y, "z": origin["z"]}	# Position of center of stack

table_width, table_length = base_dim*(2*box_size+spacing)/2 + fuzz, 6*box_size
table_height = (center["z"] - box_size)/2
table_rgba = "0.1 1 0.1 0.5"
table_x, table_y, table_z = center["x"], center["y"], table_height

# Generate box positions in MJCF format
def create_positions(center, base_dim, box_size, spacing, fuzz):
	positions = []
	
	z = center["z"]
	
	for layer_size in range(base_dim, 0, -1):
		# Determine the x, y starting points to center the layer around the center
		offset = (layer_size - 1) / 2 * (spacing + 2*box_size)
		# offset = (layer_size - 1) * (spacing + 2*box_size)
		x_values = np.linspace(center["x"]+offset, center["x"]-offset, layer_size)
		
		# Generate the grid points for this layer
		for x in x_values:
			x_ = round(x + np.random.uniform(-fuzz,fuzz), 3)
			y_ = center["y"]         
			positions.append([x_, y_, z])
		
		# Move up for the next layer
		z += 4*box_size

	# Put in string format for MJCF
	pos_strings = [f"{pos[0]} {pos[1]} {pos[2]}" for pos in positions]

	return pos_strings

# Add table to the scene 
def add_header(xml, table_pos, table_size, rgba, box_size):
	pos = f"{table_pos[0]} {table_pos[1]} {table_pos[2]}"
	size = f"{table_size[0]} {table_size[1]} {table_size[2]}"
	header = f"""\
<mujoco>
	<worldbody>
		<body pos="{pos}">
			<geom type="box" size="{size}" rgba="{rgba}"/>
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

positions = create_positions(center, base_dim, box_size, spacing, fuzz)

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