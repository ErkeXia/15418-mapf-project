import sys
from pathlib import Path

def parse_custom_yaml(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()

    obstacles = []
    agents = []
    dims = [0, 0]
    
    mode = None # 'obstacles' or 'agents'
    current_agent = {}

    for line in lines:
        line = line.strip()
        if not line: continue

        if line.startswith('dimensions:'):
            # Format: dimensions: [32, 32]
            parts = line.split('[')[1].split(']')[0].split(',')
            dims = [int(p) for p in parts]
        
        elif line.startswith('obstacles:'):
            mode = 'obstacles'
        elif line.startswith('agents:'):
            mode = 'agents'
        
        # Parse Obstacles
        elif mode == 'obstacles' and line.startswith('-'):
            # Format: - [0, 0]
            parts = line.split('[')[1].split(']')[0].split(',')
            obstacles.append((int(parts[0]), int(parts[1])))

        # Parse Agents
        elif mode == 'agents':
            if line.startswith('- name:'):
                current_agent = {}
            if 'start:' in line:
                parts = line.split('[')[1].split(']')[0].split(',')
                current_agent['start'] = (int(parts[0]), int(parts[1]))
            if 'goal:' in line:
                parts = line.split('[')[1].split(']')[0].split(',')
                current_agent['goal'] = (int(parts[0]), int(parts[1]))
                agents.append(current_agent)

    return dims, obstacles, agents

def convert_yaml(filename):
    src = Path(filename)
    try:
        rel = src.relative_to("../../dataset")
    except ValueError:
        rel = src
    print(rel)  # this still prints to the real stdout

    dest_file = Path(".") / rel
    dest_file = dest_file.with_suffix(".txt")
    dest_file.parent.mkdir(parents=True, exist_ok=True)

    try:
        dims, obstacles, agents = parse_custom_yaml(filename)
    except FileNotFoundError:
        sys.stderr.write(f"Error: {filename} not found.\n")
        return

    with open(dest_file, "w") as f:
        print(f"{dims[0]} {dims[1]}", file=f)

        print(len(obstacles), file=f)
        for obs in obstacles:
            print(f"{obs[0]} {obs[1]}", file=f)

        print(len(agents), file=f)
        for i, a in enumerate(agents):
            print(f"{i} {a['start'][0]} {a['start'][1]} {a['goal'][0]} {a['goal'][1]}", file=f)
# convert_yaml("../../dataset/w_woundedcoast/4-0.yaml")

import glob
import os

folder = "../../dataset/w_woundedcoast"

# all .yaml files in that folder
for yaml_file in glob.glob(os.path.join(folder, "*.yaml")):
    # print("Converting:", yaml_file)
    convert_yaml(yaml_file)