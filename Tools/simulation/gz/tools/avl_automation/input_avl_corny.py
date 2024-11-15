#!/usr/bin/env python3

import os
import subprocess
import yaml
import argparse

"""
This script uses a pre-generated .avl file based on the provided YAML configuration.
Ensure the .avl file exists and matches the vehicle_name specified in the YAML file.
"""

def main():
    user = os.environ.get('USER')
    # This will find Avl on a users machine.
    for root, dirs, _ in os.walk(f'/home/{user}/'):
        if "Avl" in dirs:
            target_directory_path = os.path.join(root, "Avl")
            break
    parent_directory_path = os.path.dirname(target_directory_path)
    filedir = f'{parent_directory_path}/'
    print(filedir)

    parser = argparse.ArgumentParser()
    parser.add_argument("--yaml_file", help="Path to input yaml file.")
    parser.add_argument("--avl_path", default=filedir, help="Provide an absolute AVL path. If this argument is passed, AVL will be moved there and the files will adjust their paths accordingly.")
    inputs = parser.parse_args()

    # Load YAML configuration
    yaml_file = "input_s1.yml"  # Specify your YAML file here
    if not os.path.exists(yaml_file):
        raise FileNotFoundError(f"YAML file {yaml_file} not found.")

    with open(yaml_file, 'r') as file:
        config = yaml.safe_load(file)

    # Extract information from YAML
    plane_name = config.get("vehicle_name")
    frame_type = config.get("frame_type")
    reference_area = config.get("reference_area")
    wing_span = config.get("wing_span")
    reference_point = config.get("reference_point")
    ref_pt_x = reference_point.get("X")
    ref_pt_y = reference_point.get("Y")
    ref_pt_z = reference_point.get("Z")
    num_ctrl_surfaces = 0
    ctrl_surface_order = []

    # Validate required fields
    missing_fields = []
    if plane_name is None:
        missing_fields.append("vehicle_name")
    if frame_type is None:
        missing_fields.append("frame_type")
    if reference_area is None:
        missing_fields.append("reference_area")
    if wing_span is None:
        missing_fields.append("wing_span")
    if ref_pt_x is None:
        missing_fields.append("reference_point.X")
    if ref_pt_y is None:
        missing_fields.append("reference_point.Y")
    if ref_pt_z is None:
        missing_fields.append("reference_point.Z")

    if missing_fields:
        raise ValueError(f"Missing required fields in the YAML configuration: {', '.join(missing_fields)}")

    print(f"Vehicle Name: {plane_name}")
    print(f"Frame Type: {frame_type}")
    print(f"Reference Area: {reference_area}")
    print(f"Wing Span: {wing_span}")
    print(f"Reference Point: X={ref_pt_x}, Y={ref_pt_y}, Z={ref_pt_z}")

    # Check if the corresponding .avl file exists
    avl_file = f"{plane_name}.avl"
    if not os.path.exists(avl_file):
        raise FileNotFoundError(f"The .avl file {avl_file} does not exist in the current directory.")

    print(f"Using AVL file: {avl_file}")

    # Run AVL using the existing .avl file
    try:
        subprocess.run(f'./process.sh {plane_name}', shell=True, check=True)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Error running AVL: {e}")

    # Parse AVL output (Placeholder: Update as per your `avl_out_parse` logic)
    try:
        import avl_out_parse  # Ensure avl_out_parse is correctly implemented

        AR = str((float(wing_span)*float(wing_span))/float(reference_area))
        mac = str((2/3)*(float(reference_area)/float(wing_span)))

        avl_out_parse.main(plane_name,frame_type,AR,mac,ref_pt_x,ref_pt_y,ref_pt_z,num_ctrl_surfaces,reference_area,ctrl_surface_order,inputs.avl_path)

    except TypeError as e:
        raise TypeError(
            "The function `avl_out_parse.main` received unexpected arguments. "
            "Check the function signature in `avl_out_parse` to ensure it matches this script. "
            f"Details: {e}"
        )

    # Move output files to a dedicated folder
    output_dir = os.path.join(os.getcwd(), plane_name)
    os.makedirs(output_dir, exist_ok=True)

    # Visualize PostScript plot
    ps_file = os.path.join(output_dir, f"{plane_name}.ps")
    if os.path.exists(ps_file):
        os.system(f'evince {ps_file}')
    else:
        print("PostScript file not found. Skipping visualization.")

if __name__ == "__main__":
    main()
