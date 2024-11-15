#!/usr/bin/env python3

import os
import subprocess
import yaml

"""
This script uses a pre-generated .avl file based on the provided YAML configuration.
Ensure the .avl file exists and matches the vehicle_name specified in the YAML file.
"""

def main():
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

    if not all([plane_name, frame_type, reference_area, wing_span, reference_point]):
        raise ValueError("Missing required configuration in YAML file.")

    print(f"Vehicle Name: {plane_name}")
    print(f"Frame Type: {frame_type}")
    print(f"Reference Area: {reference_area}")
    print(f"Wing Span: {wing_span}")
    print(f"Reference Point: {reference_point}")

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

        # avl_out_parse.main(
        #     plane_name=plane_name,
        #     frame_type=frame_type,
        #     AR=wing_span ** 2 / reference_area,  # Aspect Ratio
        #     mac=(2 / 3) * (reference_area / wing_span),  # Mean Aerodynamic Chord
        #     ref_pt_x=reference_point.get("X"),
        #     ref_pt_y=reference_point.get("Y"),
        #     ref_pt_z=reference_point.get("Z"),
        #     num_ctrl_surfaces=None,  # Optional; if needed
        #     area=reference_area,
        #     ctrl_surface_order=None,  # Optional; if needed
        #     avl_path=os.getcwd()
        # )

    except TypeError as e:
        raise TypeError(
            "The function `avl_out_parse.main` received unexpected arguments. "
            "Check the function signature in `avl_out_parse` to ensure it matches this script. "
            f"Details: {e}"
        )

    # Move output files to a dedicated folder
    output_dir = os.path.join(os.getcwd(), plane_name)
    os.makedirs(output_dir, exist_ok=True)

    for file in os.listdir('.'):
        if file.startswith(plane_name):
            os.rename(file, os.path.join(output_dir, file))

    print(f"Output files saved in: {output_dir}")

    # Visualize PostScript plot
    ps_file = os.path.join(output_dir, f"{plane_name}.ps")
    if os.path.exists(ps_file):
        os.system(f'evince {ps_file}')
    else:
        print("PostScript file not found. Skipping visualization.")

if __name__ == "__main__":
    main()
