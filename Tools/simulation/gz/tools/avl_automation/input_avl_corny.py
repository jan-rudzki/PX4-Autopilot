import argparse
import yaml
import subprocess
import os

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Run AVL using a pre-generated .avl file based on a YAML configuration.")
    parser.add_argument("--yaml_file", required=True, help="Path to the input YAML file containing the configuration.")
    parser.add_argument("--avl_path", default=os.getcwd(), help="Path to the AVL installation directory. Defaults to the current directory.")
    inputs = parser.parse_args()

    # Load YAML configuration
    yaml_file = inputs.yaml_file
    if not os.path.exists(yaml_file):
        raise FileNotFoundError(f"YAML file {yaml_file} not found.")

    with open(yaml_file, 'r') as file:
        config = yaml.safe_load(file)

    # Extract information from YAML
    plane_name = config.get("vehicle_name")
    frame_type = config.get("frame_type")
    reference_area = config.get("reference_area")
    wing_span = config.get("wing_span")
    reference_point = config.get("reference_point", {})
    ref_pt_x = reference_point.get("X")
    ref_pt_y = reference_point.get("Y")
    ref_pt_z = reference_point.get("Z")

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

    # Check if AVL file exists
    avl_file = f"{plane_name}.avl"
    if not os.path.exists(avl_file):
        raise FileNotFoundError(f"The .avl file {avl_file} does not exist in the current directory.")

    print(f"Using AVL file: {avl_file}")

    # Run AVL process
    try:
        subprocess.run(f'./process.sh {plane_name}', shell=True, check=True)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Error running AVL: {e}")

    # AVL output parsing logic (placeholder: replace with your own parsing logic)
    AR = str((float(wing_span) * float(wing_span)) / float(reference_area))
    mac = str((2 / 3) * (float(reference_area) / float(wing_span)))

    # Example function call to avl_out_parse.main()
    try:
        import avl_out_parse  # Ensure avl_out_parse module is implemented correctly
        avl_out_parse.main(
            plane_name=plane_name,
            frame_type=frame_type,
            AR=AR,
            mac=mac,
            ref_pt_x=ref_pt_x,
            ref_pt_y=ref_pt_y,
            ref_pt_z=ref_pt_z,
            num_ctrl_surfaces=0,  # Example; replace with actual value if needed
            area=reference_area,
            ctrl_surface_order=[],  # Example; replace with actual control surface data
            avl_path=inputs.avl_path
        )
    except Exception as e:
        raise RuntimeError(f"Error in avl_out_parse.main(): {e}")

    # Organize output files
    output_dir = os.path.join(os.getcwd(), plane_name)
    os.makedirs(output_dir, exist_ok=True)
    for file in os.listdir('.'):
        if file.startswith(plane_name):
            os.rename(file, os.path.join(output_dir, file))

    print(f"Output files saved in: {output_dir}")

    # Visualize output (e.g., PostScript plot)
    ps_file = os.path.join(output_dir, f"{plane_name}.ps")
    if os.path.exists(ps_file):
        os.system(f'evince {ps_file}')
    else:
        print("PostScript file not found. Skipping visualization.")

if __name__ == "__main__":
    main()
