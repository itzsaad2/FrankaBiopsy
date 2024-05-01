def extract_xyz_from_file(filename):
    # Initialize a list to hold the extracted XYZ coordinates
    xyz_coordinates = []

    # Open the file for reading
    with open(filename, 'r') as file:
        lines = file.readlines()

    # Process the file lines to extract XYZ coordinates
    for i in range(0, len(lines)):  # Adjust the step size if necessary based on file structure
        if "Pose" in lines[i]:
            # Extract XYZ coordinates from the 4th column of the matrix
            x = float(lines[i + 1].split()[-1])
            y = float(lines[i + 2].split()[-1])
            z = float(lines[i + 3].split()[-1])
            xyz_coordinates.append([x, y, z])

    print(xyz_coordinates)

    # # Print the extracted XYZ coordinates
    # for pose, (x, y, z) in enumerate(xyz_coordinates, start=1):
    #     print(f"Pose {pose}: [{x}, {y}, {z}]")

# Example usage
filename = "temp.txt"  # Make sure this is the path to your file
extract_xyz_from_file(filename)
