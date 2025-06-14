import numpy as np

def generate_centered_danger_map(pgm_out="danger.pgm", yaml_out="danger.yaml"):
    canvas_size = 384
    # Original figure size
    fig_width, fig_height = 129, 116

    # Create full unknown canvas
    canvas = np.full((canvas_size, canvas_size), 205, dtype=np.uint8)  # Unknown

    # Create original figure (danger levels)
    figure = np.full((fig_height, fig_width), 254, dtype=np.uint8)  # No danger (white)

    # Define danger zones in figure coordinates (similar to previous)
    # Let's make three zones horizontally:
    # Left third: max danger (black, 0)
    # Middle third: medium danger (128)
    # Right third: no danger (254)
    one_third = fig_width // 3
    figure[:, :one_third] = 0       # max danger
    figure[:, one_third:2*one_third] = 128  # medium danger
    figure[:, 2*one_third:] = 254   # no danger

    # Calculate offsets to center figure on canvas
    offset_x = (canvas_size - fig_width) // 2
    offset_y = (canvas_size - fig_height) // 2

    # Paste figure onto canvas
    canvas[offset_y:offset_y+fig_height, offset_x:offset_x+fig_width] = figure

    # Write PGM file
    with open(pgm_out, "wb") as f:
        header = f"P5\n{canvas_size} {canvas_size}\n255\n"
        f.write(header.encode())
        f.write(canvas.tobytes())

    # Write YAML file for ROS
    resolution = 0.05
    with open(yaml_out, "w") as f:
        f.write(f"image: {pgm_out}\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin: [-{canvas_size*resolution/2}, -{canvas_size*resolution/2}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")

if __name__ == "__main__":
    generate_centered_danger_map()

