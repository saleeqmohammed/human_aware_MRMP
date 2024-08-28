import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def is_overlapping(rect1, rect2):
    xmin1, xmax1, ymin1, ymax1 = rect1
    xmin2, xmax2, ymin2, ymax2 = rect2
    return not (xmax1 <= xmin2 or xmax2 <= xmin1 or ymax1 <= ymin2 or ymax2 <= ymin1)

def generate_obstacles(num_obstacles, x_range=(-10, 10), y_range=(-10, 10), size_range=(1, 5)):
    obstacles = []
    max_attempts = 1000

    for _ in range(num_obstacles):
        attempts = 0
        placed = False
        while not placed and attempts < max_attempts:
            width = np.random.uniform(size_range[0], size_range[1])
            height = np.random.uniform(size_range[0], size_range[1])
            xmin = np.random.uniform(x_range[0], x_range[1] - width)
            xmax = xmin + width
            ymin = np.random.uniform(y_range[0], y_range[1] - height)
            ymax = ymin + height

            new_obstacle = [xmin, xmax, ymin, ymax]
            if all(not is_overlapping(new_obstacle, obs) for obs in obstacles):
                obstacles.append(new_obstacle)
                placed = True
            attempts += 1
        
        if attempts >= max_attempts:
            print("Warning: Unable to place obstacle after many attempts.")

    return obstacles

def get_checkerboard_lines(obstacles, interval=0.5):
    lines = []
    for obstacle in obstacles:
        xmin, xmax, ymin, ymax = obstacle
        # Generate vertical lines
        x = xmin
        while x <= xmax:
            lines.append([x, x, ymin, ymax])
            x += interval
        # Generate horizontal lines
        y = ymin
        while y <= ymax:
            lines.append([xmin, xmax, y, y])
            y += interval
    return lines

def plot_obstacles_with_checkerboard(obstacles, lines, x_range=(-10, 10), y_range=(-10, 10)):
    fig, ax = plt.subplots()
    
    # Set plot limits
    ax.set_xlim(x_range)
    ax.set_ylim(y_range)
    
    # Add obstacles to the plot
    for obstacle in obstacles:
        xmin, xmax, ymin, ymax = obstacle
        rect = patches.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, linewidth=1, edgecolor='r', facecolor='none')
        ax.add_patch(rect)
    
    # Add checkerboard lines to the plot
    for line in lines:
        x1, x2, y1, y2 = line
        ax.plot([x1, x2], [y1, y2], color='b', linestyle='--', linewidth=0.5)
    
    ax.set_xlabel('X-axis (meters)')
    ax.set_ylabel('Y-axis (meters)')
    ax.set_title('Obstacles with Checkerboard Pattern')
    
    plt.grid(True)
    plt.show()

# Parameters
num_obstacles = 5
obstacles = generate_obstacles(num_obstacles)
checkerboard_lines = get_checkerboard_lines(obstacles, interval=0.5)

# Print the checkerboard lines for each obstacle
for i, line in enumerate(checkerboard_lines):
    print(f"Line {i + 1}: {line}")

# Plot the obstacles with checkerboard pattern
plot_obstacles_with_checkerboard(obstacles, checkerboard_lines)
