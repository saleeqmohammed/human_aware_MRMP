import numpy as np
import matplotlib.pyplot as plt

def generate_polygon(centroid, side_lengths):
    num_sides = len(side_lengths)
    
    if num_sides < 3:
        raise ValueError("A polygon must have at least 3 sides")
    
    # Centroid coordinates
    cx, cy = centroid
    
    # Calculate the angle between each vertex (in radians)
    angles = np.linspace(0, 2 * np.pi, num_sides + 1)[:-1]
    
    # Initialize the list of vertices
    vertices = []
    
    # Initial angle (can start from any angle, here we start from 0)
    current_angle = 0
    
    # Calculate each vertex based on the centroid, angle, and side length
    for i in range(num_sides):
        length = side_lengths[i % len(side_lengths)]  # If side_lengths has fewer elements, it will repeat
        # Compute the position of the vertex
        x = cx + length * np.cos(current_angle)
        y = cy + length * np.sin(current_angle)
        vertices.append((x, y))
        
        # Update the angle for the next vertex
        if i < num_sides - 1:
            next_length = side_lengths[(i + 1) % len(side_lengths)]
            # Compute the angle for the next vertex using the cosine rule
            angle_increment = np.arccos((length**2 + next_length**2 - length**2) / (2 * length * next_length))
            current_angle += angle_increment
    
    # Form the polygon by connecting vertices
    polygon_edges = []
    for i in range(num_sides):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % num_sides]
        polygon_edges.append([x1, x2, y1, y2])
    
    return vertices, polygon_edges

def plot_polygon(vertices, centroid):
    # Unzip the list of vertices into two lists: one for x-coordinates and one for y-coordinates
    x_coords, y_coords = zip(*vertices)
    
    # Create the plot
    plt.figure(figsize=(6, 6))
    
    # Plot the edges of the polygon
    plt.plot(x_coords + (x_coords[0],), y_coords + (y_coords[0],), 'b-', marker='o', label='Polygon Edges')
    
    # Plot the centroid
    plt.plot(centroid[0], centroid[1], 'ro', label='Centroid')
    
    # Set plot limits and title
    plt.xlim(min(x_coords) - 1, max(x_coords) + 1)
    plt.ylim(min(y_coords) - 1, max(y_coords) + 1)
    plt.title('Generated Polygon')
    
    # Add grid and legend
    plt.grid(True)
    plt.legend()
    
    # Show the plot
    plt.show()

# Example usage
centroid = (0, 0)
side_lengths = [5, 5, 5, 5, 5]  # For a regular pentagon

vertices, polygon_edges = generate_polygon(centroid, side_lengths)
plot_polygon(vertices, centroid)
