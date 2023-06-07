import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def plot_vector(point_a, point_b, origin, r):
    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Calculate the vector components
    vector = [point_b[0] - origin[0], point_b[1] - origin[1], point_b[2] - origin[2]]

    # Set plot limits
    min_coords = [0,0,0]
    max_coords = [1,1,1]
    ax.set_xlim(min_coords[0], max_coords[0])
    ax.set_ylim(min_coords[1], max_coords[1])
    ax.set_zlim(min_coords[2], max_coords[2])

    # Plot the vector
    ax.quiver(origin[0], origin[1], origin[2], vector[0], vector[1], vector[2], color='blue', label='Vector (origin to B)')
    ax.quiver(origin[0], origin[1], origin[2], point_a[0] - origin[0], point_a[1] - origin[1], point_a[2] - origin[2], color='red', label='Vector (origin to A)')

    # Add length annotations
    ax.text(origin[0], origin[1], origin[2], f'r={r}', color='blue', fontsize=10, ha='center')
    ax.text(point_a[0], point_a[1], point_a[2], f'r={r}', color='red', fontsize=10, ha='center')

    # Set labels and legend
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # Show the plot
    plt.show()


def calculate_coordinates(theta, phi, r,origin):
    # Convert theta and phi from degrees to radians
    theta = math.radians(theta)
    phi = math.radians(phi)

    # Calculate the Cartesian coordinates
    x = (r * math.cos(theta) * math.sin(phi))+origin[0]
    y = r * math.sin(theta) * math.sin(phi)+origin[1]
    z = r * math.cos(phi)+origin[2]

    # Return the new coordinates as a tuple
    return x, y, z



def calculate_point_on_circle(radius, center_x, center_y, angle_degrees):
    angle_radians = math.radians(angle_degrees)
    x = center_x + radius * math.cos(angle_radians)
    y = center_y + radius * math.sin(angle_radians)
    return x, y

# radius = 0.2  # Radius of the circle
# center_x = 0.3  # x-coordinate of the center
# center_y = 0  # y-coordinate of the center

# x_values = []
# y_values = []

# for angle_degrees in range(0, 360, 10):
#     x, y = calculate_point_on_circle(radius, center_x, center_y, angle_degrees)
#     x_values.append(x)
#     y_values.append(y)
#     print(f"At {angle_degrees} degrees: ({x}, {y})")

# # Plotting the points
# plt.plot(x_values, y_values, 'ro')
# plt.axis('equal')  # Set equal aspect ratio for x and y axes
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Points on a Circle')
# plt.grid(True)
# plt.show()
# import math
# import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calculate_point_on_sphere(radius, center_x, center_y, center_z, phi_degrees, theta_degrees):
    phi = math.radians(phi_degrees)
    theta = math.radians(theta_degrees)
    x = center_x + radius * math.sin(phi) * math.cos(theta)
    y = center_y + radius * math.sin(phi) * math.sin(theta)
    z = center_z + radius * math.cos(phi)
    return x, y, z

radius = 0.3  # Radius of the sphere
center_x = 0.3+radius  # x-coordinate of the center
center_y = 0  # y-coordinate of the center
center_z = 0.4  # z-coordinate of the center

phi_values = []
theta_values = []
x_values = []
y_values = []
z_values = []

for phi_degrees in range(0, 360, 10):
    for theta_degrees in range(0, 360, 10):
        phi_values.append(phi_degrees)
        theta_values.append(theta_degrees)
        x, y, z = calculate_point_on_sphere(radius, center_x, center_y, center_z, phi_degrees, theta_degrees)
        x_values.append(x)
        y_values.append(y)
        z_values.append(z)
        print(f"At (phi: {phi_degrees}, theta: {theta_degrees}) degrees: ({x}, {y}, {z})")

# Plotting the points
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_values, y_values, z_values, c='r', marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Points on a Sphere')
plt.show()


# # Example usage
# theta = 17.5
# x=0.3
# y=0
# z=0.42
# phi = -90
# r = 0.3
# origin = [r, 0, 0]
# a = [0,0,0]

# b = calculate_coordinates(theta, phi, r,origin)
# print(b)

# plot_vector(a, b, origin, r)
