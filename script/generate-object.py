import numpy as np

def generate_figure8_xy_offsets(size_meters, precision):
    waypoints = []
    a = size_meters / 2.0
    for i in range(precision):
        t = 2 * np.pi * i / precision
        x_offset = a * np.sin(t)
        y_offset = a * np.sin(t) * np.cos(t)
        waypoints.append((x_offset, y_offset))
    return waypoints

size_meters = 20.0
precision = 12



xy_points = generate_figure8_xy_offsets(size_meters, precision)

# 4. Create the textured model SDF strings
cloned_models_sdf = ""

for i, (x, y) in enumerate(xy_points):
    # Z is slightly above ground (0.02) to prevent Z-fighting with ground_plane
    # The albedo_map points to "texture.png" in the same directory
    model_xml = f"""
    <model name="target_mark_{i}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0.02 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 1.0 0.01</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <pbr>
              <metal>
                <albedo_map>texture.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>
    """
    cloned_models_sdf += model_xml
print(cloned_models_sdf)