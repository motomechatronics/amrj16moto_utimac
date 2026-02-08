# Unit 3 — J16MOTO Robot Modeling with URDF & Xacro

**Estimated time:** ~3 hours  
**Robot:** AMRJ16 (MOTO Mechatronics fleet)  
**Source file to use:** `amrj16.xacro` (provided by MOTO)  
**Outcome:** You will visualize AMRJ16 in RViz with correct base frames and a working TF tree. You’ll also use **Joint State Publisher GUI** to move continuous joints (wheels/casters) during validation.

> **Mindset (MOTO-style):** We are not “drawing” a robot. We are *describing a real machine* so ROS can reason about it.

---

## Learning goals
By the end of this unit you will be able to:

- Create a clean ROS 2 *description* package (`urdf/`, `launch/`, `rviz/`, `models/`).
- Use **Xacro** as the single source of truth for AMRJ16’s structure.
- Publish `/robot_description` and TF using `robot_state_publisher`.
- Configure RViz correctly (RobotModel + TF + fixed frame).
- Validate non-fixed joints using **joint_state_publisher_gui**.

### Please note that the Xacro section is missing in this tutorial and must be added by you.
Follow the course to understand how to do it. Hint: create a j16moto_main.xacro that calls: lidar.xacro, chassis.xacro, etc.
---

## 0. Before you start

- Close any ROS nodes from previous labs (especially RViz, robot_state_publisher, joint_state_publisher_gui).
- Make sure your ROS 2 workspace exists at `~/ros2_ws`.

> **Tip:** If RViz behaves oddly after repeated runs, check running nodes:
>
> ```bash
> ros2 node list
> ```

---

## 1. Create the AMRJ16 description package

### 1.1 Create a ROS 2 package

We’ll follow the standard “*_description*” pattern used in real projects:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake amrj16_description --dependencies urdf xacro
```

### 1.2 Create standard folders

```bash
cd ~/ros2_ws/src/amrj16_description
mkdir -p urdf launch rviz models/meshes
```

Folder purpose:

- `urdf/` — URDF/Xacro source files
- `launch/` — launch files for publishing/visualizing
- `rviz/` — saved RViz configurations
- `models/meshes/` — DAE/STL meshes for visuals/collisions

> [IMAGE: Package folder tree — show `urdf/`, `launch/`, `rviz/`, `models/meshes/`]

---

## 2. Add `amrj16.xacro` to the package

Copy the provided file into your package:

```bash
cp /path/to/amrj16.xacro ~/ros2_ws/src/amrj16_description/urdf/amrj16.xacro
```

> **Important:** In the AMRJ16 Xacro, meshes are referenced using:
>
> `package://amrj16_description/models/meshes/...`
>
> That is why we named the package **amrj16_description**.

---

## 3. Understand the “TF-first” core of AMRJ16

Open:

- `~/ros2_ws/src/amrj16_description/urdf/amrj16.xacro`

This unit focuses on the foundation:

### 3.1 `base_footprint` and `base_link`

In AMRs, we typically separate:

- **base_footprint**: navigation-friendly frame on the ground plane (no roll/pitch)
- **base_link**: body frame used as the parent of the physical robot structure

In the file you will find a fixed joint similar to:

```xml
<link name="base_footprint"/>
<link name="base_link"/>

<joint name="base_link_base_footprintjoint" type="fixed">
  <origin rpy="0 0 0" xyz="0 0 0.103" />
  <parent link="base_footprint" />
  <child link="base_link" />
</joint>
```

**Interpretation (real robot thinking):**
- `base_link` is **0.103 m above** `base_footprint`.
- This is not cosmetic—this number encodes real geometry.

> [IMAGE: RViz view of `base_footprint` and `base_link` — separated along Z by ~10 cm]

### 3.2 Chassis link (visual vs collision vs inertial)

AMRJ16’s chassis is defined with:
- a **DAE** mesh for visual
- a **low-res STL** for collision
- real-world **mass** and **inertial matrix**

You will find a section like:

```xml
<link name="chassis">
  <visual>
    <geometry>
      <mesh filename="package://amrj16_description/models/meshes/amrj16_chassis_utimac_v4.dae"/>
    </geometry>
  </visual>

  <collision>
    <geometry>
      <mesh filename="package://amrj16_description/models/meshes/amrj16_chassis_utimac_v4_low_res.stl"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="34.2"/>
    <origin rpy="0 0 0" xyz="-0.1055 0 -0.0199"/>
    <inertia ixx="10.164712" ixy="0.021460923" ixz="-0.0060724714"
             iyy="3.4403550" iyz="1.7125796" izz="12.572506"/>
  </inertial>
</link>

<joint name="base_link_joint" type="fixed">
  <parent link="base_link"/>
  <child link="chassis"/>
</joint>
```

**MOTO modeling rule of thumb:**
- Visual is for humans.
- Collision and inertial are for physics/simulation (even if we’re not simulating yet, we keep the model “production-grade”).

> [IMAGE: RViz view — chassis mesh visible, TF enabled]

### 3.3 Driving wheels (continuous joints)

AMRJ16 uses continuous joints for driving wheels. You will find blocks similar to:

```xml
<joint name="chassis_right_wheel_joint" type="continuous">
  <origin xyz="0 -0.5456 0" rpy="-1.5707 0 0"/>
  <parent link="chassis"/>
  <child link="right_wheel"/>
  <axis xyz="0 0 1"/>
</joint>
```

and similarly for the left wheel:

```xml
<joint name="chassis_left_wheel_joint" type="continuous">
  <origin xyz="0 0.5456 0" rpy="-1.5707 0 0"/>
  <parent link="chassis"/>
  <child link="left_wheel"/>
  <axis xyz="0 0 1"/>
</joint>
```

**What to learn here:**
- `rpy` often compensates for mesh orientation.
- `axis` must match the joint’s intended rotation after applying `rpy`.

> [IMAGE: RViz view — chassis + left/right driving wheels]

### 3.4 Front caster chains (how AMRJ16 “cheats physics” without cheating your TF tree)

Casters are where many students discover that “a wheel” is rarely just **one joint**.

On AMRJ16 each front caster is modeled as a **two-link chain**:

1. **Caster hub**: rotates around **Z** (swivel / yaw).
2. **Caster wheel**: rotates around its **rolling axis** (here also expressed as a continuous joint, but with a rotated joint frame).

Let’s walk through the **front-right caster** exactly as it appears in `amrj16.xacro`.

#### 3.4.1 The caster hub link

```xml
<link name="front_right_caster_hub">
  <visual>
    <geometry>
      <mesh filename="package://amrj16_description/models/meshes/amrj16_caster_hub.dae"/>
    </geometry>
  </visual>

  <inertial>
    <mass value="0.140"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <inertia ixx="0.00006" ixy="0.000000359" ixz="0.000014875"
             iyy="0.000059805" iyz="-0.00000077564" izz="0.000042536"/>
  </inertial>
</link>
```

**What you should notice (engineer brain ON):**
- The hub uses a **mesh** for visual (good for humans) and keeps collision *commented out* (faster sim / fewer headaches).
- Inertial parameters exist (mass + inertia tensor). You *can* simulate without them, but Gazebo will remind you with strange dynamics.

> [IMAGE: Close-up of the caster hub mesh and its TF frame in RViz]

#### 3.4.2 Hub joint: `chassis → hub` (continuous yaw)

```xml
<joint name="chassis_front_right_caster_hub_joint" type="continuous">
  <origin xyz="-0.255 -0.538 -0.055" rpy="0 0 1.5707"/>
  <parent link="chassis"/>
  <child link="front_right_caster_hub"/>
  <axis xyz="0 0 1"/>
  <dynamics damping="0.0" friction="0.1"/>
</joint>
```

**How to read this:**
- The caster sits at **(x=-0.255, y=-0.538, z=-0.055)** relative to the chassis frame.
- The hub joint frame is rotated with **yaw ≈ 90°** (`rpy ... 1.5707`) to align axes nicely.
- With `type="continuous"` the hub can swivel indefinitely (like a real caster).

**Validation trick (the MOTO way):**
- In RViz, enable TF frames.
- If you manually change this joint in a Joint State Publisher GUI, the hub frame should spin around **its local Z axis**.

#### 3.4.3 The caster wheel link (visual + simple collision)

```xml
<link name="front_right_caster_wheel">
  <visual>
    <geometry>
      <mesh filename="package://amrj16_description/models/meshes/amrj16_caster_wheel.dae"/>
    </geometry>
  </visual>

  <collision>
    <geometry>
      <cylinder radius="0.015" length="0.02"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.189"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <inertia ixx="0.000025441" ixy="0" ixz="0"
             iyy="0.00002543" iyz="-0.0000000017817" izz="0.000028616"/>
  </inertial>
</link>
```

**Why the collision is a cylinder (and not the mesh):**
- A cylinder is *cheap* for physics and usually “good enough”.
- Using the full mesh for collision often makes simulation slower and can create weird contacts.

#### 3.4.4 Wheel joint: `hub → wheel` (continuous roll)

```xml
<joint name="front_right_caster_hub_wheel_joint" type="continuous">
  <origin xyz="0 0.015 -0.0325" rpy="0 1.5707 0"/>
  <parent link="front_right_caster_hub"/>
  <child link="front_right_caster_wheel"/>
  <axis xyz="0 0 1"/>
  <limit effort="1000.0" velocity="100.0" />
  <dynamics damping="0.0" friction="0.1"/>
</joint>
```

This is the “mind-bender” part:

- The joint’s **axis** is still declared as `0 0 1`,
- but the joint frame is rotated by **pitch ≈ 90°** (`rpy="0 1.5707 0"`).

So: the wheel rotates around what *becomes* the rolling axis after that frame rotation.

> [IMAGE: Diagram showing joint frame rotation (pitch 90°) so Z becomes the rolling axis]

#### 3.4.5 Gazebo wheel contact parameters (why casters don’t skate forever)

Right after the wheel link you’ll see:

```xml
<gazebo reference="front_right_caster_wheel">
  <kp>500000</kp>
  <kd>500000</kd>
  <mu1>10.0</mu1>
  <mu2>10.0</mu2>
</gazebo>
```

These tags tune contact behavior in Gazebo (stiffness/damping + friction).  
Without them, small caster wheels can become “ice skates” in simulation.

#### 3.4.6 Front-left caster: same pattern, mirrored

AMRJ16 repeats the same chain for the left side:

- `front_left_caster_hub` + `chassis_front_left_caster_hub_joint`
- `front_left_caster_wheel` + `front_left_caster_hub_wheel_joint`

The only meaningful difference is the **Y sign** in the hub joint origin.

---


### 3.5 Sensors on AMRJ16 (IMU, LiDAR, RGB cameras, RGB-D)

In the ROS2 URDF course you shared, sensors are introduced as **extra links** plus a **Gazebo `<sensor>` block** attached via `<gazebo reference="...">` (and each block is then explained). We will do the same here, but using *AMRJ16’s real file* and naming. fileciteturn8file10

A useful mental model:

- **URDF/Xacro links + joints** define where the sensor *is* in the TF tree.
- **Gazebo `<sensor>` + `<plugin>`** define what the sensor *publishes* in simulation.

Let’s go one-by-one.

#### 3.5.1 Dual RGB cameras (left/right)

First you create an “empty” link (no visual/collision needed if you only want the TF):

```xml
<link name="rgb_camera_dx"/>
<joint name="camera_dx_joint" type="fixed">
  <origin rpy="0 0 0" xyz="-0.280 -0.186 -0.035"/>
  <parent link="chassis"/>
  <child link="rgb_camera_dx"/>
</joint>
```

Then you attach the Gazebo camera sensor to that link:

```xml
<gazebo reference="rgb_camera_dx">
  <sensor name="camera_dx" type="wideanglecamera">
    <camera>
      <horizontal_fov>6.283</horizontal_fov>
      <image><width>320</width><height>240</height></image>
      <clip><near>0.1</near><far>100</far></clip>
      <lens>
        <type>custom</type>
        <custom_function>
          <c1>1.05</c1><c2>4</c2><f>1.0</f><fun>tan</fun>
        </custom_function>
        <scale_to_hfov>true</scale_to_hfov>
        <cutoff_angle>3.1415</cutoff_angle>
        <env_texture_size>512</env_texture_size>
      </lens>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <cameraName>rgb_camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>rgb_camera_dx</frameName>
      <hackBaseline>0.07</hackBaseline>
    </plugin>
  </sensor>
</gazebo>
```

**What’s happening here:**
- The fixed joint places the TF frame of the camera relative to the chassis (so in RViz you can already see it).
- The Gazebo plugin (`libgazebo_ros_camera.so`) publishes image topics for that simulated camera.
- `frameName` should match your TF frame name, so messages carry a consistent `frame_id`.

The left camera `rgb_camera_sx` is the same structure, just mirrored in Y:

```xml
<joint name="rgb_camera_sx_joint" type="fixed">
  <origin rpy="0 0 0" xyz="-0.280 0.186 -0.035"/>
  <parent link="chassis"/>
  <child link="rgb_camera_sx"/>
</joint>
```

> [IMAGE: Front view of AMRJ16 showing left/right camera frames placed symmetrically on the chassis]

#### 3.5.2 2D LiDAR (LaserScan)

AMRJ16 models the LiDAR as a real link with a mesh:

```xml
<link name="lidar">
  <visual>
    <geometry>
      <mesh filename="package://amrj16_description/models/meshes/amrj16_rplidarS2.dae"/>
    </geometry>
  </visual>
  <inertial>
    <mass value="0.3"/>
    ...
  </inertial>
</link>

<joint name="chassis_lidar_joint" type="fixed">
  <origin rpy="3.14 0 0" xyz="-0.2 0 -0.020"/>
  <parent link="chassis"/>
  <child link="lidar"/>
</joint>
```

Notice the `rpy="3.14 0 0"`: it flips the LiDAR around X (often needed because the mesh “forward” direction is inverted relative to the ROS convention).

The Gazebo ray sensor publishes a `sensor_msgs/LaserScan`:

```xml
<gazebo reference="lidar">
  <sensor name="sensor_ray" type="ray">
    <pose>0 0 0.040 0 0 0</pose>
    <ray>
      <scan>
        <horizontal>
          <samples>500</samples>
          <min_angle>-3.14</min_angle>
          <max_angle>3.14</max_angle>
        </horizontal>
      </scan>
      <range><min>0.1</min><max>30.0</max></range>
    </ray>
    <update_rate>50.0</update_rate>
    <plugin name="laser" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Key checks:**
- In RViz add **LaserScan**, topic should be `/scan` (depending on namespace/remap).
- Frame should be `lidar`. If RViz complains, you likely have a TF mismatch.

> [IMAGE: RViz screenshot with LaserScan ring around AMRJ16]

#### 3.5.3 Asus Xtion style pointcloud (ray-based PointCloud)

The `asus_xtion` is implemented as a ray sensor that outputs a **PointCloud** message:

```xml
<link name="asus_xtion"> ... </link>

<joint name="chassis_asus_xtion_joint" type="fixed">
  <origin rpy="0 0 0" xyz="-0.095 0 0.131"/>
  <parent link="chassis"/>
  <child link="asus_xtion"/>
</joint>
```

And the Gazebo block:

```xml
<gazebo reference="asus_xtion">
  <sensor type="ray" name="pointcloud_sensor">
    <ray>
      <scan>
        <horizontal><samples>50</samples> ...</horizontal>
        <vertical><samples>50</samples> ...</vertical>
      </scan>
      <range><min>0.10</min><max>5.0</max></range>
      <noise> ... gaussian ... </noise>
    </ray>
    <update_rate>30</update_rate>
    <plugin name="gazebo_ros_block_laser_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <argument>~/out:=pointcloud</argument>
      </ros>
      <output_type>sensor_msgs/PointCloud</output_type>
      <frame_name>asus_xtion</frame_name>
      <min_intensity>100.0</min_intensity>
    </plugin>
  </sensor>
</gazebo>
```

This pattern is extremely close to the “PointCloud camera” example in the reference course, only adapted to AMRJ16 naming and placement. fileciteturn8file18

#### 3.5.4 RGB-D camera with optical frame chain (the “TF correctness” test)

This is the most educational part of the file because it forces you to build the classic camera frame convention chain:

1. `rgbd_camera_link_frame` (mount frame)
2. `camera_depth_frame`
3. `camera_depth_optical_frame` (rotated with `rpy="-1.57 0 -1.57"`)

```xml
<link name="rgbd_camera_link_frame">
  <visual>
    <geometry><cylinder length="0.01" radius="0.005"/></geometry>
  </visual>
</link>

<joint name="rgbd_camera_link_frame_joint" type="fixed">
  <origin rpy="0 0 0" xyz="0 0 0"/>
  <parent link="chassis"/>
  <child link="rgbd_camera_link_frame"/>
</joint>

<link name="camera_depth_frame"/>
<joint name="camera_depth_joint" type="fixed">
  <parent link="rgbd_camera_link_frame"/>
  <child link="camera_depth_frame"/>
</joint>

<link name="camera_depth_optical_frame"/>
<joint name="camera_depth_optical_joint" type="fixed">
  <origin xyz="0 0 0" rpy="-1.57 0 -1.57"/>
  <parent link="camera_depth_frame"/>
  <child link="camera_depth_optical_frame"/>
</joint>
```

That optical rotation is the “ROS camera standard” move: it converts from a mount frame to an optical frame (where +Z looks forward, etc.).  
Even if you don’t memorize it today, you should recognize the pattern.

Then the Gazebo depth camera plugin publishes RGB, depth and pointcloud topics:

```xml
<gazebo reference="rgbd_camera_link_frame">
  <sensor type="depth" name="depth_sensor">
    <update_rate>5.0</update_rate>
    <camera> ... 640x480 ... </camera>
    <plugin name="rgbd_controller" filename="libgazebo_ros_camera.so">
      <cameraName>amrj16moto</cameraName>
      <imageTopicName>rgb/image_raw</imageTopicName>
      <depthImageTopicName>depth/image_raw</depthImageTopicName>
      <pointCloudTopicName>depth/points</pointCloudTopicName>
      <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
      <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
      <frame_name>camera_depth_optical_frame</frame_name>
      ...
    </plugin>
  </sensor>
</gazebo>
```

**What to verify in RViz:**
- Add **Image** display for `rgb/image_raw`.
- Add **PointCloud2** (or PointCloud) display for `depth/points` (depending on plugin output).
- Ensure the fixed frame is something stable like `base_link` or `base_footprint`, and that `camera_depth_optical_frame` exists in TF.

> [IMAGE: RViz showing RGB image + depth pointcloud in front of AMRJ16]

#### 3.5.5 IMU (with noise model)

The IMU is placed on `base_link` with a fixed offset:

```xml
<link name="imu_link"/>
<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0.3 0 0" rpy="0 0 0"/>
</joint>
```

And then simulated with Gazebo’s IMU sensor + plugin:

```xml
<gazebo reference="imu_link">
  <sensor name="br_imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity> ... gaussian noise stddev 2e-4 ... </angular_velocity>
      <linear_acceleration> ... gaussian noise stddev 1.7e-2 ... </linear_acceleration>
    </imu>
    <plugin name="bytes_imu" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <remapping>~/out:=imu</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

**Practical takeaway:**
- The file already encodes a plausible noise model. If you ever build an EKF later, you’ll be happy this exists.

---



## 4. Quick “bench test”: generate URDF from Xacro

Before launching anything, ensure Xacro expands correctly:

```bash
cd ~/ros2_ws/src/amrj16_description/urdf
xacro amrj16.xacro > /tmp/amrj16.urdf
```

- If you see **no errors**, you’re good.
- If there are errors, fix them *before* you touch RViz.

---

## 5. Install package resources (CMakeLists.txt)

RViz and launch files are usually loaded from the package **install space**, so we must install our folders.

Open `~/ros2_ws/src/amrj16_description/CMakeLists.txt` and ensure you have an install block like:

```cmake
install(
  DIRECTORY
    urdf
    rviz
    launch
    models
  DESTINATION
    share/${PROJECT_NAME}/
)
```

---

## 6. Create a visualization launch file (robot_state_publisher + RViz)

### 6.1 Create the launch file

```bash
cd ~/ros2_ws/src/amrj16_description/launch
touch urdf_visualize.launch.py
chmod +x urdf_visualize.launch.py
```

### 6.2 Paste this launch script

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    package_description = "amrj16_description"
    xacro_file = "amrj16.xacro"

    robot_desc_path = os.path.join(
        get_package_share_directory(package_description),
        "urdf",
        xacro_file
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": True,
            "robot_description": Command(["xacro ", robot_desc_path]),
        }],
        output="screen",
    )

    rviz_config_path = os.path.join(
        get_package_share_directory(package_description),
        "rviz",
        "urdf_vis.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
```

---

## 7. Build and run

```bash
cd ~/ros2_ws
colcon build --packages-select amrj16_description
source install/setup.bash
ros2 launch amrj16_description urdf_visualize.launch.py
```

If everything is correct, `robot_state_publisher` will report it discovered segments (links) and publish `/robot_description`.

---

## 8. RViz configuration (do this carefully)

In RViz:

1. **Add** → `RobotModel`
   - **Description Source**: `Topic`
   - **Topic**: `/robot_description`
   - (If available) set QoS to **Reliable** and **Transient Local**

2. **Add** → `TF`

3. Set **Fixed Frame**:
   - Prefer `base_footprint` for AMRs (navigation-aligned)

Save configuration:
- `File → Save Config As…`
- Save to: `~/ros2_ws/src/amrj16_description/rviz/urdf_vis.rviz`

> [IMAGE: RViz panels — RobotModel, TF, Fixed Frame set to base_footprint]

---

## 9. Validate continuous joints with Joint State Publisher GUI

Because AMRJ16 has continuous joints (wheels/casters), TF for those links requires **joint states**.

Open a second terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

Move:
- `chassis_left_wheel_joint`
- `chassis_right_wheel_joint`
- caster joints (if present)

**Success criterion:**
- Links follow their parents correctly.
- Wheels rotate around the intended axis.

---

## 10. Exercises (submit as a short report)

### Exercise 1.1 — “Offset sanity check”
1. Temporarily change `xyz="0 0 0.103"` (between `base_footprint` and `base_link`) to `0.150`.
2. Relaunch and observe what changes in RViz.
3. Restore the original value.

**Answer:** What breaks (or becomes incorrect) and why?

### Exercise 1.2 — “Wheel orientation mistake”
1. In **one** driving wheel joint, change `rpy="-1.5707 0 0"` to `rpy="0 0 0"`.
2. Use joint_state_publisher_gui to rotate the wheel.
3. Restore the original `rpy`.

**Answer:** What did you observe and what does it tell you about mesh orientation vs joint axis?

---

## 11. Final checklist

- [ ] `xacro amrj16.xacro > /tmp/amrj16.urdf` runs with no errors
- [ ] RViz shows `base_footprint`, `base_link`, and `chassis`
- [ ] Wheels are visible and can be moved with joint_state_publisher_gui
- [ ] `urdf_vis.rviz` saved under `amrj16_description/rviz/`
- [ ] `CMakeLists.txt` installs `urdf/`, `launch/`, `rviz/`, `models/`

---

## What’s next (preview)

In Unit 2, we will refactor `amrj16.xacro` into **maintainable “fleet-ready” Xacro**:
- split into modules (base, chassis, wheels, casters, sensors)
- introduce properties/macros for reuse across variants
- keep the same RViz result but make the model easier to evolve
