import os
import shutil
import yaml
import argparse


def main(config_file, default_models_folder, destination_models_folder, destination_worlds_folder, force):
    # Load parameters from the config file
    with open(config_file, "r") as file:
        params = yaml.safe_load(file)


    num_drones = params['simulation']['number_of_drones']
    
    destination_world_name = f"SkywalkerX8_swarm{num_drones}.world"

    # Check if destination world already exists
    destination_world_path = os.path.join(destination_worlds_folder, destination_world_name)
    if os.path.exists(destination_world_path) and not force:
        print(f"World file {destination_world_name} already exists. Exiting.")
        return

    
    reason =  "will be forced overwritten" if force else "does not exist"
    print(f"{destination_world_path} {reason}. Generating necessary drones and world file...")
    

    fdm_port_in_default = params['simulation']['fdm_port_in_default']

    # Process each drone     
    for i in range(num_drones):
        drone = params["drones"][i]
        
        model_name = drone["name"]
        model_destination_path = os.path.join(destination_models_folder, model_name)

        # Check if the model already exists in the destination folder
        if os.path.exists(model_destination_path) and not force:
            print(f"Model {model_name} already exists. Skipping...")
            continue

        # Copy default model to destination with new name
        shutil.copytree(default_models_folder, model_destination_path,dirs_exist_ok=True)
        print(f"Copied model {model_name} to {model_destination_path}")

        # Generate and save model.config
        model_config_content = f"""
        <?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version='1.7'>model.sdf</sdf>

  <author>
    <name>Roman Bapst</name>
    <email>roman@px4.io</email>
  </author>

  <author>
    <name>Rhys Mainwaring</name>
    <email>rhys.mainwaring@me.com</email>
  </author>

  <author> <!-- Modified by Steve Carter Feujo Nomeny -->
    <name>Steve Carter Feujo Nomeny</name>
    <email>steve@scnomeny.com</email>
  </author>

  <description>
    This is a model of a Skywalker X8 plane with ArduPilot integration.
  </description>
</model>
        """
        
        with open(os.path.join(model_destination_path, "model.config"), "w") as model_config_file:
            model_config_file.write(model_config_content.strip())


        
        drone_fdm_port_in = fdm_port_in_default + i*10

        # Generate and save model.sdf
        model_sdf_content = f"""
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{model_name}">
    <pose>0 0 0.246 0 0 0</pose>

    <link name='base_link'>
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>4.5</mass>
        <inertia>
          <ixx>0.45</ixx>
          <ixy>0</ixy>
          <ixz>0.06</ixz>
          <iyy>0.325</iyy>
          <iyz>0</iyz>
          <izz>0.75</izz>
        </inertia>
      </inertial>
      <collision name='base_link_collision'>
        <pose>0 0 -0.07 0 0 0</pose>
        <geometry>
          <box>
            <size>0.80 2.15 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name='base_link_visual'>
        <geometry>
          <mesh>
            <uri>model://skywalker_x8/meshes/x8_wing.dae</uri>
          </mesh>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0</diffuse>
          <specular>1.0 1.0 1.0</specular>
          <pbr>
            <metal>
                <albedo_map>model://skywalker_x8/materials/textures/x8.tga</albedo_map>
                <metalness>0.1</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
      <visual name='m4'>
        <pose degrees="true">-0.361 0 0.0 0 90 0</pose>
        <geometry>
          <cylinder>
            <length>0.035</length>
            <radius>0.02</radius>
          </cylinder>
        </geometry>
        <material>
          <ambient>1 1 1</ambient>
          <diffuse>1 1 1</diffuse>
          <specular>1 1 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://skywalker_x8/materials/textures/blue_metal/blue_metal_basecolor.jpg</albedo_map>
              <metalness_map>model://skywalker_x8/materials/textures/blue_metal/blue_metal_metallic.jpg</metalness_map>
              <normal_map>model://skywalker_x8/materials/textures/blue_metal/blue_metal_normal.jpg</normal_map>
              <roughness_map>model://skywalker_x8/materials/textures/blue_metal/blue_metal_roughness.jpg</roughness_map>
              <roughness>1.0</roughness>
              <metalness>1.0</metalness> 
            </metal>
          </pbr>
        </material>
      </visual>
      <!-- save for debugging use -->
      <!-- <visual name="cp_wing">
        <pose>0.01 0 0 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.03</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0 0 1</ambient>
          <diffuse>0 0 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <visual name="cp_left_elevon">
        <pose>-0.30 0.75 -0.005 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.03</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 0 0</ambient>
          <diffuse>1 0 0</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <visual name="cp_right_elevon">
        <pose>-0.30 -0.75 -0.005 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.03</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0 1 0</ambient>
          <diffuse>0 1 0</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <visual name="cp_left_winglet">
        <pose>-0.36 1.04 0.08 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.03</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 0 0</ambient>
          <diffuse>1 0 0</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <visual name="cp_right_winglet">
        <pose>-0.36 -1.04 0.08 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.03</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0 1 0</ambient>
          <diffuse>0 1 0</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <visual name="cp_left_wing">
        <pose>-0.14 0.5 -0.005 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.03</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 1 0</ambient>
          <diffuse>1 1 0</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <visual name="cp_right_wing">
        <pose>-0.14 -0.5 -0.005 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.03</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 1 0</ambient>
          <diffuse>1 1 0</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual> -->
    </link>

    <link name='rotor_pusher'>
      <pose degrees="true">-0.385 0 0.0 0 90 0</pose>
      <inertial>
        <mass>0.025</mass>
        <inertia>
          <ixx>9.75e-06</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.000166704</iyy>
          <iyz>0</iyz>
          <izz>0.000167604</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <cylinder>
            <length>0.005</length>
            <radius>0.065</radius>
          </cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8 0.8 0.8</scale>
            <uri>model://skywalker_x8/meshes/iris_prop_ccw.dae</uri>
          </mesh>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0</diffuse>
          <specular>1.0 1.0 1.0</specular>
          <pbr>
            <metal>
                <albedo_map>model://skywalker_x8/materials/textures/x8.tga</albedo_map>
                <metalness>0.5</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
    <joint name='rotor_pusher_joint' type='revolute'>
      <child>rotor_pusher</child>
      <parent>base_link</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <damping>0.002</damping>
        </dynamics>
        <use_parent_model_frame>1</use_parent_model_frame>
      </axis>
    </joint>

    <link name="left_elevon">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.000102319</ixx>
          <ixy>0</ixy>
          <iyy>0.00334417</iyy>
          <ixz>0</ixz>
          <iyz>0</iyz>
          <izz>0.003446072</izz>
        </inertia>
        <pose>-0.30 0.75 -0.005 0 0 0</pose>
      </inertial>
      <visual name='left_elevon_visual'>
        <geometry>
          <mesh>
            <uri>model://skywalker_x8/meshes/x8_elevon_left.dae</uri>
          </mesh>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0</diffuse>
          <specular>1.0 1.0 1.0</specular>
          <pbr>
            <metal>
                <albedo_map>model://skywalker_x8/materials/textures/x8.tga</albedo_map>
                <metalness>0.1</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>

    <link name="right_elevon">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.000102319</ixx>
          <ixy>0</ixy>
          <iyy>0.00334417</iyy>
          <ixz>0</ixz>
          <iyz>0</iyz>
          <izz>0.003446072</izz>
        </inertia>
        <pose>-0.30 -0.75 -0.005 0 0 0</pose>
      </inertial>
      <visual name='right_elevon_visual'>
        <geometry>
          <mesh>
            <uri>model://skywalker_x8/meshes/x8_elevon_right.dae</uri>
          </mesh>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0</diffuse>
          <specular>1.0 1.0 1.0</specular>
          <pbr>
            <metal>
                <albedo_map>model://skywalker_x8/materials/textures/x8.tga</albedo_map>
                <metalness>0.1</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>

    <!-- Joint range -30/+30 deg. -->
    <joint name='left_elevon_joint' type='revolute'>
      <parent>base_link</parent>
      <child>left_elevon</child>
      <pose>-0.27 0.6 -0.005 0 0 0.265</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.53</lower>
          <upper>0.53</upper>
        </limit>
        <dynamics>
          <damping>0.1</damping>
        </dynamics>
      </axis>
    </joint>
    
    <!-- Joint range -30/+30 deg. -->
    <joint name='right_elevon_joint' type='revolute'>
      <parent>base_link</parent>
      <child>right_elevon</child>
      <pose>-0.27 -0.6 -0.005 0 0 -0.265</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.53</lower>
          <upper>0.53</upper>
        </limit>
        <dynamics>
          <damping>0.1</damping>
        </dynamics>
      </axis>
    </joint>

    <!-- sensors -->
    <link name='imu_link'>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.15</mass>
        <inertia>
          <ixx>0.00002</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00002</iyy>
          <iyz>0</iyz>
          <izz>0.00002</izz>
        </inertia>
      </inertial>
      <sensor name="imu_sensor" type="imu">
        <pose degrees="true">0 0 0 180 0 0</pose>
        <always_on>1</always_on>
        <update_rate>1000.0</update_rate>
      </sensor>
    </link>
    <joint name='skywalker_x8/imu_joint' type='revolute'>
      <child>imu_link</child>
      <parent>base_link</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>0</lower>
          <upper>0</upper>
          <effort>0</effort>
          <velocity>0</velocity>
        </limit>
        <dynamics>
          <damping>1.0</damping>
        </dynamics>
        <use_parent_model_frame>1</use_parent_model_frame>
      </axis>
    </joint>

    <!-- plugins -->
    <plugin filename="gz-sim-joint-state-publisher-system"
      name="gz::sim::systems::JointStatePublisher">
    </plugin>

    <!-- rotor_pusher lift-drag -->
    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.30</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.0</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.02</area>
      <air_density>1.2041</air_density>
      <cp>0.074205 0 0</cp>
      <forward>0 1 0</forward>
      <upward>0 0 1</upward>
      <link_name>rotor_pusher</link_name>
    </plugin>
    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.30</a0>
      <alpha_stall>1.4</alpha_stall>
      <cla>4.2500</cla>
      <cda>0.10</cda>
      <cma>0.0</cma>
      <cla_stall>-0.025</cla_stall>
      <cda_stall>0.0</cda_stall>
      <cma_stall>0.0</cma_stall>
      <area>0.02</area>
      <air_density>1.2041</air_density>
      <cp>-0.074205 0 0</cp>
      <forward>0 -1 0</forward>
      <upward>0 0 1</upward>
      <link_name>rotor_pusher</link_name>
    </plugin>
    <!-- wing lift-drag -->
    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.13</a0>
      <cla>3.7</cla>
      <cda>0.06417112299</cda>
      <cma>0.0</cma>
      <alpha_stall>0.3391428111</alpha_stall>
      <cla_stall>-3.85</cla_stall>
      <cda_stall>-0.9233984055</cda_stall>
      <cma_stall>0.0</cma_stall>
      <cp>0.01 0 0</cp>
      <area>0.80</area>
      <air_density>1.2041</air_density>
      <forward>1 0 0</forward>
      <upward>0 0 1</upward>
      <link_name>base_link</link_name>
    </plugin>
    <!-- left_elevon lift-drag -->
    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.15</a0>
      <cla>6.8</cla>
      <cda>0.06417112299</cda>
      <cma>0.0</cma>
      <alpha_stall>0.6391428111</alpha_stall>
      <cla_stall>-3.85</cla_stall>
      <cda_stall>-0.9233984055</cda_stall>
      <cma_stall>0.0</cma_stall>
      <cp>-0.30 0.75 -0.005</cp>
      <area>0.10</area>
      <air_density>1.2041</air_density>
      <forward>1 0 0</forward>
      <upward>0 0 1</upward>
      <link_name>base_link</link_name>
      <control_joint_name>left_elevon_joint</control_joint_name>
      <control_joint_rad_to_cl>-5.0</control_joint_rad_to_cl>
    </plugin>
    <!-- right_elevon lift-drag -->
    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.15</a0>
      <cla>6.8</cla>
      <cda>0.06417112299</cda>
      <cma>0.0</cma>
      <alpha_stall>0.6391428111</alpha_stall>
      <cla_stall>-3.85</cla_stall>
      <cda_stall>-0.9233984055</cda_stall>
      <cma_stall>0.0</cma_stall>
      <cp>-0.30 -0.75 -0.005</cp>
      <area>0.10</area>
      <air_density>1.2041</air_density>
      <forward>1 0 0</forward>
      <upward>0 0 1</upward>
      <link_name>base_link</link_name>
      <control_joint_name>right_elevon_joint</control_joint_name>
      <control_joint_rad_to_cl>-5.0</control_joint_rad_to_cl>
    </plugin>
    <!-- left_winglet lift-drag -->
    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.0</a0>
      <cla>4.752798721</cla>
      <cda>0.6417112299</cda>
      <cma>0.0</cma>
      <alpha_stall>0.3391428111</alpha_stall>
      <cla_stall>-3.85</cla_stall>
      <cda_stall>-0.9233984055</cda_stall>
      <cma_stall>0.0</cma_stall>
      <cp>-0.36 1.04 0.08</cp>
      <area>0.12</area>
      <air_density>1.2041</air_density>
      <forward>1 0 0</forward>
      <upward>0 1 0</upward>
      <link_name>base_link</link_name>
    </plugin>
    <!-- right_winglet lift-drag -->
    <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.0</a0>
      <cla>4.752798721</cla>
      <cda>0.6417112299</cda>
      <cma>0.0</cma>
      <alpha_stall>0.3391428111</alpha_stall>
      <cla_stall>-3.85</cla_stall>
      <cda_stall>-0.9233984055</cda_stall>
      <cma_stall>0.0</cma_stall>
      <cp>-0.36 -1.04 0.08</cp>
      <area>0.12</area>
      <air_density>1.2041</air_density>
      <forward>1 0 0</forward>
      <upward>0 -1 0</upward>
      <link_name>base_link</link_name>
    </plugin>

    <!-- original from standard_vtol -->
    <!-- left_wing_lift-drag -->
    <!-- <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.05984281113</a0>
      <cla>4.752798721</cla>
      <cda>0.6417112299</cda>
      <cma>0.0</cma>
      <alpha_stall>0.3391428111</alpha_stall>
      <cla_stall>-3.85</cla_stall>
      <cda_stall>-0.9233984055</cda_stall>
      <cma_stall>0.0</cma_stall>
      <cp>-0.05 0.3 0.05</cp>
      <area>0.50</area>
      <air_density>1.2041</air_density>
      <forward>1 0 0</forward>
      <upward>0 0 1</upward>
      <link_name>base_link</link_name>
      <control_joint_name>left_elevon_joint</control_joint_name>
      <control_joint_rad_to_cl>-1.0</control_joint_rad_to_cl>
    </plugin> -->
    <!-- right_wing_lift-drag -->
    <!-- <plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
      <a0>0.05984281113</a0>
      <cla>4.752798721</cla>
      <cda>0.6417112299</cda>
      <cma>0.0</cma>
      <alpha_stall>0.3391428111</alpha_stall>
      <cla_stall>-3.85</cla_stall>
      <cda_stall>-0.9233984055</cda_stall>
      <cma_stall>0.0</cma_stall>
      <cp>-0.05 -0.3 0.05</cp>
      <area>0.50</area>
      <air_density>1.2041</air_density>
      <forward>1 0 0</forward>
      <upward>0 0 1</upward>
      <link_name>base_link</link_name>
      <control_joint_name>right_elevon_joint</control_joint_name>
      <control_joint_rad_to_cl>-1.0</control_joint_rad_to_cl>
    </plugin> -->

    <plugin filename="gz-sim-apply-joint-force-system"
      name="gz::sim::systems::ApplyJointForce">
      <joint_name>rotor_pusher_joint</joint_name>
    </plugin>
    <plugin filename="gz-sim-apply-joint-force-system"
      name="gz::sim::systems::ApplyJointForce">
      <joint_name>left_elevon_joint</joint_name>
    </plugin>
    <plugin filename="gz-sim-apply-joint-force-system"
      name="gz::sim::systems::ApplyJointForce">
      <joint_name>right_elevon_joint</joint_name>
    </plugin>

    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <!-- Port settings -->
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>{drone_fdm_port_in}</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>

      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>

      <!-- Sensors -->
      <imuName>imu_sensor</imuName>

      <!--
          incoming control command [0, 1]
          so offset it by 0 to get [0, 1]
          and divide max target by 1.
          offset = 0
          multiplier = 838 max rpm / 1 = 838
        -->

      <!-- 
          SERVO3_FUNCTION   70 (Throttle)
          SERVO3_MAX        2000
          SERVO3_MIN        1000
          SERVO3_REVERSED   0
          SERVO3_TRIM       1000
       -->
      <control channel="2">
        <jointName>rotor_pusher_joint</jointName>
        <useForce>1</useForce>
        <multiplier>838</multiplier>
        <offset>0</offset>
        <servo_min>1000</servo_min>
        <servo_max>2000</servo_max>
        <type>VELOCITY</type>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
      </control>

      <!-- 
          SERVO1_FUNCTION   77 (Elevon Left)
          SERVO1_MAX        1900
          SERVO1_MIN        1100
          SERVO1_REVERSED   0
          SERVO1_TRIM       1500

          pwm:          =>  [1100, 1900] 
          input:        =>  [0, 1]
          offset: -0.5  =>  [-0.5, 0.5]
          scale:   2.0  =>  [-1.0, 1.0]
          scale: 0.524  =>  [-0.524, 0.524]
       -->
      <control channel="0">
        <jointName>left_elevon_joint</jointName>
        <useForce>1</useForce>
        <multiplier>1.048</multiplier>
        <offset>-0.5</offset>
        <servo_min>1100</servo_min>
        <servo_max>1900</servo_max>
        <type>POSITION</type>
        <p_gain>10.0</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
      </control>

      <!-- 
          SERVO2_FUNCTION   78 (Elevon Right)
          SERVO2_MAX        1900
          SERVO2_MIN        1100
          SERVO2_REVERSED   0
          SERVO2_TRIM       1500
       -->
      <control channel="1">
        <jointName>right_elevon_joint</jointName>
        <useForce>1</useForce>
        <multiplier>1.048</multiplier>
        <offset>-0.5</offset>
        <servo_min>1100</servo_min>
        <servo_max>1900</servo_max>
        <type>POSITION</type>
        <p_gain>10.0</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
      </control>

    </plugin>
  </model>
</sdf>

        """
        
        with open(os.path.join(model_destination_path, "model.sdf"), "w") as model_sdf_file:
            model_sdf_file.write(model_sdf_content.strip())

    # Creating world file
    print(f"Creatting world file: {destination_world_path}.")
    

    # Generate world.sdf with all drone models
    world_sdf_content = f"""
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="runway">
    <plugin filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin filename="gz-sim-imu-system"
      name="gz::sim::systems::Imu">
    </plugin>

    <scene>
      <ambient>1.0 1.0 1.0</ambient>
      <background>0.8 0.8 0.8</background>
      <sky></sky>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <include>
      <pose degrees="true">-29 545 0 0 0 363</pose>
      <uri>model://runway</uri>
    </include>

    <!-- SkyWalker X8 model -->
    """
    for i in range(num_drones):       
        drone = params["drones"][i]
        model_name = drone["name"]
        x = drone["start_orientaton_moos"]["x"]
        y = drone["start_orientaton_moos"]["y"]
        hdg = drone["start_orientaton_moos"]["hdg"]
        rot_z = 90 - hdg
        
        world_sdf_content += f"""
    <include>
        <pose degrees="true">{x} {y} 0.2 0 0 {rot_z}</pose>
        <uri>model://{model_name}</uri>
    </include>
        
        """
    world_sdf_content += """
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>-1.0</real_time_factor>
    </physics>
  </world>
</sdf>
    """

    # Save the new world file
    with open(destination_world_path, "w") as world_file:
        world_file.write(world_sdf_content.strip())

    print(f"Generated world file at {destination_world_path}")


if __name__ == "__main__":

    home_path = os.path.expanduser("~")

    parser = argparse.ArgumentParser(description="Generate SDF files for vehicles and world.")
    parser.add_argument(
        "--config_file",
        default=os.path.join(home_path, "moos-ivp-uav/missions/UAV_Fly/missionConfig.yaml"),
        help="Path to the configuration YAML file. Default: ~/moos-ivp-uav/missions/UAV_Fly/missionConfig.yaml"
    )
    parser.add_argument(
        "--default_models_folder",
        default=os.path.join(home_path, "SITL_Models/Gazebo/models/skywalker_x8/"),
        help="Path to the folder containing default models. Default: ~/SITL_Models/Gazebo/models/skywalker_x8/"
    )

    parser.add_argument(
        "--destination_models_folder",
        default=os.path.join(home_path, "moos-ivp-uav/GazeboSim/models/"),
        help="Path to the destination models folder. Default: ~/moos-ivp-uav/GazeboSim/models/"
    )
    parser.add_argument(
        "--destination_worlds_folder",
        default=os.path.join(home_path, "moos-ivp-uav/GazeboSim/worlds/"),
        help="Path to the destination worlds folder. Default: ~/moos-ivp-uav/GazeboSim/worlds/"
    )
    parser.add_argument(
        "--forceOverwrite", '-f',
        action="store_true",
        help="Force overwrite existing files."
    )
    
    args = parser.parse_args()

    main(
        config_file=args.config_file,
        default_models_folder=args.default_models_folder,
        destination_models_folder=args.destination_models_folder,
        destination_worlds_folder=args.destination_worlds_folder,
        force=args.forceOverwrite
    )
