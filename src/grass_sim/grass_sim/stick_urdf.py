"""Stick-figure URDF of the GRASS RPR arm, built from robot.yaml (L1, L2 and joint limits)."""

import math

TEMPLATE = """<?xml version="1.0"?>
<robot name="grass_stick">
  <material name="stick"><color rgba="0.9 0.9 0.9 1"/></material>
  <material name="joint"><color rgba="0.2 0.6 1.0 1"/></material>
  <material name="tip"><color rgba="1.0 0.6 0.1 1"/></material>

  <link name="world"/>
  <joint name="world_fix" type="fixed"><parent link="world"/><child link="base_link"/></joint>
  <!-- Z: the whole arm slides up and down a column on the base axis -->
  <link name="base_link">
    <visual><geometry><sphere radius="0.012"/></geometry><material name="joint"/></visual>
    <visual>
      <origin xyz="0 0 {z_half}"/>
      <geometry><cylinder radius="0.002" length="{z_len}"/></geometry><material name="stick"/>
    </visual>
  </link>

  <joint name="z" type="prismatic">
    <parent link="base_link"/><child link="carriage"/><axis xyz="0 0 1"/>
    <limit lower="{z_lo}" upper="{z_hi}" effort="1" velocity="1"/>
  </joint>
  <link name="carriage">
    <visual><geometry><sphere radius="0.010"/></geometry><material name="joint"/></visual>
  </link>

  <joint name="j1" type="revolute">
    <parent link="carriage"/><child link="link1"/><axis xyz="0 0 1"/>
    <limit lower="{j1_lo}" upper="{j1_hi}" effort="1" velocity="1"/>
  </joint>
  <link name="link1">
    <visual>
      <origin xyz="0 {l1_half} 0" rpy="1.570796 0 0"/>
      <geometry><cylinder radius="0.004" length="{l1}"/></geometry><material name="stick"/>
    </visual>
  </link>

  <joint name="j2" type="revolute">
    <parent link="link1"/><child link="link2"/><origin xyz="0 {l1} 0"/><axis xyz="0 0 1"/>
    <limit lower="{j2_lo}" upper="{j2_hi}" effort="1" velocity="1"/>
  </joint>
  <link name="link2">
    <visual><geometry><sphere radius="0.010"/></geometry><material name="joint"/></visual>
    <visual>
      <origin xyz="0 {l2_half} 0" rpy="1.570796 0 0"/>
      <geometry><cylinder radius="0.004" length="{l2}"/></geometry><material name="stick"/>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="link2"/><child link="camera_link"/><origin xyz="0 {l2} 0"/>
  </joint>
  <link name="camera_link">
    <visual><geometry><sphere radius="0.008"/></geometry><material name="tip"/></visual>
  </link>
</robot>
"""


def stick_urdf(cfg):
    l1, l2 = cfg.l1_mm / 1000.0, cfg.l2_mm / 1000.0
    z_lo, z_hi = cfg.z.minimum / 1000.0, cfg.z.maximum / 1000.0
    return TEMPLATE.format(
        l1=l1, l1_half=l1 / 2, l2=l2, l2_half=l2 / 2,
        z_lo=z_lo, z_hi=z_hi, z_len=z_hi - z_lo, z_half=(z_lo + z_hi) / 2,
        j1_lo=math.radians(cfg.j1.minimum), j1_hi=math.radians(cfg.j1.maximum),
        j2_lo=math.radians(cfg.j2.minimum), j2_hi=math.radians(cfg.j2.maximum))
