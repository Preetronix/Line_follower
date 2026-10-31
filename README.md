# Camera-Based Autonomous Line-Following Robot

A two-wheeled, camera-based autonomous robot project combining **ROS 2 + Gazebo simulation, a generic aah CAD model, and an electronics design for Raspberry Pi Zero 2W**. The robot uses a front-facing Raspberry Pi Camera module 3 and OpenCV to follow a closed-loop dotted track. No IR line sensors are used as mentioned in problem statement.

> **Project status:** The ROS 2/Gazebo source is included and has been statically checked while assembling this repository, but it has 
** Has been live-tested on the target Pixi (Mac) + ROS 2 Jazzy + Gazebo Harmonic machine**. 

## At a glance

| Subsystem | Design |
|---|---|
| Main processor | Raspberry Pi Zero 2W |
| Vision |Raspberry Pi Camera module 3; OpenCV image processing |
| Drive | Two geared DC motors, two motored wheels, one front caster |
| Motor control | 1 x L298 dual H-bridge motor driver |
| CAD | Design made on Onshape |
| Simulation | ROS 2 Jazzy + Gazebo Harmonic; four-wheel differential drive |
| Line sensors | None — camera-only line detection |

## Repository layout

```text
camera-based-line-following-robot/
├── README.md
├── cad/
│   ├── step/                     # Onshape-compatible coloured STEP assemblies
│   ├── stl/                      # Static mesh exports
├── electronics/
│   └── cirkit/
│       ├── parts_list            # List of all material for building the circuit
└── ros2_ws/
    └── src/                      # ROS 2 Python package, SDF, world, launch, config
```

## 1. CAD

The CAD is a concept-level assembly. Some component shapes, port positions, fasteners, and clearances are simplified.

## 2. Electronics

### Electronics inventory

- 1 × Raspberry Pi Zero 2W
- 1 × Raspberry Pi Camera module 3
- 2 × L298 Dual H Bridge
- 2 × geared DC motors with wheels and one caster wheel
- 1 × 5V power supply
- 1 x 12V power supply

## 3. ROS 2 and Gazebo

Target environment: ** Pixi 0.81.0, ROS 2 Jazzy, Gazebo Harmonic **.
 The SDF camera target is 30 Hz at 160 × 120, but the measured rate depends on Gazebo rendering performance and the host computer.

## Dimensions

| Parameter | Value |
|---|---:|
| Chassis: 180 × 130 × 4 mm
| Wheel diameter: 69 mm
| Wheel track: 167 mm
| Overall robot: 188.25 × 186 × 75.5 mm

The Gazebo physics chassis is derived from an .stl model thus has no colour.

## Licensing

No root-level license has been selected for this combined repository.