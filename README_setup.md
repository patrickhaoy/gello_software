# GELLO Software Setup Guide

This guide covers setting up the GELLO software to control UR5e robotic arms and Robotiq grippers in both simulation and real-world environments.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [UR5e Robot Configuration](#ur5e-robot-configuration)
   - [Network Security Setup](#network-security-setup)
   - [Network Configuration](#network-configuration)
   - [External Control Setup](#external-control-setup)
3. [Robotiq Gripper Setup](#robotiq-gripper-setup)
4. [GELLO Software Setup](#gello-software-setup)
   - [Simulation Environment](#simulation-environment)
   - [Single UR5e Arm (Real)](#single-ur5e-arm-real)
   - [Bimanual UR5e Arms (Real)](#bimanual-ur5e-arms-real)
5. [Troubleshooting](#troubleshooting)
6. [Additional Resources](#additional-resources)

---

## Prerequisites

Before starting, ensure you have:

- UR5e robotic arm with pendant
- Robotiq 2F-85 gripper
- PC with Ethernet connection
- USB flash drive for URCap installation
- Python environment (see main README.md for setup)

---

## UR5e Robot Configuration

### Network Security Setup

**Step 1: Access UR5e Pendant Settings**

1. Power on the UR5e robot
2. On the pendant, switch to **Manual Mode** (top right, 3rd icon from right)
3. Navigate to **Settings** (top right corner)

**Step 2: Configure System Permissions**

**Important**: Network permissions are disabled by default. You must enable them for PC communication.

Navigate through all settings and ensure the following:

**System Settings:**
- **Remote Control**: Enable
- **Constrained Freedrive**: Enable

**Security Settings:**
- **Restrict inbound network access to this subnet**: Leave blank
- **Disable inbound access to additional interfaces (by port)**: Leave blank
- **Services**: Enable all services

**Note**: This is the most common reason for communication failures between PC and UR5e. Take time to thoroughly check each setting.

### Network Configuration

**Step 3: Configure UR5e Network Settings**

1. In pendant settings, go to **Network Settings**
2. Set to **Static Address** with the following configuration:

```
IP Address:     192.168.1.xxx
Subnet Mask:    255.255.255.0
Default Gateway: 0.0.0.0
Preferred DNS:   0.0.0.0
Alternative DNS: 0.0.0.0
```

**Step 4: Configure PC Network Settings**

1. Connect Ethernet cable from PC to UR5e
2. On your PC, go to **Network Settings** → **Ethernet Connection**
3. Set **IPv4** to **Manual** with:

```
Address:  192.168.1.yyy
Netmask:  255.255.255.0
```

**Note**: `xxx` and `yyy` should be different numbers (e.g., `.10` and `.11`)

### External Control Setup

**Step 5: Install External Control URCap**

1. Download the External Control URCap from:
   - [Universal Robots External Control URCap](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap)

2. Copy the URCap file to a USB flash drive
3. Insert USB drive into UR5e pendant
4. Install the URCap following the on-screen prompts

**Step 6: Configure External Control**

1. Navigate to **Installation** → **URCaps** → **External Control**
2. Configure the following settings:

```
Host IP:      192.168.1.yyy
Custom Port:  30004
Host Name:    192.168.1.yyy
```

**Note**: Use the same IP address as your PC network configuration

**Verification**: The arm should now be connected and controllable from your PC. To verify run `ping 192.168.1.xxx` from PC.

---

## Robotiq Gripper Setup

**Step 1: Install Gripper URCap**

1. Download the Robotiq Gripper URCap from:
   - [Robotiq Support Downloads](https://robotiq.com/support)

2. Copy the URCap file to a USB flash drive
3. Insert USB drive into UR5e pendant
4. Install the URCap following the on-screen prompts

**Step 2: Configure Gripper**

1. Navigate to **Installation** → **URCaps** → **Gripper**
2. Click **Scan** - the 2F-85 gripper should appear in the list
3. Select and configure the gripper

**Verification**: The gripper should now be detected and controllable.

---

## GELLO Software Setup

### Simulation Environment

**Step 1: Environment Setup**

Follow the main `README.md` to:
- Create virtual environment
- Install dependencies
- Identify motors
- Compute offsets

**Step 2: Launch Simulation**

```bash
# Launch simulation nodes
python experiments/launch_nodes.py --robot sim_ur

# Run GELLO agent in simulation
python experiments/run_env.py --agent=gello
```

**Expected Result**: You should be able to control the robot in MuJoCo simulation.

### Single UR5e Arm (Real)

**Step 1: Launch Real Robot Nodes**

```bash
# Launch real robot nodes
python experiments/launch_nodes.py --robot ur

# Run GELLO agent with real robot
python experiments/run_env.py --agent=gello
```

**Expected Result**: You should be able to control the real UR5e arm.

### Bimanual UR5e Arms (Real)

**Step 1: Configure Second Arm**

For the second UR5e arm, configure different network settings:

**UR5e Pendant Network Settings (Second Arm):**
```
IP Address:     192.168.1.zzz
Subnet Mask:    255.255.255.0
Default Gateway: 0.0.0.0
Preferred DNS:   0.0.0.0
Alternative DNS: 0.0.0.0
```

**External Control URCap (Second Arm):**
```
Host IP:      192.168.1.yyy  # Same as PC
Custom Port:  30004
Host Name:    192.168.1.yyy
```

**Note**: `zzz` should be different from both `xxx` and `yyy` (e.g., `.12`)

**Step 2: Launch Bimanual System**

```bash
# Launch bimanual robot nodes
python experiments/launch_nodes.py --robot bimanual_ur

# Run GELLO agent with bimanual setup
python experiments/run_env.py --agent=gello --bimanual
```

**Expected Result**: You should be able to control both UR5e arms simultaneously.

---