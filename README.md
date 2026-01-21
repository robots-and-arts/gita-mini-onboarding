# Gita Mini Robot – Onboarding Notes

![Gita Mini Robot](https://a.storyblok.com/f/255103/1036x1200/25fe0e8e7f/shop-citron-desktopl-1.jpg/m/smart/filters:quality(70))

**Author:** Aarya Sumuk  
**Last Updated:** January 2026  

This repository contains onboarding notes and basic operating instructions for the **Gita Mini robot**, intended for students and first-time users.

---

## Official Gita Resources

If you’d like to learn more about Gita, check out the official pages below:

- Official Website: https://piaggiofastforward.com/
- Gettig Started: https://knowledge.mygita.com/setting-up-getting-started
- User Manual: https://knowledge.mygita.com/hubfs/User-manual-gita-mini-v1.2.pdf

---

## Quick Start – Make Gita Follow You

1. Access the **robodorm** (card access required).
2. Remove Gita from the charger and place it on the ground.
3. Press the **power button on the back** to turn the robot on.
4. Stand **directly in front of the robot**.
5. Press the **button on the front**.

👉 Gita will immediately switch from **Park mode** to **Follow mode** and start following the person standing in front of it.

### Stop Following (Park Mode)

- Press the **front button again**.
- Gita will stop and return to **Park mode**.

---

## Handling and Safety Notes

- When **lifting the robot** or **placing it back on the charger**:
  - Lift only from the **front and back handles**
  - **Do NOT** hold or lift the robot by the **side wheels**

This helps prevent damage to the wheels and internal components.

---

## Startup & Shutdown Checklist

### Before Use
- ✅ Access the **robodorm**
- ✅ Unplug Gita from the charger
- ✅ Place Gita on the ground
- ✅ Power on using the **back button**

### After Use
- ✅ Return Gita to **Park mode**
- ✅ Power off using the **back button**
- ✅ Place Gita back on the **ground charger in the robodorm**

---

## Controlling Gita Using ROS (Advanced)

This section describes how to access and send commands to the Gita Mini robot using **ROS 2**.  
This is intended only for students who have completed **robot safety training**.

---

### Prerequisites (Required)

Before attempting ROS control:

1. **Install the MyGita App**
   - Download the **MyGita** app from the Google Play Store
   - Create an account

2. **Complete Safety Training**
   - Attend an in-person office hours safety training
   - After training, you will be added to the **crew** for each Gita robot you are allowed to use

> ⚠️ ROS access will not work unless your account has robot access.

---

### Network Requirements

After turning the robot on:

- Ensure the **robot is connected to the `EE-IoT` network**
  - This is the default configuration
- The **robot and control computer must be on the same network**
  - Other Stanford networks typically do **not** work due to firewall restrictions

---

### Robodorm Computer Access

1. Access the **robodorm** (card access required)
2. Turn on the robot
3. Log in to the robodorm computer with `Username: cs334_user` (password provided after training)
4. Confirm WiFi is **EE-IoT**

---

### Starting the ROS Docker Environment

Docker is **already installed and configured** on the robodorm computer.

From the Desktop directory:

```bash
docker compose build
docker compose up -d
docker compose ps
```
---

### Opening a Terminal Inside the Container

Use the service name from `docker compose ps`:

```bash
docker compose exec <service_name> bash
```

---

### Running ROS Commands

Inside the Docker container:

```bash
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.sh
```

Run the control node:

```bash
ros2 run gita_test gita_test_node --ros-args -p robot_id:=<id>
```

**Robot IDs:**
- `1` → Packard
- `3` → Rosie

---

### Control Method

- The ROS node uses **keyboard input** to control the robot
- Supported actions include:
  - Movement (forward, backward, turning)
  - Sit / stand
  - Pair / unpair

Key mappings can be found by inspecting the test code.

---

### Important Notes

- The robot must be **powered on** before running ROS
- The robot and computer must be on the **same network**
