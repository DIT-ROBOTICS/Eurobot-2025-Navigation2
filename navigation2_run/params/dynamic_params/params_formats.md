# 🧭 Navigation Parameter Configuration

This document combines all YAML configuration files used for tuning various robot navigation behaviors, such as speed adjustments, rival detection, and docking alignment.

---

## 🗂️ Configurations Overview

| File Name                   | Purpose                             |
|----------------------------|-------------------------------------|
| `rival_params.yaml`        | Rival and docking behavior          |
| `nav_didilong_params.yaml` | High-performance general motion     |
| `nav_fast_params.yaml`     | Fast linear and angular movement    |
| `nav_slow_params.yaml`     | Slower, stable movement             |
| `nav_linearBoost_params.yaml` | Boosted linear velocity         |
| `nav_angularBoost_params.yaml` | Boosted angular velocity       |

---

## 🔍 rival_params.yaml

Defines parameters for rival avoidance and docking control.

```yaml
nav_rival_parameters:
  rival_inscribed_radius: 0.22     # Minimum safe distance from rivals

dock_rival_parameters:
  dock_rival_radius: 0.46          # Docking trigger radius
  dock_rival_degree: 120           # Docking alignment threshold in degrees
```

---

## 🏎️ nav_didilong_params.yaml

High-performance general motion profile (Didilong robot config).

```yaml
robot_parameters:
  max_linear_velocity: 1.5         # Maximum forward speed (m/s)
  max_angular_velocity: 1.0        # Maximum rotation speed (rad/s)
```

---

## ⚡ nav_fast_params.yaml

Fast movement with high angular agility.

```yaml
robot_parameters:
  max_linear_velocity: 1.1         # Balanced fast linear speed
  max_angular_velocity: 12.0       # High-speed turning
```

---

## 🐢 nav_slow_params.yaml

Slower configuration for stable or cautious navigation.

```yaml
robot_parameters:
  max_linear_velocity: 0.8         # Reduced forward speed
  max_angular_velocity: 2.0        # Moderate turning speed
```

---

## ➕ nav_linearBoost_params.yaml

Configuration focused on boosting straight-line speed.

```yaml
robot_parameters:
  max_linear_velocity: 1.1         # Enhanced linear speed
  max_angular_velocity: 2.0        # Moderate rotation
```

---

## 🔄 nav_angularBoost_params.yaml

Configuration focused on improving turning responsiveness.

```yaml
robot_parameters:
  max_linear_velocity: 0.5         # Reduced linear speed for precision
  max_angular_velocity: 12.0       # Maximum angular agility
```

---

## 📌 Usage

To use these configurations, specify the appropriate YAML file in `navigation2_run/params/nav2_params_xx.yaml`.

- **controller_server**
- **rival_layer**
- **docking_server**

---

## 🛠️ Notes

- All velocities:
  - **Linear** in meters/second (m/s)
  - **Angular** in radians/second (rad/s)
- Choose the configuration that fits your robot's environment and desired behavior.
- You may dynamically switch between them during runtime if supported.

---
