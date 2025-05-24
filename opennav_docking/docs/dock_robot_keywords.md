# `/dock_robot` API: Supported `/dock_type` Keywords

This document provides a detailed explanation of the supported keywords used in the `/dock_type` parameter of the `/dock_robot` action server. These keywords allow dynamic customization of the docking behavior at runtime.

- The order of keywords **does not** matter.  
- The system parses and combines them to adjust the navigation strategy accordingly.  
- The keywords are **case-sensitive**.  
- **CamelCase** is the default format.

---

## ✅ Keyword Categories

### 1. Template Base

| Keyword | Description |
|---------|-------------|
| `dock`  | **(Required)** Triggers the docking process. Must be present in the keyword string to initiate docking. |

---

### 2. Controller Type

Adjusts the controller plugin or configuration used during navigation.

| Keyword        | Behavior                                                                                  |
|----------------|-------------------------------------------------------------------------------------------|
| `fast`         | Uses a high-speed controller profile to reduce navigation time. Prioritizes speed over stability. |
| `slow`         | Uses a slow and cautious controller profile for more cautious navigation. Suitable when stability is a priority. |
| `linearBoost`  | Purely increases linear velocity during navigation. Useful when you don't need to spin.  |
| `angularBoost` | Increases angular velocity for quicker orientation, designed for rotate-in-place to avoid unstable behaviors. |

**Default:** slow

---

### 3. Goal Checker Type

Adjusts the goal-checking behavior (i.e., when the robot is considered to have "arrived").

| Keyword    | Behavior                                                                                  |
|------------|-------------------------------------------------------------------------------------------|
| `precise`  | Uses strict tolerances for position and orientation. Ensures exact pose alignment.       |
| `loose`    | Uses relaxed thresholds to consider goal reached sooner. Ideal when speed is more important than precision. |
| `unerring` | Extremely strict tolerances for position and orientation. May take more time to finish.  |

**Default:** loose

---

### 4. Offset Direction

Applies a positional offset to the final goal before docking.

| Keyword | Behavior                                                                                 |
|---------|------------------------------------------------------------------------------------------|
| `x`     | Shifts the docking goal along the X-axis. Useful to avoid obstructions or align with hardware. |
| `y`     | Shifts the docking goal along the Y-axis.                                               |
| `z`     | Shifts the docking goal along the Z-axis (if applicable, e.g., in 3D simulation or aerial robots). |

**Default:** z

---

### 5. Docking Style

Defines the overall docking behavior strategy.

| Keyword    | Behavior                                                                                  |
|------------|-------------------------------------------------------------------------------------------|
| `ordinary` | Default docking behavior with no special modifications.                                  |
| `gentle`   | Reduces velocity and acceleration limits for a smooth, impact-free docking. Useful for fragile connections. |
| `rush`     | Minimizes docking time by using aggressive motion and looser goal tolerances.             |

**Default:** ordinary

---

### 6. Special Control

Additional behaviors for special use.

| Keyword     | Behavior                                                                                                         | Notes                                                            |
|-------------|------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------|
| `delaySpin` | Introduces a deliberate spin delay with certain distance after initial movement. Useful to back up slightly to avoid collision while spinning. | The threshold is set in the `controller_server` section in the YAML file. |
| `nonStop`   | Robot continues the docking process without deceleration when transitioning from navigation to docking.         | Combine with `nonStopGoalChecker` for a relatively looser criteria. |
| `didilong`  | Navigates the robot at extreme speed, designed purely for navigation.                                            | Combine with `didilingController` and `didilongGoalChecker` for an extremely fast and loose process. |

**Default:** none

---
