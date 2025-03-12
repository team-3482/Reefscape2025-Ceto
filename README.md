# Arrowbotics (3482) Reefscape 2025 Code for `Ceto`

#### Statistics:

![GitHub commit activity](https://img.shields.io/github/commit-activity/t/team-3482/Reefscape2025-Ceto?style=flat-square)
![Github Started](https://img.shields.io/github/created-at/team-3482/Reefscape2025-Ceto?style=flat-square&label=started)
![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/team-3482/Reefscape2025-Ceto/gradle.yml?style=flat-square)
![GitHub License](https://img.shields.io/github/license/team-3482/Reefscape2025-Ceto?style=flat-square)
![GitHub repo size](https://img.shields.io/github/repo-size/team-3482/Reefscape2025-Ceto?style=flat-square)
![GitHub top language](https://img.shields.io/github/languages/top/team-3482/Reefscape2025-Ceto?style=flat-square)

#### Dependencies:

![WPILib Version](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fraw.githubusercontent.com%2Fteam-3482%2FReefscape2025-Ceto%2Fmain%2F.wpilib%2Fwpilib_preferences.json&query=%24.projectYear&style=flat-square&label=WPILib&color=%23AC2B37)
![Phoenix 6 Version](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fraw.githubusercontent.com%2Fteam-3482%2FReefscape2025-Ceto%2Fmain%2Fvendordeps%2FPhoenix6-frc2025-latest.json&query=%24.version&style=flat-square&label=Phoenix%206&color=%2396C93D)
![PathplannerLib Version](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fraw.githubusercontent.com%2Fteam-3482%2FReefscape2025-Ceto%2Fmain%2Fvendordeps%2FPathplannerLib.json&query=%24.version&style=flat-square&label=PathplannerLib&color=%233A51BB)

This code is for team 3482's on-season robot, Ceto.

<hr>

## Capabilities 

Scouts, look here!

### Reef

| Task  | L1 | L2 | L3 | L4 |
|-------|:--:|:--:|:--:|:--:|
| Coral | ✅ | ✅ | ✅ | ❌ |
| Algae | -  | ✅ | ❌ | -  |

### Barge

| Park | Shallow Climb | Deep Climb |
|:----:|:-------------:|:----------:|
|  ✅  |       ❌      |     ❌     |

### Other

| Task                       | Capable |
|----------------------------|:-------:|
| Leave Starting Line (Auto) |   ✅    |
| Processor                  |   ✅    |

<br>

🛈 `Ceto` is capable of scoring 2 `L3 (or lower) Coral` in the Autonomous period on the left or right side. It can either score 1 `Algae` once Teleop starts, or discard it to start Teleop at a `Coral Station`.<br>
<sub>It is also capable of scoring 1 `L3 (or lower) Coral` if starting from the middle and staying out of the way of alliance bots running autons on the left or right, if requested.</sub>


⚠️ The `Coral` subsystem can only intake from the `Coral Stations`, and not the ground.

⚠️ The `Algae` subsystem can only intake from the `Reef`, and not the ground.

<br>

If you have any additional questions about `Ceto`, feel free to talk to us!

<hr>

## Resources

### LED Status Codes

| Color     | Status                     |
|-----------|----------------------------|
| ⚫ Off    | Standby                    |
| 🔴 Red    | Error                      |
| 🟠 Orange | Disabled                   |
| 🟢 Green  | OK                         |
| 🔵 Blue   | Reef/Processor Tag In View |
| ⚪ White  | Has Coral                  |

### Autonomous Paths

TBA

### Controls

<details>
  <summary>Controller Bindings</summary>
  <img src="https://docs.google.com/drawings/d/e/2PACX-1vSC9Kgz5UuIplrKstMqQF4jVtzlN4xEv1x5urSxMqhfPQsvJs29qJOLpVRK4puhl9MaWH_dZFEPxZpH/pub?w=1440&h=1440" width="720" alt="controller bindings"/>
</details>

<hr>

## Contributors

<a href="https://github.com/team-3482/Reefscape2025-Ceto/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=team-3482/Reefscape2025-Ceto" alt="contributors"/>
</a>
