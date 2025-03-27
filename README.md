# Arrowbotics (3482) Reefscape 2025 Code for `Ceto`

#### Statistics:

![GitHub Commit Activity](https://img.shields.io/github/commit-activity/t/team-3482/Reefscape2025-Ceto?style=flat-square)
![Github Started](https://img.shields.io/github/created-at/team-3482/Reefscape2025-Ceto?style=flat-square&label=started)
![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/team-3482/Reefscape2025-Ceto/gradle.yml?style=flat-square)
![GitHub License](https://img.shields.io/github/license/team-3482/Reefscape2025-Ceto?style=flat-square)
![GitHub repo size](https://img.shields.io/github/repo-size/team-3482/Reefscape2025-Ceto?style=flat-square)
![GitHub Top Language](https://img.shields.io/github/languages/top/team-3482/Reefscape2025-Ceto?style=flat-square)

#### Dependencies:

![WPILib Version](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fraw.githubusercontent.com%2Fteam-3482%2FReefscape2025%2Fmain%2F.wpilib%2Fwpilib_preferences.json&query=%24.projectYear&style=flat-square&label=WPILib&color=%23AC2B37)
![Phoenix 6 Version](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fraw.githubusercontent.com%2Fteam-3482%2FReefscape2025%2Fmain%2Fvendordeps%2FPhoenix6-frc2025-latest.json&query=%24.version&style=flat-square&label=Phoenix%206&color=%2396C93D)
![PathplannerLib Version](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fraw.githubusercontent.com%2Fteam-3482%2FReefscape2025%2Fmain%2Fvendordeps%2FPathplannerLib.json&query=%24.version&style=flat-square&label=PathplannerLib&color=%233A51BB)
![AdvantageKit Version](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fraw.githubusercontent.com%2Fteam-3482%2FReefscape2025%2Frefs%2Fheads%2Fmain%2Fvendordeps%2FAdvantageKit.json&query=version&style=flat-square&label=AdvantageKit&color=%23fbc404)

This code is for team 3482's on-season robot, Ceto.

<hr>

## Capabilities - Scouts, look here!

### Reef

| Task         | L1 | L2 | L3 | L4 |
|--------------|:--:|:--:|:--:|:--:|
| Score Coral  | ✅ | ✅ | ✅ | ❌ |
| Remove Algae | -  | ✅ | ✅ | -  |

⚠️ `Coral` is sourced from `Coral Stations`, not the ground.<br>
⚠️ `Algae` is removed from the reef and ejected directly onto the ground.

### Barge

| Park | Shallow Climb | Deep Climb |
|:----:|:-------------:|:----------:|
|  ✅  |       ❌      |     ❌     |

### Other

| Task                       | Capable |
|----------------------------|:-------:|
| Leave Starting Line (Auto) |   ✅    |
| Processor                  |   ❌    |

🛈 `Ceto` scores **2** `L3 Coral` during the Autonomous period on the left or right side.<br>
It is also capable of scoring **1** `L3 Coral` starting from the middle and staying out of the way of other autons.<br>
`Ceto` autonomously removes algae from any side of the reef it scores on.

### Design

|  Dimensions  | Height |   Swerve   |      Vison      | Top Speed |           Sensors            |
|--------------|--------|------------|-----------------|-----------| -----------------------------|
| 27 x 27 in   | 35 in  | Kraken X60 | 2x Limelight 3G | 4.0 m/s   | Beam-break, CTRE Talon tachs |

If you have any additional questions about `Ceto`, feel free to talk to us!

<hr>

## Resources

### LED Status Codes

| Color     | Status                     |
|-----------|----------------------------|
| ⚫ Off    | Standby (enabled)          |
| 🔴 Red    | Error                      |
| 🟠 Orange | Disabled                   |
| 🟢 Green  | Completed OK               |
| 🔵 Blue   | Reef/Processor Tag In View |
| ⚪ White  | Has Coral                  |

### Autonomous Paths

![Path](https://github.com/user-attachments/assets/75e373d9-f89a-46b4-9048-9185e126dbae)

This shows our preferred autonomous path. It can be ran on the left or right side.
`Ceto` is capable of scoring 2 `L3 Coral` and either picking up, or discarding 1 `Algae`, in about ~13.5 seconds

#### Drop Algae 

1. Starting Line -> `KL` or `CD`
2. Score `Coral` on `K` or `D`
3. Pick up `Algae`
4. Drive to `Coral Station` and drop `Algae` halfway through
5. Intake `Coral`
6. Drive back to `KL` or `CD`
7. Score `Coral` on `L` or `C`
8. Drive back to `Coral Station` to be ready for Teleop

#### Hold Algae

1. Starting Line -> `KL` or `CD`
2. Score `Coral` on `K` or `D`
3. Drive to `Coral Station`
4. Intake `Coral`
5. Drive back to `KL` or `CD`
6. Score `Coral` on `L` or `C`
7. Pick up `Algae`
8. Drive to `Barge` to be ready for Teleop

https://github.com/user-attachments/assets/2b92c98b-00dd-4389-8422-ad95c4a4d2f4

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
