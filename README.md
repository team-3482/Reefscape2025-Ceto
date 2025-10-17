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

<img width="350" height="350" alt="ceto" src="https://github.com/user-attachments/assets/98503e05-2517-48c2-9c60-b16aa41e0cf8" />

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
| Score in Processor         |   ❌    |
| Score in Net               |   ❌    |


🛈 `Ceto` scores **3** `L3 Coral` during the Autonomous period on the left or right side.<br>
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

| Color     | Status            |
|-----------|-------------------|
| ⚫ Off    | Standby (enabled) |
| 🔴 Red    | Error             |
| 🟠 Orange | Disabled          |
| 🟢 Green  | Completed OK      |
| 🔵 Blue   | Reef Tag In View  |
| ⚪ White  | Has Coral         |

### Autonomous Paths

![path](https://github.com/user-attachments/assets/8024ac9d-7fda-443e-8e0b-239d16ffaa52)

This shows our preferred autonomous path. It can be ran on the left or right side.
`Ceto` is capable of scoring 3 `L3 Coral` in 14±0.5 seconds consistently. It removes Algae automatically.

First coral
1. Starting Line -> `J`
2. Remove `L3 Algae` & score `L3 Coral`
3. `J` -> `Coral Station` and intake

Second coral
1. `Coral station` -> `K`
2. Remove `L2 Algae` & score `L3 Coral`
3. `K` -> `Coral Station` and intake

Third coral
1. `Coral station` -> `L`
2. Score `L3 Coral`<br>
This runs during any leftover time during auton : 
3. `L` -> `Coral Station` and intake

https://github.com/user-attachments/assets/74acafa7-0642-4a69-975c-721236e07aed

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
