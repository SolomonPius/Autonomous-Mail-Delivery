# Autonomous Mail-Delivery Robot with Bayesian Localization

**ROB301 – Introduction to Robotics | University of Toronto**  
**Abanoub Bashara & Solomon Pius | December 2025**

A fully autonomous **TurtleBot3 Waffle Pi** capable of mail delivery along a closed-loop hallway, starting from an **unknown location** with no GPS, odometry, or SLAM. Localization is achieved entirely through probabilistic reasoning over noisy vision-based observations.

---

## Table of Contents

- [Project Overview](#project-overview)
- [System Architecture](#system-architecture)
- [Components](#components)
  - [1. Perception & Line Following Control](#1-perception--line-following-control)
  - [2. Vision-Based Color Detection](#2-vision-based-color-detection)
  - [3. Bayesian Localization](#3-bayesian-localization)
  - [4. Hybrid State Machine & System Integration](#4-hybrid-state-machine--system-integration)
- [Results](#results)
- [Repository Structure](#repository-structure)
- [Dependencies & Setup](#dependencies--setup)

---

## Project Overview

This project was completed as the final design project for **ROB301 – Introduction to Robotics** at the **University of Toronto**. The objective was to design a mobile robot system capable of autonomous mail delivery along a closed-loop hallway, operating under significant sensing ambiguity and starting from a completely unknown position.

The robot was constrained to rely solely on:

- **Vision-based perception** (camera input only)
- **Line following** for navigation
- **Probabilistic reasoning** for localization

No geometric localization, wheel odometry, or mapping (SLAM) was permitted — making Bayesian inference the core of the system's intelligence.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Hybrid State Machine                         │
│              LINE mode  ←────────────→  PATCH mode              │
└────────────┬──────────────┬──────────────┬──────────────────────┘
             │              │              │
     ┌───────▼──────┐ ┌─────▼──────┐ ┌────▼───────────────┐
     │  PID Control │ │   RGB      │ │  Bayesian Filter   │
     │  /line_idx   │ │ Classifier │ │  (11-state belief) │
     └───────┬──────┘ └─────┬──────┘ └────┬───────────────┘
             │              │              │
     ┌───────▼──────────────▼──────────────▼───────────────┐
     │     Camera  →  /line_idx  +  /mean_img_rgb           │
     └──────────────────────────────────────────────────────┘
                                  │
                          ┌───────▼──────┐
                          │   /cmd_vel   │
                          │  (TurtleBot) │
                          └──────────────┘
```

The camera publishes to two ROS topics consumed separately: `/line_idx` drives the PID controller, and `/mean_img_rgb` drives colour classification. The state machine switches between **Line mode** (PID) and **Patch mode** (drive straight + Bayesian update) based on colour readings.

---

## Components

### 1. Perception & Line Following Control

The robot navigated a white tape path using a **PD controller** (integral term set to zero) driven by the `/line_idx` ROS topic, which provides the pixel column of the brightest point in the camera frame. The controller computed angular velocity corrections to drive the tape toward the centre of the 640px-wide frame (setpoint: pixel 320) while maintaining a constant forward speed of **0.04 m/s**.

**Controller equation:**

```
ω = Kp * e + Kd * (e - e_prev)

Kp = 0.005
Kd = 0.0005
Ki = 0.0  (disabled)
ω clipped to [-0.5, 0.5] rad/s
```

**Key design decisions:**

- **Integral term disabled** — avoids windup when the tape disappears under a coloured patch; accumulated error would cause violent corrections on re-entry
- **Output saturation** — angular velocity clipped to ±0.5 rad/s for stable motion
- **Tuning method** — Kp raised until oscillation onset (Kp_crit), then set to 0.5 × Kp_crit; Kd started at 0.1 × Kp and increased to damp oscillations

When a coloured patch is detected, **PID is disabled** and the robot drives straight at 0.04 m/s until the tape is re-acquired.

---

### 2. Vision-Based Colour Detection

A vision pipeline classified floor patches each frame using an **RGB Euclidean-distance classifier** against five manually calibrated colour centroids, published via `/mean_img_rgb`:

| Label  | R   | G   | B   |
|--------|-----|-----|-----|
| Orange | 249 | 145 | 89  |
| Green  | 170 | 178 | 169 |
| Blue   | 201 | 133 | 178 |
| Yellow | 194 | 174 | 159 |
| Line   | 145 | 129 | 130 |

The nearest centroid by Euclidean distance determines the current colour classification each frame.

**Mode-switching logic (debouncing):**

- **LINE → PATCH**: switches after just **1 patch reading** (fast response)
- **PATCH → LINE**: requires **3 consecutive line readings** (conservative, prevents premature PID re-engagement mid-patch)

**Noise-robust observation window:**

Because the camera consistently misread colours in the first frames over a patch (due to robot angle and lighting), the Bayesian update uses a **majority vote over frames 60–80** of each patch traversal. The first 59 readings are discarded; the most frequent colour in the next 20 readings becomes the official observation. This window consistently captured reliable colour readings in testing while still leaving sufficient time for a delivery manoeuvre if needed.

---

### 3. Bayesian Localization

Localization was implemented as a **discrete Bayesian filter** over **11 office states** (offices 2–12) arranged in a closed loop. The filter maintained a belief distribution and updated it on each patch entry via a **predict → update cycle**.

**Office colour map:**

| Office | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 |
|--------|---|---|---|---|---|---|---|---|----|----|----|
| Colour | 🟡 Y | 🟢 G | 🔵 B | 🟠 O | 🟠 O | 🟢 G | 🔵 B | 🟠 O | 🟡 Y | 🟢 G | 🔵 B |

Multiple offices share the same colour, making single observations ambiguous — the filter resolves this through accumulation across multiple patches.

**Prediction step** (transition model, `u=1` forward):

```
P(x_k | x_{k-1}, u) ≈  0.99  if moving to next state
                         ~0    otherwise (ε = e^-6)
```

**Update step** (measurement model likelihoods):

| True colour | Correct obs | Yellow | Orange | Green | Blue |
|-------------|-------------|--------|--------|-------|------|
| Blue        | 0.90        | 0.033  | 0.033  | 0.033 | —    |
| Green       | 0.70        | 0.20   | 0.10   | —     | 0.0  |
| Yellow      | 0.70        | —      | 0.20   | 0.10  | 0.0  |
| Orange      | 0.80        | 0.10   | —      | 0.10  | 0.0  |

These off-diagonal likelihoods reflect the real-world tendency for green/orange to be misread as yellow under variable lighting.

**Delivery trigger:** once the MAP estimate exceeds **p > 0.5** and matches a goal office, the delivery manoeuvre is initiated. Belief is published to the `/belief` topic each update and logged to `belief_log.csv`.

---

### 4. Hybrid State Machine & System Integration

The system uses a **two-mode state machine** inside a single ROS node (`LineFollowerColorAware`), with colour classification running every camera frame and localization triggered once per patch entry:

```
┌──────────┐   1 patch reading    ┌───────────┐
│          │ ──────────────────→  │           │
│   LINE   │                      │   PATCH   │
│  (PID)   │ ←──────────────────  │ (straight)│
└──────────┘   3 line readings    └─────┬─────┘
                                        │ frames 60–80 sampled
                                        ▼
                                  majority vote → z
                                        │
                                        ▼
                                  Bayes update
                                        │
                              MAP office + p > 0.5?
                                  AND goal office?
                                        │ YES
                                        ▼
                               DELIVERY MANOEUVRE
                          (fwd 2s → left spin 2s →
                           pause 2s → right spin 2s)
```

**Callbacks and update rates:**

| Callback | Trigger | Action |
|----------|---------|--------|
| `color_callback` | Every `/mean_img_rgb` frame | Classify colour, update mode |
| `line_callback` | Every `/line_idx` update | PID or straight drive, trigger Bayes update, check delivery |

**Edge cases handled:**

- Tape loss on patch exit — mitigated by ensuring near-perpendicular patch entry so straight driving keeps the robot on path
- Colour noise — resolved by the 60–80 frame observation window and majority vote
- Delivery de-duplication — `delivered_offices` set prevents re-triggering at already-served offices

---

## Results

The final demonstration started the robot on the boundary between offices 6 and 7 with delivery targets at **offices 2, 4, and 7**.

| Metric | Result |
|--------|--------|
| Full loop completion | ✅ 100% — 1 min 47 sec |
| Colour readings accuracy | ✅ Every patch read correctly |
| Observations to convergence | 4 patches (converged at office 10) |
| MAP confidence at convergence | 0.98 (office 2) |
| Deliveries completed | ✅ 3/3 (offices 2, 4, 7) |
| Human intervention | Minimal — self-corrected angle exit issues |

**Belief convergence trace (demo run):**

| # | Observed | MAP office | Confidence |
|---|----------|------------|------------|
| 1 | green    | 3          | 0.26       |
| 2 | blue     | 4          | 0.33       |
| 3 | orange   | 5          | 0.44       |
| 4 | yellow   | 10         | 0.82       |
| 5 | green    | 11         | 0.87       |
| 6 | blue     | 12         | 0.87       |
| 7 | yellow   | **2**      | **0.98** ← delivery triggered |

After convergence at observation 7, the filter maintained certainty (p=1.0) for all subsequent patches.

---

## Repository Structure

```
├── README.md
└── final_project.py     # Single ROS node — all logic in LineFollowerColorAware class
                         #   - PID line follower      (follow_line_pid, line_callback)
                         #   - RGB colour classifier  (color_callback)
                         #   - Bayesian filter        (bayes_update, transition_prob, measurement_prob)
                         #   - Delivery controller    (start_delivery, delivery_step)
                         #   - Belief CSV logger      → belief_log.csv
```

ROS node name: `color_line_follower_rgb_bayes_delivery`

---

## Dependencies & Setup

**Hardware:**
- TurtleBot3 Waffle Pi (Raspberry Pi 3B+, OpenCR motor controller, downward-facing RGB camera)
- White tape loop environment with coloured floor patches (blue, green, yellow, orange)

**Software:**
- ROS (tested on the TurtleBot3 standard ROS environment)
- Python 2/3 (rospy)
- NumPy

**ROS topics consumed:**

| Topic | Type | Description |
|-------|------|-------------|
| `/mean_img_rgb` | `Float64MultiArray` | Mean RGB of current camera frame |
| `/line_idx` | `UInt32` | Pixel column of the brightest (tape) point |

**ROS topics published:**

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `Twist` | Velocity commands to the robot |
| `/belief` | `Float64MultiArray` | Current belief distribution over 11 offices |

**Running the node:**
```bash
# Clone the repository
git clone https://github.com/<your-username>/<repo-name>.git
cd <repo-name>

# Launch on the TurtleBot3
rosrun <your-package-name> final_project.py

# Optional: specify a custom path for the belief log CSV
rosrun <your-package-name> final_project.py _belief_log_path:=/path/to/belief_log.csv
```

The node logs every Bayesian update to `belief_log.csv` in the working directory, capturing the observed colour, MAP office estimate, confidence, and full belief distribution for each patch.

---

*University of Toronto · ROB301 Introduction to Robotics · Final Project · Abanoub Bashara & Solomon Pius · December 2025*

<img width="454" height="464" alt="Image" src="https://github.com/user-attachments/assets/72c4be83-17b4-48b6-9cbd-5ad48b1d3aae" />
