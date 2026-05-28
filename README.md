# Autonomous-Mail-Delivery
Autonomous mail-delivery robot built using PID line-following, vision-based color detection, and a discrete Bayesian filter for probabilistic localization across 11 office states — no odometry or SLAM. Coordinated via a hybrid state machine; achieved >95% localization accuracy in physical testing.
# Autonomous Mail-Delivery Robot with Bayesian Localization

**ROB301 – Introduction to Robotics | University of Toronto**

A fully autonomous mobile robot capable of mail delivery along a closed-loop hallway, starting from an **unknown location** with no GPS, odometry, or SLAM. Localization is achieved entirely through probabilistic reasoning over noisy vision-based observations.

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
│   (coordinates perception, estimation, control, and delivery)    │
└────────────┬──────────────┬──────────────┬──────────────────────┘
             │              │              │
     ┌───────▼──────┐ ┌─────▼──────┐ ┌────▼───────────────┐
     │  Line Follow │ │   Color    │ │  Bayesian Filter   │
     │  PID Control │ │ Detection  │ │  (11-state belief) │
     └───────┬──────┘ └─────┬──────┘ └────┬───────────────┘
             │              │              │
     ┌───────▼──────────────▼──────────────▼───────────────┐
     │                  Camera Input                        │
     └──────────────────────────────────────────────────────┘
```

All sensing flows from a single camera. The state machine orchestrates how perception, estimation, and actuation interact in real time.

---

## Components

### 1. Perception & Line Following Control

The robot navigated a white tape path using a **PID-based line-following controller** driven entirely by camera input. The controller continuously computed angular velocity corrections based on the lateral deviation of the tape in the camera frame, while maintaining a constant forward velocity.

**Key design decisions:**

- **Integral term disabled** — prevented windup when the tape temporarily disappeared from view
- **Output saturation** — ensured stable motion and prevented overcorrection
- **Carefully tuned P and D gains** — minimized oscillations while maintaining responsiveness to path curvature

This low-level controller provided stable navigation around the entire loop, even under imperfect lighting and minor alignment errors.

---

### 2. Vision-Based Color Detection

A vision pipeline classified floor patches using an **RGB-distance–based color classifier**, distinguishing between the white tape and multiple color patches that serve as positional landmarks. A key challenge was that several locations shared similar or identical colors, creating deliberate ambiguity that required the probabilistic layer to resolve.

**Robustness measures:**

- Color detection evaluated **continuously** rather than at discrete trigger points
- **Conservative mode-switching logic** — stable entry/exit conditions to prevent jitter between states
- **Multi-frame aggregation** — color readings accumulated over several frames before being used for a localization update

This approach significantly reduced false detections and improved performance under real-world lighting variation.

---

### 3. Bayesian Localization

Localization was implemented as a **discrete Bayesian filter** over **11 possible office states**, representing each distinct location along the delivery loop.

The filter maintained a probability distribution (belief) over all states and updated it through a standard **predict → update cycle**:

| Step | Description |
|------|-------------|
| **Predict** | Propagates belief forward using a transition model of motion between adjacent locations |
| **Update** | Incorporates new color observations via a measurement likelihood model |

**Key properties:**

- **No odometry required** — motion is modeled implicitly through the loop topology
- **Ambiguity tolerance** — multiple locations sharing the same color are resolved over successive observations
- **Confidence thresholding** — delivery actions are only initiated once the belief in a single state exceeds a set threshold

The filter converged to the correct location after only a small number of observations, even when initial observations were ambiguous or noisy.

---

### 4. Hybrid State Machine & System Integration

The full system was organized as a **hybrid state machine** with clearly separated operational modes:

```
INIT → LINE_FOLLOW → PATCH_DETECTED → UPDATE_BELIEF → [LOCALIZED?]
                                                            │
                                              YES ──────────▼
                                                      DELIVERY_MANEUVER
                                              NO  → continue loop
```

**State responsibilities:**

- `LINE_FOLLOW` — continuous PID control along the tape path
- `PATCH_DETECTED` — triggers color classification and frame aggregation
- `UPDATE_BELIEF` — runs the Bayesian predict-update step
- `DELIVERY_MANEUVER` — executes the delivery action once localization confidence is met

**Edge cases explicitly handled:**

- Tape loss during navigation
- Noisy or conflicting color readings
- Misalignment recovery
- Fallback behaviors when belief fails to converge

---

## Results

| Metric | Result |
|--------|--------|
| Localization accuracy | >95% under noisy sensing conditions |
| Observations to convergence | Small number (typically a few patches) |
| Delivery success | Multiple target locations in a single run |
| Human intervention required | Minimal — full loop completed autonomously |

The system was validated in both simulation and physical hardware testing. The Bayesian filter consistently converged to the correct office location, and the robot successfully completed autonomous delivery runs without requiring manual correction.

---

## Repository Structure

```
├── README.md
└── src/
    ├── main.py                  # Entry point and state machine
    ├── line_follower.py         # PID control and tape tracking
    ├── color_detector.py        # RGB color classification pipeline
    ├── bayesian_localizer.py    # Discrete Bayesian filter
    └── config.py                # Tuning parameters and constants
```

> **Note:** Source code will be added to this repository shortly.

---

## Dependencies & Setup

**Hardware:**
- Mobile robot platform with front-facing camera
- White tape loop environment with colored floor patches

**Software:**
- Python 3.x
- OpenCV (vision pipeline)
- NumPy (Bayesian filter computations)
- Robot-specific motor control library (platform-dependent)

**Running the system:**
```bash
# Clone the repository
git clone https://github.com/<your-username>/<repo-name>.git
cd <repo-name>

# Install dependencies
pip install opencv-python numpy

# Launch the robot controller
python src/main.py
```

> Configuration parameters (PID gains, color thresholds, belief threshold) are set in `src/config.py`.

---

*University of Toronto · ROB301 Introduction to Robotics · Final Project*
