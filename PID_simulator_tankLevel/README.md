# PID Control Simulator — Tank Level

Interactive web app for tuning a PID controller on a nonlinear tank model.
Move the gains and watch the response and the control effort update instantly.

**[Live demo](https://pid-tank-level-nathalia.streamlit.app/)**

## What it does

A pump fills a tank while liquid drains out through the outlet. The controller adjusts
the pump flow to hold the level at a setpoint, even when an external disturbance
increases the outflow.

Everything runs in the browser. Move a slider and the whole page recalculates in
milliseconds — no simulation software required.

## The model

The plant is nonlinear, because the outflow depends on the square root of the level:

$$A \frac{dh}{dt} = u - k\sqrt{h} - q_{dist}$$

- $h$ — tank level (m)
- $A$ — tank cross-sectional area (m²)
- $u$ — pump flow, the control signal (m³/s)
- $k$ — outflow coefficient
- $q_{dist}$ — extra outflow used as a disturbance (m³/s)

The controller is a standard parallel PID:

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \frac{de(t)}{dt}$$

with $e(t) = h_{sp} - h(t)$.

**Actuator limits.** The pump cannot deliver negative flow and cannot exceed a
maximum, so the control signal is clamped to $u \in [0, q_{max}]$. This is where
the anti-windup logic matters.

## Anti-windup

When the pump saturates, the level error keeps accumulating in the integral term
even though the actuator cannot respond. The integral grows without bound and the
controller overshoots badly once the tank finally reaches the setpoint.

This app uses **conditional integration**: the integral is only updated when the
control signal is not saturated. Turn the anti-windup checkbox off and push the
tank into saturation to see the difference in the overshoot value.

## Derivative handling

The derivative term is the most fragile of the three, and two separate problems
show up in practice. The first is measurement noise amplification: differentiating
a noisy signal produces large swings in the control effort. The second is the
*derivative kick* — a step change in the setpoint enters the derivative as an
impulse and triggers a violent, instantaneous control action.

Both are handled the way industrial controllers handle them:

- **Derivative on measurement.** The derivative acts on the level, not on the
  error, so a setpoint change no longer produces a kick.
- **Low-pass filtering.** The result passes through a first-order filter with time
  constant $\tau_f$, which attenuates high-frequency content:

$$u_d(t) = K_d \cdot \frac{de}{dt} \cdot \frac{1}{\tau_f s + 1}$$

The filter time constant is exposed as a slider. Set it to `0` to recover the raw,
unfiltered behaviour. The usual rule of thumb is to choose $\tau_f$ between one
fifth and one tenth of the plant time constant.

## Features

- PID gains (Kp, Ki, Kd) tuned with sliders
- Derivative filter time constant τf, from 0 (off) to 2 s
- Anti-windup on/off
- Process parameters: setpoint, tank area, outflow coefficient, max pump flow
- Configurable step disturbance, with its magnitude and start time
- Live performance metrics: overshoot, settling time (2% band), rise time (10–90%),
  steady-state error
- Level and control signal plotted over time, with the saturation limit marked

## Tech stack

- **Python** — simulation loop and controller
- **NumPy** — time vector and state arrays
- **Streamlit** — interface, sliders and layout
- **Plotly** — interactive charts

No external API, database or hardware.

## Getting started

```bash
git clone [github.com](https://github.com/nathaliagcostaeng/my-projects.git)
cd my-projects/PID_simulator_tankLevel
python -m venv .venv

# Windows (PowerShell)
.venv\Scripts\Activate.ps1
# Linux / macOS
source .venv/bin/activate

pip install -r requirements.txt
streamlit run app.py
