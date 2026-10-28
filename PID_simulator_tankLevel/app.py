import numpy as np
import plotly.graph_objects as go
import streamlit as st
from plotly.subplots import make_subplots

st.set_page_config(page_title="PID Simulator - Tank Level", layout="wide")


def simulate(kp, ki, kd, setpoint, t_final, dt, area, k_out, q_max, t_dist, q_dist, anti_windup, tau_f):
    n = int(t_final / dt)
    t = np.arange(n) * dt
    h = np.zeros(n)
    u = np.zeros(n)
    integral = 0.0
    prev_error = setpoint - h[0]
    derivative_meas = 0.0
    alpha = dt / (tau_f + dt)  # first-order low-pass filter coefficient

    for i in range(1, n):
        error = setpoint - h[i - 1]

        # Derivative on measurement, filtered: avoids the startup spike and the
        # derivative kick on a setpoint step change
        d_meas_raw = (h[i - 1] - h[i - 2]) / dt if i >= 2 else 0.0
        derivative_meas += alpha * (d_meas_raw - derivative_meas)

        u_raw = kp * error + ki * integral - kd * derivative_meas
        u_sat = np.clip(u_raw, 0, q_max)  # pump cannot have negative or infinite flow

        # Anti-windup via conditional integration
        if not anti_windup or u_raw == u_sat:
            integral += error * dt
        prev_error = error

        disturbance = q_dist if t[i] >= t_dist else 0.0
        dh = (u_sat - k_out * np.sqrt(max(h[i - 1], 0)) - disturbance) / area
        h[i] = max(h[i - 1] + dh * dt, 0)
        u[i] = u_sat

    return t, h, u


def compute_metrics(t, h, setpoint):
    peak = h.max()
    overshoot = max(0.0, (peak - setpoint) / setpoint * 100)

    outside = np.where(np.abs(h - setpoint) > 0.02 * setpoint)[0]  # 2% criterion
    if len(outside) == 0:
        settling_time = 0.0
    elif outside[-1] == len(t) - 1:
        settling_time = float("nan")  # did not settle
    else:
        settling_time = t[outside[-1] + 1]

    i10 = np.argmax(h >= 0.1 * setpoint)
    i90 = np.argmax(h >= 0.9 * setpoint)
    rise_time = t[i90] - t[i10] if h.max() >= 0.9 * setpoint else float("nan")

    steady_state_error = abs(setpoint - h[-1])
    return overshoot, settling_time, rise_time, steady_state_error


# ---------------- User Interface ----------------
st.title("💧 PID Control Simulator — Tank Level")
st.caption("Nonlinear model: A·dh/dt = u − k·√h − q_disturbance")

with st.sidebar:
    st.header("PID Controller")
    kp = st.slider("Kp", 0.0, 20.0, 5.0, 0.1)
    ki = st.slider("Ki", 0.0, 5.0, 0.5, 0.05)
    kd = st.slider("Kd", 0.0, 20.0, 1.0, 0.1)
    anti_windup = st.checkbox("Anti-windup", value=True)
    tau_f = st.slider("Derivative filter τf (s)", 0.0, 2.0, 0.5, 0.05)

    st.header("Process")
    setpoint = st.slider("Level setpoint (m)", 0.5, 5.0, 2.0, 0.1)
    area = st.slider("Tank area (m²)", 0.5, 10.0, 2.0, 0.1)
    k_out = st.slider("Outflow coefficient k", 0.1, 3.0, 1.0, 0.1)
    q_max = st.slider("Max pump flow (m³/s)", 1.0, 20.0, 5.0, 0.5)

    st.header("Disturbance")
    t_dist = st.slider("Time (s)", 0, 300, 150)
    q_dist = st.slider("Extra outflow (m³/s)", 0.0, 3.0, 0.5, 0.1)

    t_final = st.slider("Simulation time (s)", 50, 400, 300, 10)

t, h, u = simulate(kp, ki, kd, setpoint, t_final, 0.05, area, k_out, q_max, t_dist, q_dist, anti_windup, tau_f)

# Metrics computed only before the disturbance (step response)
mask = t < t_dist if t_dist > 0 else np.ones_like(t, bool)
ov, ts, tr, ess = compute_metrics(t[mask], h[mask], setpoint)

c1, c2, c3, c4 = st.columns(4)
c1.metric("Overshoot", f"{ov:.1f} %")
c2.metric("Settling time (2%)", "did not settle" if np.isnan(ts) else f"{ts:.1f} s")
c3.metric("Rise time (10–90%)", "—" if np.isnan(tr) else f"{tr:.1f} s")
c4.metric("Steady-state error", f"{ess:.3f} m")

fig = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.08,
                    subplot_titles=("Tank level", "Control signal (pump flow)"))
fig.add_trace(go.Scatter(x=t, y=h, name="Level h(t)", line=dict(width=2)), row=1, col=1)
fig.add_hline(y=setpoint, line_dash="dash", line_color="green", annotation_text="Setpoint", row=1, col=1)
fig.add_trace(go.Scatter(x=t, y=u, name="u(t)", line=dict(color="orange")), row=2, col=1)
fig.add_hline(y=q_max, line_dash="dot", line_color="red", annotation_text="Saturation", row=2, col=1)
if t_dist < t_final:
    fig.add_vline(x=t_dist, line_dash="dot", line_color="gray")
fig.update_yaxes(title_text="m", row=1, col=1)
fig.update_yaxes(title_text="m³/s", row=2, col=1)
fig.update_xaxes(title_text="Time (s)", row=2, col=1)
fig.update_layout(height=650, showlegend=False)
st.plotly_chart(fig, use_container_width=True)

with st.expander("How to interpret"):
    st.markdown("""
- **High Kp**: faster response, but more overshoot.
- **Ki** eliminates steady-state error, but too much causes oscillation.
- **Kd** dampens the response and reduces overshoot.
- **Derivative filter τf**: smooths the derivative. Set it to 0 to see the raw,
  noisy behaviour; the industrial default is roughly 1/5 to 1/10 of the plant time constant.
- **Anti-windup:** turn it off while the pump is saturated and watch the overshoot grow.
- The **disturbance** shows the controller's ability to reject external disturbances.
""")

