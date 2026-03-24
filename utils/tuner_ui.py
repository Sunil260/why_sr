'''Tuner UI for live PID parameter adjustment and performance visualization.'''

from fasthtml.common import *
import pandas as pd
import plotly.express as px

app, rt = fast_app(hdrs=(SortableJS,))

@rt("/")
def get():
    # PID Controls + Live Chart
    return Titled("Pi 5 Drive Tuner",
        Div(hx_get="/chart", hx_trigger="every 0.5s"), # Auto-refreshing chart
        Form(hx_post="/update", hx_target="#msg")(
            Label(f"Kp: "), 
            Input(type="range", name="kp", min="0", max="5", step="0.1", value=pid_l.kp, hx_post="/update"),
            Label(f"Target Speed (m/s): "),
            Input(type="number", name="target", step="0.05", value=state.target_ms, hx_post="/update"),
            Div(id="msg")("Adjust sliders to tune live")
        )
    )

@rt("/chart")
def get_chart():
    if not state.history: return "Waiting for data..."
    df = pd.DataFrame(state.history)
    fig = px.line(df, x="t", y=["target", "actual"], title="Velocity Tracking")
    return NotStr(fig.to_html(full_html=False, include_plotlyjs='cdn'))

@rt("/update")
def post(kp: float = None, target: float = None):
    if kp is not None: pid_l.update_params(kp=kp); pid_r.update_params(kp=kp)
    if target is not None: state.target_ms = target
    return f"Updated: Kp={kp}, Target={target}"