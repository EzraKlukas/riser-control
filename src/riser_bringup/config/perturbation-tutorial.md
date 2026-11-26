# AP torque perturbation experiment

This experiment adds a synthetic moment to the ankle wrench seen by the
inverted pendulum. Positive `torque.x` drives positive backboard pitch in the
sagittal model. The participant responds to the resulting motion with its
ankle controller. The perturbation generator sends no commands to Gazebo:
it calculates `torque.x = measured_and_tared_x + multisine + csv`. Both tares
and weighing use the measured wrench before either contribution is added.

## Configure the trial

Build and source the workspace. Make a trial copy of the controller parameters
with a 5 N m peak bound and five evenly spaced frequencies from 0.25 to 1.75 Hz.
The included CSV is a 1 s, 12 N m peak pulse at its original sample interval.
For a more visible Gazebo demonstration, the trial below uses the same
shape at 30 N m over 4 simulated seconds.

```bash
share="$(ros2 pkg prefix --share riser_bringup)"
trial_config="$(mktemp --suffix=.yaml /tmp/riser-perturbations.XXXXXX)"
python3 - "$share" "$trial_config" <<'PY'
from pathlib import Path
import sys
import yaml

share, output = map(Path, sys.argv[1:])
config = yaml.safe_load((share / "config/controllers.yaml").read_text())
peak_nm, duration_s = 30.0, 4.0
samples = [float(line) for line in (share / "config/ap_perturbation.csv").read_text().splitlines()]
pulse = output.with_suffix(".csv")
pulse.write_text("".join(f"{value * peak_nm / 12.0:.6f}\n" for value in samples))
config["ankle_wrench"]["ros__parameters"]["perturbation"].update(
    multisine_peak_nm=5.0,
    multisine_low_hz=0.25,
    multisine_high_hz=1.75,
    csv_path=str(pulse),
    csv_sample_period_s=duration_s / (len(samples) - 1),
)
output.write_text(yaml.safe_dump(config))
PY
ros2 launch riser_bringup control.launch.py backend:=gazebo \
  human:=true \
  controllers_file:="$trial_config" plot:=true wrench_input:=false
```

For a headless trial use `headless:=true plot:=false plot_telemetry:=true`.
The CSV format is one finite torque in N m per line, without a header or time
column. Use an absolute path, specify the interval in `csv_sample_period_s`,
and put zero on both the first and last lines. The controller validates and
loads the file at configuration, then interpolates the samples against its read
clock. Changing the file or its settings requires reconfiguring the controller.

## Start from the empty platform

Wait for the platform to settle at its starting pose. In a sourced
terminal, tare the empty plates, spawn the participant, weigh it, and tare the
standing wrench. Wait for each service to succeed before continuing. Spawn
without the participant's optional ankle pulse to isolate the sensor perturbations.

```bash
ros2 service call /ankle_wrench/tare std_srvs/srv/SetBool '{data: true}'
ros2 run riser_human_balance spawn_participant
ros2 service call /ankle_wrench/measure_mass std_srvs/srv/Trigger '{}'
ros2 param get /ankle_wrench subject_mass_kg
ros2 service call /ankle_wrench/tare std_srvs/srv/SetBool '{data: false}'

ros2 run controller_manager spawner inverted_pendulum --inactive \
  --param-file "$(ros2 pkg prefix --share riser_human_balance)/config/pendulum.yaml"
ros2 control switch_controllers --strict \
  --deactivate riser_position_controller --activate inverted_pendulum
```

## Add the perturbations

With the participant balancing, enable the multisine and watch the difference
between the measured and exported torques. It fades in over 0.5 s. Run it for
several seconds, then fade it out and wait for the body to settle. Play the CSV
and watch the participant oppose the positive torque pulse and return upright.

```bash
ros2 service call /ankle_wrench/set_multisine std_srvs/srv/SetBool '{data: true}'
# Observe the multisine, then disable it.
ros2 service call /ankle_wrench/set_multisine std_srvs/srv/SetBool '{data: false}'
# Wait for the stance to settle.
ros2 service call /ankle_wrench/play_perturbation std_srvs/srv/Trigger '{}'
```

Leaving the multisine enabled when you call `play_perturbation` adds both
signals. On the next update, valid feedback starts playback at sample zero.
Playback runs once and ends at zero. Another request is accepted after it finishes.
Invalid input or controller deactivation cancels playback. Both signals
start disabled when the controller activates.

## Watch the torque signals in PlotJuggler

Load `config/ap_perturbation_plot.xml` from the installed `riser_bringup`
share directory for a two-curve torque panel. Blue is the pre-perturbation
signal and red is the torque exported to the pendulum. If starting the stream
restores the app's previous buffer size, set **Buffer** to 90 seconds to keep
the full trace in view.

To set it up manually, select **ROS2 Topic Subscriber** and subscribe to
`/riser/plot/ankle_before_perturbation` and `/riser/plot/ankle_exported`.
Expand `wrench/torque/x` under each topic and drag both curves onto one plot.
The display node resolves interfaces by name and republishes two
`geometry_msgs/WrenchStamped` messages with the same timestamp. The input
broadcaster runs at 25 Hz with the default controller manager settings.

```bash
ros2 topic echo --once /riser/plot/ankle_before_perturbation
ros2 topic echo --once /riser/plot/ankle_exported
```

Before injecting anything the traces overlap. Their difference is the injected
moment. Both stamped wrench topics include vertical force. Measured mass,
validity and hardware read time remain available on
`/ankle_wrench_plot_broadcaster/dynamic_joint_states`.

## Introspect another intermediate value

In `wrench_aggregation_logic.hpp`, give the value stable member storage, assign
it where it is computed, and add it to `register_introspection()`. The
pre-perturbation torque uses:

```cpp
register_value("torque_x_before_perturbation", &torque_x_before_nm_);
```

The controller supplies that callable during configuration. It invokes
`REGISTER_ROS2_CONTROL_INTROSPECTION(name, value)` in the controller's scope,
where ros2_control provides its registry bookkeeping. Keep registered values as
members so their addresses survive every update. Once the controller is active,
you can find `ankle_wrench.torque_x_before_perturbation` in the registry:

```bash
ros2 topic echo --once --qos-durability transient_local \
  /controller_manager/introspection_data/names
```

For a short raw inspection, PlotJuggler can also subscribe to the native
`/controller_manager/introspection_data/names` and `/controller_manager/introspection_data/values`
pair. The registry stream runs at the controller-manager rate. For continuous
live plotting, use the named 25 Hz wrench topics. The pre-perturbation topic
reads the same registered member through the plot broadcaster. To expose a
new variable on that broadcaster, export a state interface for its member and
add the name to the broadcaster's interface list. [ros2_control Jazzy introspection guide](https://control.ros.org/jazzy/doc/ros2_control/doc/introspection.html)
