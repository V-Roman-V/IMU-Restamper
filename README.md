# imu_restamper

ROS 2 node that republishes an IMU stream with **corrected timestamps**.

Some drivers stamp IMU samples with host time when they get to them, not when the sensor took them. The stamps then
jitter, and after a driver stall a burst of queued samples arrives almost at once. This breaks LIO pipelines that
trust `header.stamp`. The sensor itself samples on a regular clock, so the node puts every sample back on that grid.

Example: the Livox Mid-360 IMU on a Unitree G1 (vendor driver, 200 Hz). Raw stamp spacing is 0.06–17 ms, with
stalls of up to 14 ms at every LiDAR scan. The output is on the 5 ms grid with jitter below 0.2 ms, and adds about
0.1 ms latency on average.

## How it works

The stamping delay is never negative, so the grid is kept on the **lower envelope** of the stamps by a one-sided
running minimum (passive synchronization, Olson 2010):

```
pred = t_prev + T + creep      creep lets the grid rise back onto the envelope
t    = min(stamp, pred)        late stamp: keep the grid; earlier stamp: the grid was too late
```

- **Losses:** samples later than T/2 are held until a sample is on time again (they were only delayed) or for
  25 ms (samples were lost, and the gap is placed before them).
- **Repeats:** a sample identical to the previous one (all six axes) is dropped.
- **Outages:** a stamp jump above `max_gap` restarts the grid; the output gap equals the real gap.

The algorithm is a header-only class without ROS dependencies:
[include/imu_restamper/restamper.hpp](include/imu_restamper/restamper.hpp).

## Run

```bash
colcon build --packages-select imu_restamper
ros2 launch imu_restamper restamp.launch.py     # logs counters every 10 s
```

Set the topics and the period in [config/restamp.yaml](config/restamp.yaml).

| Parameter | Default | Purpose |
|---|---|---|
| `imu_in` / `imu_out` | `/imu` / `/imu/restamped` | Input and output topics |
| `period` | `0.005` | Nominal sample period [s] |
| `reliable_input` | `true` | QoS of the input subscription |
| `acc_scale` | `1.0` | Accelerometer scale (`9.80665` converts g to m/s²) |
| `drop_duplicates` | `true` | Drop repeated samples |
| `creep` | `1e-6` | Grid rise per sample [s]. Must exceed how much slower than `period` the sensor runs (1 µs covers +200 ppm). A steadily growing `lost` counter on a clean input means it is too small |
| `max_gap` | `0.1` | [s] Quiet input triggers a warning; a larger stamp jump restarts the grid |

## Tools

```bash
ros2 run imu_restamper record_restamp_check.py 60 check.csv --raw /imu --out /imu/restamped   # input + output side by side
ros2 run imu_restamper plot_restamp_check.py check.csv check.png "title"                     # summary + plot (numpy, matplotlib)
ros2 run imu_restamper record_stamp_trace.py 60 trace.csv /imu                               # raw stamp trace
ros2 run imu_restamper restamp_eval trace.csv --drop-imu 40 --outage 2                       # offline evaluation
ros2 run imu_restamper restamp_eval --synthetic --minutes 10 --ppm 50 --drop-imu 200
```

`restamp_eval` has no ROS dependency and also builds standalone:
`c++ -std=c++17 -O2 -I include src/restamp_eval.cpp -o restamp_eval`.
