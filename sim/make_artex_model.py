#!/usr/bin/env python3
"""Generate a Gazebo (Harmonic) model and a PX4 SITL airframe for the ARTEX, from PX4's x500.

    python3 sim/make_artex_model.py --px4 ~/drone-dev/PX4-Autopilot --out sim/gz_artex

Writes:
    <out>/models/artex_base/{model.sdf,model.config}   body, rotors, IMU/baro/mag/GPS
    <out>/models/artex/{model.sdf,model.config}        + motor physics + downward TFmini
    <out>/airframes/4099_gz_artex                      PX4 SITL airframe (the ARTEX's settings)

Then `sim/deploy_artex_sim.sh <jetson>` installs them into that Jetson's PX4 tree and rebuilds.

EVERY NUMBER IS BELOW, WITH ITS SOURCE. Measured by the operator on 20 Oct 2026 unless noted.
Where a value is estimated, it says so. The model is CALIBRATED so that it hovers at the throttle
PX4's own hover-thrust estimator learned on ARTEX113's real flights (0.41-0.44), because that
anchors the thrust model to the real aircraft even though the motor KV is not certain.
"""
from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

G = 9.80665

# ---- measured --------------------------------------------------------------------------
MASS_TOTAL = 0.500             # kg, all-up with the flight battery (operator: "about 500 g")
FRONT_BACK = 0.1366            # m, motor-to-motor front to back
SIDE_SIDE = 0.200              # m, motor-to-motor side to side
BATTERY = dict(mass=0.312, x=0.143, y=0.044, z=0.023)  # 4S 5000 mAh, 143x44x23 mm, 312 g
MOTOR_MASS = 0.018             # kg, EMAX ECO II 2004 (spec sheet ~18 g)
PROP_DIAMETER = 0.131          # m, Gemfan Hurricane SL 5125-3 (5.1 in, 3-blade)
TFMINI_RANGE = (0.40, 12.0)    # m, Benewake TFmini, mounted at the centre facing down
# operator, 26 Oct 2026: heights above the floor with the aircraft standing on it
BODY = dict(x=0.205, y=0.050, z=0.050)   # m, the "coffin" body, modelled as a box (L x W x H)
BODY_CLEARANCE = 0.0223        # m, body underside to floor
MOTOR_MOUNT_H = 0.0270         # m, motor mount to floor
PROP_H = 0.0484                # m, prop plane to floor (mount 27.0 + 20.7 mount-to-prop)
FRONT_ARM = 0.1016             # m, front arms "4 inches": from the body's side (y 25 mm) to a
                               # front motor (x 68.3, y 100) is 101.4 mm -- consistent
ORIGIN_H = BODY_CLEARANCE + BODY["z"] / 2   # model origin = body centre, 47.3 mm up
# operator's photos, 26 Oct 2026: the battery is strapped ON TOP of the body, not inside it
BATTERY_Z = BODY["z"] / 2 + BATTERY["z"] / 2    # battery centre above the body centre (36.5 mm)

# ---- from the aircraft's own logs -----------------------------------------------------------
HOVER_THROTTLE = 0.42          # PX4 hover_thrust_estimate, ARTEX113 flights: 0.411-0.442

# ---- estimated (stated, not measured) -------------------------------------------------------
KV = 2400                      # operator: "likely 2400 KV"; with 5 in props on 4S
V_LOADED = 14.8                # V, 4S under load
LOAD_FACTOR = 0.79             # loaded rpm / (KV x V); typical for a 5 in 3-blade
OMEGA_MIN = 150.0              # rad/s at idle output (PX4 x500 default, DShot idle ~5%)
MOMENT_CONSTANT = 0.0125       # m, rotor torque/thrust for a 5 in prop (x500's 13 in: 0.016)

# ---- no thrust fudge ------------------------------------------------------------------------------
# 20 Oct 2026 the model hovered at 0.333 and a (1079/1321)^2 "calibration" was applied. WRONG CAUSE,
# found 26 Oct: the airframe ran PX4's default rate gains (about 3x the real ARTEX's on roll), the
# model sat in a +/-125 deg/s roll limit cycle with the motors swinging 150 <-> 2849 rad/s, and the
# mean of w^2 over that swing is what lifted it. With the real gains the hover is steady and
# motorConstant * w^2 is right as derived. Kept as a named 1.0 so the history stays readable.
THRUST_CAL = 1.0


def derived() -> dict:
    omega_max = KV * V_LOADED * LOAD_FACTOR * 2 * math.pi / 60.0
    hx, hy = FRONT_BACK / 2, SIDE_SIDE / 2
    # PX4 maps throttle u linearly to commanded rotor speed: w = w_min + u (w_max - w_min).
    omega_hover = OMEGA_MIN + HOVER_THROTTLE * (omega_max - OMEGA_MIN)
    k_thrust = THRUST_CAL * (MASS_TOTAL * G / 4) / omega_hover ** 2
    rotor_mass = 0.016077                       # kept from x500 (link mass, not inertia-relevant)
    base_mass = MASS_TOTAL - 4 * rotor_mass
    b, m = BATTERY, BATTERY["mass"]
    body_mass = MASS_TOTAL - m - 4 * MOTOR_MASS
    box = lambda mm, a, bb: mm / 12 * (a * a + bb * bb)  # noqa: E731
    # centre of gravity of the base link: body at 0, battery on top at BATTERY_Z, motors on their
    # mounts (MOTOR_MOUNT_H). Inertia is taken about that point (parallel axis).
    mz = MOTOR_MOUNT_H - ORIGIN_H
    cg_z = (m * BATTERY_Z + 4 * MOTOR_MASS * mz) / (m + body_mass + 4 * MOTOR_MASS)
    up = m * (BATTERY_Z - cg_z) ** 2 + body_mass * cg_z ** 2 + 4 * MOTOR_MASS * (mz - cg_z) ** 2
    ixx = box(m, b["y"], b["z"]) + box(body_mass, BODY["y"], BODY["z"]) + 4 * MOTOR_MASS * hy ** 2 + up
    iyy = box(m, b["x"], b["z"]) + box(body_mass, BODY["x"], BODY["z"]) + 4 * MOTOR_MASS * hx ** 2 + up
    izz = box(m, b["x"], b["y"]) + box(body_mass, BODY["x"], BODY["y"]) + 4 * MOTOR_MASS * (hx * hx + hy * hy)
    return dict(omega_max=omega_max, omega_hover=omega_hover, k_thrust=k_thrust, hx=hx, hy=hy,
                base_mass=base_mass, ixx=ixx, iyy=iyy, izz=izz, cg_z=cg_z,
                max_thrust_g=k_thrust * omega_max ** 2 / G * 1000)


def sub(text: str, pattern: str, repl: str, count: int = 0, expect: int | None = None) -> str:
    new, n = re.subn(pattern, repl, text, count=count, flags=re.S)
    if expect is not None and n != expect:
        raise SystemExit(f"pattern {pattern!r}: expected {expect} replacements, made {n}")
    return new


def make_base(src: str, d: dict) -> str:
    s = src.replace("<model name='x500_base'>", "<model name='artex_base'>")
    s = sub(s, r"(<link name=\"base_link\">.*?<mass>)2\.0(</mass>)", rf"\g<1>{d['base_mass']:.4f}\2", expect=1)
    s = sub(s, r"(<link name=\"base_link\">\s*<inertial>)", rf"\g<1>\n        <pose>0 0 {d['cg_z']:.4f} 0 0 0</pose>", expect=1)
    s = sub(s, r"(<link name=\"base_link\">.*?<ixx>)[0-9.e-]+", rf"\g<1>{d['ixx']:.3e}", expect=1)
    s = sub(s, r"(<link name=\"base_link\">.*?<iyy>)[0-9.e-]+", rf"\g<1>{d['iyy']:.3e}", expect=1)
    s = sub(s, r"(<link name=\"base_link\">.*?<izz>)[0-9.e-]+", rf"\g<1>{d['izz']:.3e}", expect=1)
    # every 0.174 / -0.174 arm coordinate: x -> half front-back, y -> half side-side
    s = sub(s, r"<pose>(-?)0\.174 (-?)0\.174 ", lambda m: f"<pose>{m.group(1)}{d['hx']:.4f} {m.group(2)}{d['hy']:.4f} ")
    # prop visuals and collisions: x500's props are scaled to ~11.6 in; make them 5.1 in
    scale = 0.8461538461538461 * PROP_DIAMETER / 0.2792307692307692
    s = s.replace("<scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>",
                  f"<scale>{scale:.4f} {scale:.4f} {scale:.4f}</scale>")
    s = s.replace("0.2792307692307692 0.016923076923076923", f"{PROP_DIAMETER:.4f} 0.0100")
    s = s.replace("0.35355339059327373 0.35355339059327373 0.05", "0.24 0.24 0.05")
    # No rotor collisions. x500's are 0.85 mm-thick boxes; spinning into the ground after a
    # tumble they tripped an ODE assertion and killed the Gazebo server (26 Oct 2026). The
    # body's collision box still meets the floor; prop strikes are not modelled.
    s = sub(s, r"\s*<collision name=\"rotor_\d_collision\">.*?</collision>", "", expect=4)
    # No x500 landing gear. Its legs (22 cm tall, skids 26 cm apart) tipped the model over on a
    # slow stick take-off (26 Oct 2026). The ARTEX's measured stance instead: the body box at
    # its real size and height, and -- ASSUMED, the operator gave heights not the feet -- four
    # small feet under the motors carrying it 22.3 mm off the floor. The x500 frame MESH still
    # draws legs; they are visual only.
    s = sub(s, r"\s*<collision name=\"base_link_collision_[1-4]\">.*?</collision>", "", expect=4)
    s = s.replace("<size>0.24 0.24 0.05</size>", f"<size>{BODY['x']} {BODY['y']} {BODY['z']}</size>")
    s = sub(s, r"(<collision name=\"base_link_collision_0\">\s*<pose>)0 0 \.007 0 0 0", r"\g<1>0 0 0 0 0 0", expect=1)
    foot_h = MOTOR_MOUNT_H
    feet = "".join(f"""
      <collision name="foot_{i}">
        <pose>{x:.4f} {y:.4f} {-ORIGIN_H + foot_h / 2:.4f} 0 0 0</pose>
        <geometry><box><size>0.012 0.012 {foot_h:.4f}</size></box></geometry>
      </collision>""" for i, (x, y) in enumerate(((d['hx'], d['hy']), (d['hx'], -d['hy']),
                                                   (-d['hx'], d['hy']), (-d['hx'], -d['hy']))))
    s = sub(s, r"(<collision name=\"base_link_collision_0\">.*?</collision>)", lambda m: m.group(1) + feet, expect=1)
    # rotors in the measured prop plane, relative to the body centre; spawn standing on the floor
    s = sub(s, r"(<link name=\"rotor_\d\">.*?<pose>-?[0-9.]+ -?[0-9.]+ )0\.06( 0 0 0</pose>)",
            rf"\g<1>{PROP_H - ORIGIN_H:.4f}\2", expect=4)
    s = s.replace("<pose>0 0 .24 0 0 0</pose>", f"<pose>0 0 {ORIGIN_H + 0.002:.4f} 0 0 0</pose>", 1)
    return s


def make_motors(src: str, d: dict) -> str:
    s = src.replace("<model name='x500'>", "<model name='artex'>")
    s = s.replace("<uri>model://x500_base</uri>", "<uri>model://artex_base</uri>")
    s = s.replace("<maxRotVelocity>1000.0</maxRotVelocity>", f"<maxRotVelocity>{d['omega_max']:.0f}</maxRotVelocity>")
    s = s.replace("<motorConstant>8.54858e-06</motorConstant>", f"<motorConstant>{d['k_thrust']:.4e}</motorConstant>")
    s = s.replace("<momentConstant>0.016</momentConstant>", f"<momentConstant>{MOMENT_CONSTANT}</momentConstant>")
    lidar = f"""
    <link name="lidar_sensor_link">
      <pose relative_to="base_link">0 0 {-BODY["z"] / 2:.4f} 0 1.5708 0</pose>
      <inertial><mass>0.001</mass><inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <sensor name="lidar" type="gpu_lidar">
        <pose>0 0 0 3.14159 0 0</pose>
        <update_rate>100</update_rate>
        <ray>
          <scan><horizontal><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></horizontal>
                <vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical></scan>
          <range><min>{TFMINI_RANGE[0]}</min><max>{TFMINI_RANGE[1]}</max><resolution>0.01</resolution></range>
        </ray>
        <always_on>1</always_on>
        <visualize>false</visualize>
      </sensor>
    </link>
    <joint name="lidar_sensor_joint" type="fixed"><parent>base_link</parent><child>lidar_sensor_link</child></joint>
  </model>"""
    return s.replace("  </model>", lidar, 1)


def make_airframe(d: dict) -> str:
    ec = "\n".join(f"param set-default SIM_GZ_EC_FUNC{i} 10{i}\n"
                   f"param set-default SIM_GZ_EC_MIN{i} {OMEGA_MIN:.0f}\n"
                   f"param set-default SIM_GZ_EC_MAX{i} {d['omega_max']:.0f}" for i in range(1, 5))
    return f"""#!/bin/sh
#
# @name Gazebo ARTEX (5 in quad X, 500 g, generated by FlightPlanner/sim/make_artex_model.py)
#
# @type Quadrotor
#
# Control allocation and tuning mirror the REAL ARTEX (113/115 dumps, identical, 26 Oct 2026):
# normalised rotor positions +/-1, KM +/-0.05, the vendor's rate gains (roll K 0.35, pitch K 0.40,
# yaw K 1.2 -- NOT PX4's defaults: until 26 Oct this file used the defaults and the model oscillated
# in roll), gyro filters 80/30 Hz, and option A (hover 0.42 + estimator, land 0.6, tilt 20,
# climb 1 m/s, tumble 45 deg) and the height settings (barometer reference EKF2_HGT_REF 0 -- PX4's
# default 1 is GPS, which indoors does not exist; found 26 Oct). Physics live in the Gazebo model.

. ${{R}}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${{PX4_SIMULATOR:=gz}}
PX4_GZ_WORLD=${{PX4_GZ_WORLD:=default}}
PX4_SIM_MODEL=${{PX4_SIM_MODEL:=artex}}

param set-default SIM_GZ_EN 1

param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4
param set-default CA_ROTOR0_PX 1.0
param set-default CA_ROTOR0_PY 1.0
param set-default CA_ROTOR0_KM 0.05
param set-default CA_ROTOR1_PX -1.0
param set-default CA_ROTOR1_PY -1.0
param set-default CA_ROTOR1_KM 0.05
param set-default CA_ROTOR2_PX 1.0
param set-default CA_ROTOR2_PY -1.0
param set-default CA_ROTOR2_KM -0.05
param set-default CA_ROTOR3_PX -1.0
param set-default CA_ROTOR3_PY 1.0
param set-default CA_ROTOR3_KM -0.05

{ec}

param set-default MC_ROLLRATE_K 0.35
param set-default MC_ROLLRATE_P 0.15
param set-default MC_ROLLRATE_I 0.2
param set-default MC_ROLLRATE_D 0.003
param set-default MC_PITCHRATE_K 0.40
param set-default MC_PITCHRATE_P 0.15
param set-default MC_PITCHRATE_I 0.2
param set-default MC_PITCHRATE_D 0.003
param set-default MC_YAWRATE_K 1.2
param set-default MC_YAWRATE_P 0.2
param set-default MC_YAWRATE_I 0.1
param set-default MC_YAWRATE_D 0.0
param set-default MC_ROLL_P 6.5
param set-default MC_PITCH_P 6.5
param set-default MC_YAW_P 2.8
param set-default IMU_GYRO_CUTOFF 80
param set-default IMU_DGYRO_CUTOFF 30

param set-default MPC_THR_HOVER 0.42
param set-default MPC_USE_HTE 1
param set-default MPC_LAND_SPEED 0.6
param set-default MPC_MAN_TILT_MAX 20
param set-default MPC_Z_VEL_MAX_UP 1.0
param set-default FD_FAIL_P 45
param set-default FD_FAIL_R 45
param set-default BAT1_N_CELLS 4
param set-default EKF2_HGT_REF 0
param set-default EKF2_RNG_DELAY 5
param set-default EKF2_RNG_CTRL 1
param set-default EKF2_RNG_A_HMAX 1.0
"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--px4", default=str(Path.home() / "drone-dev/PX4-Autopilot"))
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent / "gz_artex"))
    args = ap.parse_args()
    px4, out = Path(args.px4), Path(args.out)
    models = px4 / "Tools/simulation/gz/models"
    d = derived()
    for name, maker, srcdir in (("artex_base", make_base, "x500_base"), ("artex", make_motors, "x500")):
        dst = out / "models" / name
        dst.mkdir(parents=True, exist_ok=True)
        (dst / "model.sdf").write_text(maker((models / srcdir / "model.sdf").read_text(), d))
        cfg = (models / srcdir / "model.config").read_text().replace(srcdir, name)
        (dst / "model.config").write_text(cfg)
    (out / "airframes").mkdir(parents=True, exist_ok=True)
    (out / "airframes/4099_gz_artex").write_text(make_airframe(d))
    print("ARTEX model generated:")
    for k in ("base_mass", "ixx", "iyy", "izz", "omega_max", "omega_hover", "k_thrust", "max_thrust_g"):
        print(f"  {k:13s} {d[k]:.4g}")
    print(f"  rotor arms     x +/-{d['hx']:.4f} m, y +/-{d['hy']:.4f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
