import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "models" / "team01_car" / "model.sdf"

def numbers(text, count=6):
    values = [float(v) for v in (text or "").split()]
    return values + [0.0] * (count - len(values))

def rotation(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
            (-sp, cp*sr, cp*cr))

def test_model_envelope_and_limits():
    model = ET.parse(MODEL).getroot().find("model")
    for pose in model.iter("pose"):
        assert len((pose.text or "").split()) == 6
    low, high = [float("inf")]*3, [float("-inf")]*3
    for link in model.findall("link"):
        lp = numbers(link.findtext("pose"))
        rot = rotation(*lp[3:6])
        for item in [*link.findall("visual"), *link.findall("collision")]:
            ip = numbers(item.findtext("pose"))
            center = [lp[i] + sum(rot[i][j]*ip[j] for j in range(3)) for i in range(3)]
            box = item.find("geometry/box/size")
            cylinder = item.find("geometry/cylinder")
            if box is not None:
                half_local = [v/2 for v in numbers(box.text, 3)[:3]]
                half = [sum(abs(rot[i][j])*half_local[j] for j in range(3)) for i in range(3)]
            elif cylinder is not None:
                radius = float(cylinder.findtext("radius"))
                length = float(cylinder.findtext("length"))
                axis = [rot[i][2] for i in range(3)]
                half = [abs(axis[i])*length/2 + radius*math.sqrt(max(0.0, 1-axis[i]**2)) for i in range(3)]
            else:
                continue
            for i in range(3):
                low[i], high[i] = min(low[i], center[i]-half[i]), max(high[i], center[i]+half[i])
    dimensions = [high[i]-low[i] for i in range(3)]
    assert dimensions[0] <= 0.55 + 1e-9
    assert dimensions[1] <= 0.25 + 1e-9
    assert dimensions[2] <= 0.18 + 1e-9
    for joint in model.findall("joint"):
        if "steering" in joint.attrib["name"]:
            assert math.isclose(
                float(joint.findtext("axis/limit/lower")), -0.523599, abs_tol=1e-9
            )
            assert math.isclose(
                float(joint.findtext("axis/limit/upper")), 0.523599, abs_tol=1e-9
            )

    # Front wheel cylinders sweep around Z at full steering. Include this moving
    # geometry, rather than validating only the straight-ahead SDF pose.
    wheel = next(v for v in model.findall("link") if v.attrib["name"] == "front_left_wheel")
    wheel_pose = numbers(wheel.findtext("pose"))
    wheel_radius = float(wheel.findtext("collision/geometry/cylinder/radius"))
    wheel_width = float(wheel.findtext("collision/geometry/cylinder/length"))
    steering_joint = next(v for v in model.findall("joint") if v.attrib["name"] == "front_left_steering_joint")
    steering = float(steering_joint.findtext("axis/limit/upper"))
    swept_half_x = abs(math.sin(steering))*wheel_width/2 + wheel_radius*abs(math.cos(steering))
    swept_half_y = abs(math.cos(steering))*wheel_width/2 + wheel_radius*abs(math.sin(steering))
    swept_length = max(high[0], wheel_pose[0]+swept_half_x) - low[0]
    swept_width = 2*(abs(wheel_pose[1])+swept_half_y)
    assert swept_length <= 0.55
    assert swept_width <= 0.25

    visual_names = {
        visual.attrib["name"]
        for visual in model.find("link[@name='chassis']").findall("visual")
    }
    assert {
        "lower_wedge_body", "front_hood", "front_upper_chamfer", "windshield",
        "angular_roof", "rear_slope", "rear_upper_chamfer",
        "left_side_panel", "right_side_panel",
    } <= visual_names

def test_lidar_is_front_mounted_and_forward_only():
    model = ET.parse(MODEL).getroot().find("model")
    link = next(v for v in model.findall("link") if v.attrib["name"] == "team01_lidar_link")
    pose = numbers(link.findtext("pose"))
    assert pose[0] >= 0.17
    assert pose[2] + 0.04 > 0.08  # world height after the declared spawn offset
    sensor = link.find("sensor")
    assert sensor.attrib["type"] == "gpu_lidar"
    assert math.isclose(float(sensor.findtext("lidar/scan/horizontal/min_angle")), -math.pi/2, abs_tol=1e-6)
    assert math.isclose(float(sensor.findtext("lidar/scan/horizontal/max_angle")), math.pi/2, abs_tol=1e-6)
    assert float(sensor.findtext("lidar/range/max")) >= 6.0

def test_entry_manifest_points_to_package_assets():
    entry = json.loads((ROOT / "race_entry.json").read_text())
    assert entry["entry_id"] == "team01"
    assert entry["ros_package"] == "team01_race"
    assert entry["algorithm_launch_file"] == "race.launch.py"
    team_root = ROOT.parents[1]
    assert (team_root / entry["vehicle_model_file"]).is_file()

def test_practice_launch_and_helpers_are_complete():
    launch = (ROOT / "launch" / "practice.launch.py").read_text()
    assert 'package="ros_gz_sim"' in launch
    assert 'executable="create"' in launch
    assert 'default_value="monza"' in launch
    assert 'default_value="team01"' in launch
    assert 'default_value="0.37867"' in launch
    assert 'default_value="-2.05604"' in launch
    assert 'default_value="1.24353"' in launch
    assert 'default_value="1.45738"' in launch
    assert "gz sim" not in launch

    scripts = ROOT / "scripts"
    for name in (
        "source_practice_env.sh",
        "run_team01_practice.sh",
        "check_practice_ready.sh",
        "practice_start.sh",
    ):
        path = scripts / name
        assert path.is_file()
        assert os.access(path, os.X_OK)
