#!/usr/bin/env python3
"""Fetches PAL Robotics' TIAGo Pro for the browser demo, at a pinned commit with checksums.

The model is the flattened one in Open Planning's roboplan_example_models (MIT): PAL Robotics'
description and meshes (Apache 2.0) expanded from their xacro packages, which are built for Humble
only. It lands in batchplan_examples/tiago_pro (git-ignored) with both licenses, adapted for ROS 2
control and planning:

- the planar base joint becomes base_joint_x and base_joint_y (prismatic) and base_joint_theta
  (continuous), which robot_state_publisher and ros2_control move, as batchplan names them;
- the wheels are fixed: the base moves on those joints;
- gripper_left_tray_link and gripper_right_tray_link sit between each gripper's fingertips, z up
  while the gripper pinches a tray's rim, as the frames the tray carry constrains;
- config/tiago_pro.srdf's groups and end effectors are joined by the model's disabled pairs.

Run it once: `pixi run fetch-tiago-pro`. It checks what is already there and downloads only what
is missing or changed.
"""

import hashlib
import sys
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

COMMIT = "2a6b447e648b1a3337f10cbab995cda0c0a71d55"  # roboplan 0.8.0
SOURCE = f"https://raw.githubusercontent.com/open-planning/roboplan/{COMMIT}/"
MODEL = "roboplan_example_models/models/tiago_pro_robot_model/"
LICENSE_SHA256 = "eb1aefeda6ceb10afdf5f0a35fdd9c372d643fc7632a561c32b3226e68ed4d4d"
FILES = {
    "meshes/omni_base_description/base/base_antenna_link_reflected.stl": "3c2c1d46198029ce11fcd4165465046ffa09f91f9f53abe14494b6bd6210471f",
    "meshes/omni_base_description/base/base_antenna_link.stl": "736b08ffb56bb2d461209bce857b9a0ad0411e49447329ff8d0d8fa7310b92be",
    "meshes/omni_base_description/base/base_dock_link.stl": "d13ab33c5a39f9daa7f34a1b728c6b8afa2b105671846af29eb9f2d8c617cd0c",
    "meshes/omni_base_description/base/base_link.stl": "35cb915a06d5c78f4e80c41ac24a5b2376a7072502f9b1e144873db1dfe0cd48",
    "meshes/omni_base_description/wheels/suspension_front_link_reflected.stl": "8085d4a3a665d641ed5a002f1afa7c5489c5a12c7fe8c91f876b37a96f099a3e",
    "meshes/omni_base_description/wheels/suspension_front_link.stl": "9a2f26924de595e4c6e2b0ef7465c7af1d09ee3070e9f7173523113b545119d5",
    "meshes/omni_base_description/wheels/suspension_rear_link_reflected.stl": "6c2d0011eac2b5d27902dfd4b0e6cee943b9e68327ca27d937d1d7a24f87c717",
    "meshes/omni_base_description/wheels/suspension_rear_link.stl": "382922eaf317749b749ff3dd84c04e9f4dc206a2e7227a7a62c79f097f665c4c",
    "meshes/omni_base_description/wheels/wheel_link_reflected.stl": "eb51f06c02c3607a17fb6d135a0538d4872b904a7920d8ce05c0d1f61f06c2a9",
    "meshes/omni_base_description/wheels/wheel_link.stl": "00e72531f701fe9571fc7f27fff061b212ef5b6390ffb4bddd9a519c3e931b4e",
    "meshes/pal_pro_gripper_description/base_link_tc.stl": "e438c3363a7ad819b5ee44520f2b8e6a6e9636a0cc44afefacd98ae3f8a0ab2e",
    "meshes/pal_pro_gripper_description/fingertip.stl": "aef403084be21aa0fbc5ad411e4bf22c796a044a70b906834539453a5637df83",
    "meshes/pal_pro_gripper_description/inner_finger.stl": "ac2ec34c8b0cefe1d4637122553a8c9409a2ed6d389a90050962eed5a41cc5a3",
    "meshes/pal_pro_gripper_description/outer_finger.stl": "f6b296645c242878eaba955b30bd9b932aae88491bb9a627383f4a41e79f3dcc",
    "meshes/pal_sea_arm_description/arm_tiago_pro/arm_1_link_collision.stl": "f911e6d2a6f9383c30183183962648c7e3872116bc29225f1bf10c8cd0491487",
    "meshes/pal_sea_arm_description/arm_tiago_pro/arm_1_link.stl": "8f3672dbbb71d5ff9a158f687eb4438de0455b8aa73c92e9bb64f6aa50cfcd38",
    "meshes/pal_sea_arm_description/arm_tiago_pro/arm_2_link_collision.stl": "5dd1d5f0843169ef66d4a05ff34e664e161bbb832917a80f939cd2766430e9f4",
    "meshes/pal_sea_arm_description/arm_tiago_pro/arm_2_link.stl": "96ce7ed57708af6d2b3586dc4ec07d175a7d6025e9848f75539ef80d99b59375",
    "meshes/pal_sea_arm_description/arm_tiago_pro/arm_3_link_collision.stl": "4d9d8428b84d4511e95ac362c541a951c83efcfe47b0db958e35e231e944f9ff",
    "meshes/pal_sea_arm_description/arm_tiago_pro/arm_3_link.stl": "30ece623c4d71cba0eae5ebb4e96c56fd85552eca388cd46cfe1fe5263f6a4d5",
    "meshes/pal_sea_arm_description/arm_tiago_pro/arm_4_link_collision.stl": "8f86c50cda3e2c351c7b76c7f049fa28352e08ceabcae9b5eceaf58f204ff6c3",
    "meshes/pal_sea_arm_description/arm_tiago_pro/arm_4_link.stl": "3b37e8c6d1ee5498e22f34491dc83fa08f0cfb0f1309cd4e00eff79365f167a9",
    "meshes/pal_sea_arm_description/arm_tiago_pro/spherical-wrist/arm_5_link_collision.stl": "8d12d72d6114b807b2e7a20910672f06d64be64e1a760b3c6ec1ed774a59c6be",
    "meshes/pal_sea_arm_description/arm_tiago_pro/spherical-wrist/arm_5_link.stl": "8695a8df814a4e375a13201a24dd5f6cd1b640f7e568defd164637e77ed41a5a",
    "meshes/pal_sea_arm_description/arm_tiago_pro/spherical-wrist/arm_6_link_collision.stl": "b50fe52fd99bf91ed25de5988156ce3fa846fb68357c745afed0663d5242596a",
    "meshes/pal_sea_arm_description/arm_tiago_pro/spherical-wrist/arm_6_link.stl": "b17826b37b80a57831ff11da67adbb539b5144d8213a43bf8dac34749a4b878a",
    "meshes/pal_sea_arm_description/arm_tiago_pro/spherical-wrist/arm_7_link_collision.stl": "5c4ce3f61cc50d55f2d10ba78443c2453b6c9e5c0caba587f4d80a2ef26a6fe5",
    "meshes/pal_sea_arm_description/arm_tiago_pro/spherical-wrist/arm_7_link.stl": "5c4ce3f61cc50d55f2d10ba78443c2453b6c9e5c0caba587f4d80a2ef26a6fe5",
    "meshes/pal_urdf_utils/laser/sick_tim551.stl": "9e0184e01a67754bce3a008fb1f52b292dbda69857ed9092909d211817915cf2",
    "meshes/tiago_pro_description/torso/laptop_tray_collision.stl": "849b78bbe967caf550ba918b02e274770f9652094fdf6aaff23c5c24358230a2",
    "meshes/tiago_pro_description/torso/torso_fixed_collision.stl": "56a8f0732fdbb1f4145a61269d59f3ed07be1bb9c060e7caf44d8dd37cd06d80",
    "meshes/tiago_pro_description/torso/torso_fixed.stl": "1f2adfb0ff0fd6d705cee85ff4b95a6f988f39fcfff3dfa9f95baf4477fa3553",
    "meshes/tiago_pro_description/torso/torso_lift_link_collision.stl": "6c45a1e2778576ad7ef2d6ced8ffbbf6a4aaf2a2de8aecb59f7b44329b758d0b",
    "meshes/tiago_pro_description/torso/torso_lift_link.stl": "98922a8c4c2b4e2ba57e9109faef69fa2a8288fd403912935da67ddf07df759c",
    "meshes/tiago_pro_head_description/head_1_link_collision.stl": "bf984aed7be282270a78b2ed0f751e9910e6db9500f769956b95bd0c3522983e",
    "meshes/tiago_pro_head_description/head_1_link.stl": "8912a5c065cddb61267801db4c49f35c32213e1d3f3be9c1a48c13f3bfcca5fb",
    "meshes/tiago_pro_head_description/head_2_link_collision.stl": "bdc70cb83ec4c699b70edcef550a635b66e3e2ad426ef517ba8d2b9a46abb098",
    "meshes/tiago_pro_head_description/head_2_link.stl": "934da0461c8a633a119ed6362ab34d9b02dc0fc0c8a246e2a563bf5b2cd3736a",
    "meshes/tiago_pro_head_description/realsense-d435_cover_link.stl": "610e506d141ce723c2bd76eea5d5657f6bfb07d7873bdfc41b021e0143327e70",
    "tiago_pro_config.yaml": "9d931217f82ffdc3eb6f759eaa8145d7782e9652fba9a054fee484a8585e8fb5",
    "tiago_pro.srdf": "a0548c8187d0d7b1f083c8063cfeaa18aee2d73ab325aa9ea118e4ae2cadfb09",
    "tiago_pro.urdf": "6da6ff3e378c022b68c641d1dd9bd4b0026b773df419520d253440aae00c8fb9",
}

PACKAGE = Path(__file__).resolve().parent.parent / "batchplan_examples"
OUT = PACKAGE / "tiago_pro"
NOTICE = f"""TIAGo Pro's description and meshes are PAL Robotics' (https://github.com/pal-robotics),
under the Apache License 2.0. This flattened copy is from Open Planning's roboplan_example_models
(https://github.com/open-planning/roboplan, commit {COMMIT}), under the MIT license in LICENSE.
scripts/fetch_tiago_pro.py changed tiago_pro.urdf (base joints, fixed wheels, tray frames, mesh
paths) and joined tiago_pro.srdf's disabled pairs to batchplan_examples' groups.
"""


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


# Tries per download: GitHub's raw file server answers 503 at times, which a second try gets past,
# as `curl --retry` allows for.
ATTEMPTS = 3


def download(url: str) -> bytes:
    """`url`'s content, trying again after a server error or a dropped connection."""
    for attempt in range(1, ATTEMPTS + 1):
        try:
            with urllib.request.urlopen(url, timeout=60) as response:
                return response.read()
        except urllib.error.HTTPError as e:
            if e.code < 500 or attempt == ATTEMPTS:
                raise
        except (urllib.error.URLError, TimeoutError):
            if attempt == ATTEMPTS:
                raise
        time.sleep(2**attempt)
    raise AssertionError("every attempt returns or raises")


def fetch(source: str, target: Path, digest: str):
    """Downloads `source` to `target` unless it is already there with `digest`."""
    if target.exists() and sha256(target) == digest:
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    data = download(SOURCE + source)
    if hashlib.sha256(data).hexdigest() != digest:
        sys.exit(f"{source}: checksum mismatch")
    target.write_bytes(data)


def adapt(urdf: ET.Element):
    """The model's URDF as the demo runs it (see the module docstring)."""
    for joint in urdf.findall("joint"):
        if joint.get("name", "").startswith("wheel_"):
            joint.set("type", "fixed")
    base = next(j for j in urdf.findall("joint") if j.get("type") == "planar")
    parent, child = base.find("parent").get("link"), base.find("child").get("link")
    urdf.remove(base)
    # Speeds as the model's planning configuration has them: 0.5 m/s, 1 rad/s.
    chain = [
        ("x", "prismatic", "1 0 0", parent, "base_x_link", 'lower="-5" upper="5" velocity="0.5"'),
        ("y", "prismatic", "0 1 0", "base_x_link", "base_y_link", 'lower="-5" upper="5" velocity="0.5"'),
        ("theta", "continuous", "0 0 1", "base_y_link", child, 'velocity="1.0"'),
    ]
    for name, kind, axis, above, below, limit in chain:
        if below != child:
            ET.SubElement(urdf, "link", name=below)
        urdf.append(ET.fromstring(
            f'<joint name="base_joint_{name}" type="{kind}"><parent link="{above}"/><child link="{below}"/>'
            f'<axis xyz="{axis}"/><limit effort="1000" {limit}/></joint>'
        ))
    for side in ("left", "right"):
        ET.SubElement(urdf, "link", name=f"gripper_{side}_tray_link")
        urdf.append(ET.fromstring(
            f'<joint name="gripper_{side}_tray_joint" type="fixed"><origin xyz="0 0 0" rpy="-1.5707963 0 0"/>'
            f'<parent link="gripper_{side}_grasping_link"/><child link="gripper_{side}_tray_link"/></joint>'
        ))
    for mesh in urdf.iter("mesh"):
        mesh.set("filename", mesh.get("filename").replace(f"package://{MODEL}", "package://batchplan_examples/tiago_pro/"))


def main():
    # The meshes where the adapted URDF points; the model's own URDF and SRDF beside them, as given.
    given = lambda name: OUT / (name if name.startswith("meshes/") else f"given/{name}")
    for name, digest in FILES.items():
        fetch(MODEL + name, given(name), digest)
    fetch("LICENSE", OUT / "LICENSE", LICENSE_SHA256)
    urdf = ET.parse(given("tiago_pro.urdf")).getroot()
    adapt(urdf)
    ET.indent(urdf)
    (OUT / "tiago_pro.urdf").write_text(ET.tostring(urdf, encoding="unicode") + "\n")
    groups = ET.parse(PACKAGE / "config" / "tiago_pro.srdf").getroot()
    pairs = ET.parse(given("tiago_pro.srdf")).getroot().findall("disable_collisions")
    groups.extend(pairs)
    ET.indent(groups)
    (OUT / "tiago_pro.srdf").write_text(ET.tostring(groups, encoding="unicode") + "\n")
    (OUT / "NOTICE").write_text(NOTICE)
    print(f"TIAGo Pro in {OUT}: {len(FILES)} files, {len(pairs)} disabled pairs")


if __name__ == "__main__":
    main()
