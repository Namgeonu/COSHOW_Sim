"""Read-only checks after non-root OpenCV provisioning and both layout builds."""
import hashlib
import os
from pathlib import Path
import subprocess

assert os.getuid() == 1000, 'Expected the disposable desktop user'
print('INSTALLER_UID', os.getuid())
print(subprocess.check_output(['dpkg-query', '-W', 'python3-numpy', 'python3-opencv'], text=True).strip())
for directory in ('COSHOW', 'COSHOW-merged'):
    project = Path('/home/coshow') / directory
    for relative in ('bt/scenarios/coshow/configs/coshow_rehearsal.yaml',
                     'ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml',
                     'ros2_ws/src/aideck_aruco_ros/config/drones.yaml'):
        if directory == 'COSHOW-merged' and relative.startswith('bt/'):
            # This separate layout fixture contains only ROS packages; the
            # full project/BT fixture is COSHOW and was checked above.
            print('NOT PRESENT IN ROS-ONLY MERGED FIXTURE', relative)
            continue
        copied = project / relative
        assert copied.read_bytes() == (Path('/source') / relative).read_bytes(), relative
        print('UNCHANGED', directory, relative, hashlib.sha256(copied.read_bytes()).hexdigest())
    for folder in ('build', 'install'):
        tree = project / 'ros2_ws' / folder
        assert all(path.lstat().st_uid == os.getuid() for path in tree.rglob('*')), str(tree)
    print('PASS non-root build/install ownership:', directory)
print('PASS existing ROS execution dependencies provisioned; collaborator YAML and build ownership preserved')
