"""scout.core must stay ROS-free so the whole suite runs on a plain Python
install (dev Mac / CI). This walks the sources and fails on any ROS import."""

import pathlib

CORE = pathlib.Path(__file__).resolve().parent.parent / 'scout' / 'core'

FORBIDDEN = (
    'rclpy', 'tf2_ros', 'tf2_py', 'tf_transformations',
    'sensor_msgs', 'geometry_msgs', 'std_msgs', 'nav_msgs', 'nav2_msgs',
    'action_msgs', 'rosidl', 'ament_index_python', 'yaml', 'rosbridge',
)


def test_core_imports_are_pure():
    offenders = []
    for path in sorted(CORE.glob('*.py')):
        for lineno, line in enumerate(path.read_text().splitlines(), 1):
            s = line.strip()
            if not (s.startswith('import ') or s.startswith('from ')):
                continue
            token = s.split()[1].split('.')[0]
            if token in FORBIDDEN:
                offenders.append('%s:%d: %s' % (path.name, lineno, s))
    assert not offenders, 'ROS imports found in scout.core:\n' + '\n'.join(offenders)
