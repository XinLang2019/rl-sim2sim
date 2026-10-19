"""World-frame reset pose accepted by HTTP and the ROS sidecar (m, rad)."""
import math


def validate_reset_pose(value):
    if not isinstance(value, dict) or set(value) != {'position', 'rpy'}:
        raise ValueError('Robot reset requires position and rpy arrays')
    for name, limit in (('position', 10000), ('rpy', 2 * math.pi)):
        values = value[name]
        if not isinstance(values, list) or len(values) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > limit for v in values):
            raise ValueError('Invalid reset '+name+': three finite values, limit '+str(limit))
    return {name: [float(v) for v in value[name]] for name in ('position', 'rpy')}
