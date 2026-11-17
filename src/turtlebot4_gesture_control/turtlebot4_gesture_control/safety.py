"""Full-surround scan guard; independent of the LiDAR's yaw mounting angle."""
import math


def scan_clearance(ranges, range_min, range_max, angle_increment):
    if (not math.isfinite(angle_increment) or len(ranges)*abs(angle_increment)<math.radians(300)
            or not 0 < range_min < range_max or not math.isfinite(range_max)):
        return None
    good=[]
    for value in ranges:
        if math.isinf(value) and value > 0:
            good.append(range_max)
        elif math.isfinite(value) and 0 <= value < range_min:
            return 0.0  # Below sensor minimum is not evidence of free space.
        elif math.isfinite(value) and value>=range_min:
            good.append(min(value,range_max))
    if len(good)<.85*len(ranges):
        return None
    return min(good) if good else None
