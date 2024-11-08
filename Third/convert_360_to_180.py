def convert_360_to_180(self, angle_360):
    if angle_360 > 180:
        return angle_360 - 360
    else:
        return angle_360