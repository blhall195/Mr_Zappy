from mag_cal.calibration import Calibration

CALIBRATION_DICT = {
    "mag": {
        "axes": "-X-Y-Z",
        "transform": [
            [0.0231683, -4.50966e-05, -0.000208465],
            [-4.50968e-05, 0.0233006, -2.46289e-05],
            [-0.000208464, -2.46296e-05, 0.0231333],
        ],
        "centre": [0.407859, -1.9058, 2.11295],
        "rbfs": [],
        "field_avg": None,
        "field_std": None,
    },
    "dip_avg": None,
    "grav": {
        "axes": "-Y-X+Z",
        "transform": [
            [0.101454, 0.00155312, -0.000734401],
            [0.00155312, 0.101232, 0.00149594],
            [-0.000734397, 0.00149594, 0.0987455],
        ],
        "centre": [0.364566, -0.0656354, 0.193454],
        "rbfs": [],
        "field_avg": None,
        "field_std": None,
    },
}


class RollingValue:
    def __init__(self, num_samples):
        self.values = []
        self.num_samples = num_samples

    def add_value(self, value: float):
        self.values.append(value)
        if len(self.values) > self.num_samples:
            self.values = self.values[1:]

    @property
    def avg_value(self):
        if len(self.values) > 0:
            total = 0
            for value in self.values:
                total += value
            return total / len(self.values)
        else:
            return 0

    @property
    def last_value(self):
        if len(self.values) > 0:
            return self.values[-1]
        else:
            return 0


class ThreeAxisAngles:
    def __init__(self, history_length: int):
        self.x_store = RollingValue(history_length)
        self.y_store = RollingValue(history_length)
        self.z_store = RollingValue(history_length)

    def add_values(self, x: float, y: float, z: float):
        self.x_store.add_value(x)
        self.y_store.add_value(y)
        self.z_store.add_value(z)

    @property
    def last_value(self):
        return self.x_store.last_value, self.y_store.last_value, self.z_store.last_value

    @property
    def avg_value(self):
        return self.x_store.avg_value, self.y_store.avg_value, self.z_store.avg_value


class CustomAngleClass:
    def __init__(self):
        self.grav_raw = ThreeAxisAngles(10)
        self.mag_raw = ThreeAxisAngles(10)

        # self.calib = Calibration(mag_axes="-X-Y-Z", grav_axes="-Y-X+Z")
        self.calibration = Calibration.from_dict(CALIBRATION_DICT)

        # convert to properties
        self.azimuth = RollingValue(10)
        self.inclination = RollingValue(10)
        self.roll = RollingValue(1)

    def get_last_value(self):
        return self.calibration.get_angles(self.mag_raw.last_value, self.grav_raw.last_value)

    def get_avg_value(self):
        return self.calibration.get_angles(self.mag_raw.avg_value, self.grav_raw.avg_value)
