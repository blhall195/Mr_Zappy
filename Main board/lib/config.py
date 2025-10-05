import os

class Config:
    def __init__(self):
        # Existing tolerances
        self.mag_tolerance = float(os.getenv("mag") or 10.0)
        self.grav_tolerance = float(os.getenv("grav") or 10.0)
        self.dip_tolerance = float(os.getenv("dip") or 10.0)
        self.anomaly_detection = os.getenv("anomaly_detection", "False").lower() in ("true", "1", "yes")


        # Stability settings
        self.accuracy = os.getenv("accuracy", "Medium")  # default True
        self.stability_tolerance = float(os.getenv("stability_tolerance") or 0.2)  # from your settings.toml
        self.stability_buffer_length = int(os.getenv("stability_buffer_length") or 5)  # buffer window size
        self.EMA_alpha = float(os.getenv("EMA_alpha") or 0.2)  # default smoothing factor

        # Leg tolerances
        self.leg_angle_tolerance = float(os.getenv("leg_angle_tolerance") or 1.7)
        self.leg_distance_tolerance = float(os.getenv("leg_distance_tolerance") or 0.05)

        # Leg offset
        self.laser_distance_offset = float(os.getenv("laser_distance_offset") or 0.14)

        # Power settings
        self.auto_shutdown_timeout = int(os.getenv("auto_shutdown_timeout") or 1800)
        self.laser_timeout = int(os.getenv("laser_timeout") or 1800)
