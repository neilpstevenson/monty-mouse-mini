import ujson
import os

# ============================================================================
# Track Configuration
# ============================================================================

class TrackConfig:
    def __init__(self):
        # Marker thresholds
        self.RADIUS_THRESH = 25000
        self.START_STOP_THRESH = 25000
        # Track dimensions
        self.TRACK_MEASURED_LENGTH = 900
        self.TRACK_STOPPING_TARGET = 200
        # Speeds
        self.TOP_SPEED = 4000
        self.FINISH_LINE_SPEED = 1000
        self.ACCELERATION = 8000
        self.DECELERATION = 2000
        
    def save(self):
        with open("track.json", "w") as file:
            ujson.dump(self.__dict__, file)

    def load(self):
        with open("track.json", "r") as file:
            dict = ujson.load(file)
        # Update all
        for setting in dict:
            setattr(self, setting, dict[setting])

# Create single instance
track_config = TrackConfig()

