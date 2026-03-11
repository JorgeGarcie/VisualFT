"""Error hierarchy for the VisualFT system.

All custom exceptions inherit from VisionFTError.
"""


class VisionFTError(Exception):
    """Base exception for all VisualFT errors."""


class SafetyViolation(VisionFTError):
    """Raised when a command violates safety limits (workspace, velocity, force)."""


class RobotFault(VisionFTError):
    """Raised when the robot enters a fault state."""


class ConnectionLost(VisionFTError):
    """Raised when the RDK connection is lost or heartbeat times out."""


class SensorError(VisionFTError):
    """Raised when a sensor fails or returns invalid data."""
