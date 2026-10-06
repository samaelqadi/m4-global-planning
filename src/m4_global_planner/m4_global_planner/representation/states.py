from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True, slots=True)
class PoseState:
    x: float
    y: float
    z: float
    heading: float

    def __post_init__(self):
        # all state values must be valid numbers
        values = (self.x, self.y, self.z, self.heading)

        if not all(isfinite(value) for value in values):
            raise ValueError('State values must be finite')
