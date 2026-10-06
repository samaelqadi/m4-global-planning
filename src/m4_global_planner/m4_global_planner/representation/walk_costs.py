from math import isfinite


class StraightWalkCost:
    """Distance-only test cost; terrain integration is deliberately pending."""

    def __init__(self, parameters):
        self.rate = parameters['straight_walk']['distance_cost_per_meter']
        if not isfinite(self.rate) or self.rate < 0:
            raise ValueError('Distance cost rate must be finite and nonnegative')

    def calculate(self, distance):
        return distance * self.rate


class WalkMotionCost(StraightWalkCost):
    """Test penalties are per edge, not per meter or radian."""

    def __init__(self, parameters):
        super().__init__(parameters)
        self.penalties = {
            'forward': 0.0,
            'reverse': parameters['reverse_extra_cost'],
            'pivot': parameters['pivot_extra_cost'],
        }
        if not all(isfinite(value) and value >= 0 for value in self.penalties.values()):
            raise ValueError('Motion penalties must be finite and nonnegative')

    def calculate(self, distance, motion='forward'):
        return super().calculate(distance) + self.penalties[motion]
