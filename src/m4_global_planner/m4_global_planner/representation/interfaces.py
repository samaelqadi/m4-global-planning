from abc import ABC, abstractmethod


class PlanningMap(ABC):
    @abstractmethod
    def get_bounds(self):
        # return xmin, xmax, ymin, ymax
        pass

    @abstractmethod
    def get_ground_height(self, x, y):
        # return ground height at this position
        pass


class StateValidator(ABC):
    @abstractmethod
    def is_valid(self, state, planning_map):
        # check whether this state can physically exist
        pass


class StateGenerator(ABC):
    @abstractmethod
    def generate_states(self, planning_map, validator):
        # generate valid planning states
        pass
