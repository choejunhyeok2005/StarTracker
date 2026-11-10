"""
managers/state_manager.py

State Manager
"""

from enum import Enum, auto


class SystemState(Enum):

    INIT = auto()

    MANUAL = auto()

    # 기존 코드와의 호환용 별칭
    PREVIEW = MANUAL

    TARGET_CAPTURE = auto()

    TRACKING = auto()

    DRIFT_CORRECTION = auto()

    CAPTURE = auto()

    FINISH = auto()

    ALIGN = auto()
    


class StateManager:

    def __init__(self):

        self.state = SystemState.INIT

    def set_state(self, state):

        self.state = state

        print(f"[STATE] -> {state.name}")

    def get_state(self):

        return self.state

    def is_state(self, state):

        return self.state == state
