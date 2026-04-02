from enum import Enum
from commands2 import Subsystem
import wpilib
from wpilib import DriverStation

class PiLights(Subsystem):
    def __init__(self):
        super().__init__()
        self.PWMOUT = wpilib.DigitalOutput(0)
        self._current_state: GameStates | None = None
        self.current_alliance = wpilib.DriverStation.getAlliance()
        self.inactive_color = "RED" if self.current_alliance == wpilib.DriverStation.Alliance.kRed else "BLUE"
        self.current_status = None

    def set_state(self, state: LEDState) -> None:
        if self.current_status == state.value:
            return
        self.PWMOUT.pulse(state.value)
        self.current_status = state.value
        print(f"Changing state to {state.name} - {state.value}")

    def _get_game_state(self, time: float) -> GameStates:
        for state in reversed(GameStates):
            if time >= state.value:
                return state
        return GameStates.AUTO

    def is_hub_active(self) -> bool:
        alliance = DriverStation.getAlliance()
        # If we have no alliance, we cannot be enabled, therefore no hub.
        if alliance is None:
            return False

        # Hub is always enabled in autonomous.
        if DriverStation.isAutonomousEnabled():
            return True

        # At this point if we're not teleop enabled, there is no hub.
        if not DriverStation.isTeleopEnabled():
            return False

        # We're teleop enabled, compute.
        match_time = DriverStation.getMatchTime()
        game_data = DriverStation.getGameSpecificMessage()

        match game_data:
            case "R":
                red_inactive_first = True
            case "B":
                red_inactive_first = False
            case _:
                # No or invalid game data, assume hub is active.
                return True

        # Shift 1 is active for blue if red won auto, or red if blue won auto.
        shift1_active = not red_inactive_first if alliance == DriverStation.Alliance.kRed else red_inactive_first

        if match_time > 130:
            return True  # Transition shift, hub is active
        elif match_time > 105:
            # Shift 1
            return shift1_active
        elif match_time > 80:
            # Shift 2
            return not shift1_active
        elif match_time > 55:
            # Shift 3
            return shift1_active
        elif match_time > 30:
            # Shift 4
            return not shift1_active
        else:
            return True  # End game, hub always active

    def periodic(self) -> None:
        time = wpilib.Timer.getMatchTime()
        game_state = self._get_game_state(time)
        first_inactive = wpilib.DriverStation.getGameSpecificMessage()

        active_color = LEDState.BLUE_ACTIVE if self.current_alliance == wpilib.DriverStation.Alliance.kBlue else LEDState.RED_ACTIVE
        ending_color = LEDState.BLUE_ENDING if self.current_alliance == wpilib.DriverStation.Alliance.kBlue else LEDState.RED_ENDING

        match_time = DriverStation.getMatchTime()
        if DriverStation.isAutonomousEnabled():
            if match_time > GameStates.AUTO_ENDING:
                self.set_state(LEDState.AUTO_ACTIVE)
            else:
                self.set_state(LEDState.AUTO_ENDING)
        elif DriverStation.isTeleopEnabled():
            if match_time > GameStates.TRANSITION_ENDING:
                # In transition
                self.set_state(active_color)
            elif match_time > GameStates.SHIFT_ONE:
                # Transition ending
                self.set_state(ending_color)
            elif match_time > GameStates.SHIFT_ONE_ENDING:
                self.set_state(active_color) if self.is_hub_active() else self.set_state(LEDState.CURRENT_INACTIVE)
            elif match_time > GameStates.SHIFT_TWO:
                self.set_state(ending_color) if self.is_hub_active() else self.set_state(LEDState.INACTIVE_ENDING)
            elif match_time > GameStates.SHIFT_TWO_ENDING:
                self.set_state(active_color) if self.is_hub_active() else self.set_state(LEDState.CURRENT_INACTIVE)
            elif match_time > GameStates.SHIFT_THREE:
                self.set_state(ending_color) if self.is_hub_active() else self.set_state(LEDState.INACTIVE_ENDING)
            elif match_time > GameStates.SHIFT_THREE_ENDING:
                self.set_state(active_color) if self.is_hub_active() else self.set_state(LEDState.CURRENT_INACTIVE)
            elif match_time > GameStates.SHIFT_FOUR:
                self.set_state(ending_color) if self.is_hub_active() else self.set_state(LEDState.INACTIVE_ENDING)
            elif match_time > GameStates.SHIFT_FOUR_ENDING:
                self.set_state(active_color) if self.is_hub_active() else self.set_state(LEDState.CURRENT_INACTIVE)
            elif match_time > GameStates.ENDGAME:
                self.set_state(ending_color) if self.is_hub_active() else self.set_state(LEDState.INACTIVE_ENDING)
            elif match_time > GameStates.ENGGAME_ENDING:
                self.set_state(active_color)
            elif match_time < GameStates.ENGGAME_ENDING:
                self.set_state(LEDState.PARTY_MODE)

        else:
            pass # Default state


class GameStates(Enum):
    AUTO_ENDING = 5.0
    TRANSITION_ENDING = 2 * 60 + 15.0
    SHIFT_ONE   = 2 * 60 + 10.0
    SHIFT_ONE_ENDING = 1 * 60 + 50.0
    SHIFT_TWO   = 1 * 60 + 45.0
    SHIFT_TWO_ENDING = 1 * 60 + 25.0
    SHIFT_THREE = 1 * 60 + 20.0
    SHIFT_THREE_ENDING = 60.0
    SHIFT_FOUR  = 55.0
    SHIFT_FOUR_ENDING = 25.0
    ENDGAME      = 30.0
    ENGGAME_ENDING  = 5.0

class LEDState(Enum):
    BLUE_ACTIVE      = 0.00002
    BLUE_ENDING      = 0.00004
    RED_ACTIVE       = 0.00006
    RED_ENDING       = 0.00008
    CURRENT_INACTIVE = 0.0001
    INACTIVE_ENDING  = 0.00012
    AUTO_ACTIVE      = 0.00014
    AUTO_ENDING      = 0.00016
    PARTY_MODE       = 0.00018
