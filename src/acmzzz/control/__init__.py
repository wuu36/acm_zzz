"""control module for acmzzz."""

from .regulator import PIRegulator, PIDRegulator
from .controller_params import (
    ControllerParams,
    FOCConfig,
    design_speed_pi_gains,
    design_current_pi_gains,
    auto_design_pi_gains,
)
from .foc import FOCController, foc_control
from .controller import MotorController

__all__ = [
    "PIRegulator",
    "PIDRegulator",
    "ControllerParams",
    "FOCConfig",
    "FOCController",
    "MotorController",
    "design_current_pi_gains",
    "design_speed_pi_gains",
    "auto_design_pi_gains",
    "foc_control",
]