
import dearpygui.dearpygui as dpg
from typing import Dict, Callable, Tuple, Optional

class ControlPanelManger:
    def __init__(self):
        self.control_ids: Dict[str, int] = {}
        self.callbacks: Dict[str, Callable] = {}
    
    def setup(
        self,
        parent: str = "MainWindow",
        advanced: bool = False,
        on_start: Optional[Callable] = None,
        on_stop: Optional[Callable] = None,
        on_reset: Optional[Callable] = None,
        on_speed_change: Optional[Callable] = None
    ):
        # Store callbacks
        if on_start:
            self.callbacks['start'] = on_start
        if on_stop:
            self.callbacks['stop'] = on_stop
        if on_reset:
            self.callbacks['reset'] = on_reset
        if on_speed_change:
            self.callbacks['speed_change'] = on_speed_change

        # if advanced:
        #     setup_advanced_control_panel(
        #         self.control_ids, parent,
        #         self.callbacks.get('start'),
        #         self.callbacks.get('stop'),
        #         self.callbacks.get('reset'),
        #         self.callbacks.get('speed_change')
        #     )
        # else:
        #     setup_control_panel(
        #         self.control_ids, parent,
        #         self.callbacks.get('start'),
        #         self.callbacks.get('stop'),
        #         self.callbacks.get('reset'),
        #         self.callbacks.get('speed_change')
        #     )