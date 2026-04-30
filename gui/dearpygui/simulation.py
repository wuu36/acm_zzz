

from typing import Callable, Optional, Dict, List

from dataclasses import dataclass

@dataclass
class SimulationConfig:
    dt: float = 1e-4
    update_interval: float = 0.05
    max_points: int = 1000
    speed_ramp_rate: float = 10.0   # rpm per second


class SimulationConsole:
    def __init__(
            self,
            control_ids: Optional[Dict[str, int]] = None,
            series_ids: Optional[Dict[str, int]] = None,
            config: Optional[SimulationConfig] = None
    ):
        # state container
        self.time: float = 0.0

        # thread management
        self.running: bool = False


        # gui IDs
        self.control_ids = control_ids or {}
        self.series_ids = series_ids or {}

    def start(self):
        if not self.running:
            self.running = True
    
    def stop(self):
        self.running = False
    

def create_simulation_callbacks(console: SimulationConsole) -> Dict[str, Callable]:
    def on_start(sender, app_data):
        console.start()
        
    def on_stop(sender, app_data):
        console.stop()
    return {
        'start': on_start,
        'stop': on_stop,
    }