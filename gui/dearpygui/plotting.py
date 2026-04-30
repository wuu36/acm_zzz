
import dearpygui.dearpygui as dpg
from typing import Dict, List, Optional, Tuple


def setup_plots(
        plot_ids: Dict[str, int | str],
        series_ids: Dict[str, int | str],
        parent: str = "MainWindow"
) -> Tuple[Dict[str, int | str], Dict[str, int | str]]:
    # speed plot
    with dpg.group(parent=parent):
        dpg.add_text("Speed Response", parent=parent)
        with dpg.plot(label="Speed vs Time", height=400, width=-1, parent=parent):
            dpg.add_plot_legend()
            plot_ids['x_axis_speed'] = dpg.add_plot_axis(
                dpg.mvXAxis, label="Time [s]", tag="x_axis_speed"
            )
            with dpg.plot_axis(dpg.mvYAxis, label="Speed [rpm]", tag="y_axis_speed"):
                series_ids['speed_actual'] = dpg.add_line_series(
                    [], [], label="Actual", parent="y_axis_speed"
                )
                series_ids['speed_cmd'] = dpg.add_line_series(
                    [], [], label="Command", parent="y_axis_speed"
                )
    dpg.add_separator(parent=parent)
    return plot_ids, series_ids    

class PlottingManager:
    """Manages multiple plot windows for motor simulation.

    Provides a higher-level interface for creating, updating, and
    managing real-time plots. This class encapsulates the plot IDs
    and series IDs, making it easier to manage complex plotting scenarios.

    Attributes:
        plot_ids: Dictionary of plot axis IDs
        series_ids: Dictionary of line series IDs
        time_history: Time data history
        speed_history: Speed data history
        torque_history: Torque data history
        id_history: D-axis current history
        iq_history: Q-axis current history
        ud_history: D-axis voltage history
        uq_history: Q-axis voltage history
        max_points: Maximum history length

    Example:
        >>> manager = PlottingManager()
        >>> manager.setup("MainWindow")
        >>> manager.update(time=0.1, speed=500, torque=2.0)
        >>> manager.refresh_plots()
    """

    def __init__(self, max_points: int = 1000):
        """Initialize plotting manager.

        Args:
            max_points: Maximum number of points to keep in history
        """
        self.plot_ids: Dict[str, int | str] = {}
        self.series_ids: Dict[str, int | str] = {}
        self.max_points = max_points

        # Data histories
        self.time_history: List[float] = []
        self.speed_history: List[float] = []
        self.torque_history: List[float] = []
        self.id_history: List[float] = []
        self.iq_history: List[float] = []
        self.ud_history: List[float] = []
        self.uq_history: List[float] = []

        self.speed_cmd: float = 0.0
    
    def setup(self, parent: str = "MainWindow", include_voltage: bool = False):
        setup_plots(self.plot_ids, self.series_ids, parent)
        # if include_voltage:
            # setup