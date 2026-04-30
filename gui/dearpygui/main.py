
import sys
from pathlib import Path

print("hello dearpygui~")

# add project root to path for direct execution
if __name__ == '__main__' and __package__ is None:
    project_root = Path(__file__).resolve().parent.parent.parent
    sys.path.insert(0, str(project_root))
    from gui.dearpygui.simulation import SimulationConsole, SimulationConfig, create_simulation_callbacks
    from gui.dearpygui.plotting import PlottingManager
    from gui.dearpygui.controls import ControlPanelManger
else:
    print("2")

import dearpygui.dearpygui as dpg
import numpy as np

class MotorSimulationApp:
    def __init__(self, title: str = "acmzzz motor simulation"):
        self.title = title

        # managers
        self.plotting_manager = PlottingManager(max_points=1000)
        self.control_manager = ControlPanelManger()
        self.console = SimulationConsole(
            config=SimulationConfig(
                dt=1e-4,
                update_interval=0.05,
                max_points=1000
            )
        )

        # set motor simulation callback
    
    def setup(self):
        dpg.create_context()

        # create viewport (will be maximized)
        dpg.create_viewport(title=self.title, width=1920, height=1080)

        # setup themes
        self._setup_themes()

        # setup main window (will fill viewport via set_primary_window)
        with dpg.window(tag="mainwindow", label="motor control"):
            # setup control panel
            callbacks = create_simulation_callbacks(self.console)
            self.control_manager.setup(
                "mainwindow",
                on_start=callbacks['start'],
                on_stop=callbacks['stop'],
                # on_reset=callbacks['reset'],
                # on_speed_change=callbacks['speed_change']
            )

            # setup plots
            self.plotting_manager.setup("MainWindow")
            
        # link GUI IDs to console
        self.console.control_ids = self.control_manager.control_ids
        
        # setup viewport and maximize
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.maximize_viewport()

        # Set main window to fill viewport
        # dpg.set_primary_window("mainwindow", True)
        
    def _setup_themes(self):
        with dpg.theme(tag="plot_theme"):
            with dpg.theme_component(dpg.mvLineSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, (0, 200, 255))
                dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight, 2)

    def run(self):
        """run the application."""
        self.setup()
        
        # main loop
        while dpg.is_dearpygui_running():
            dpg.render_dearpygui_frame()
        
        dpg.destroy_context()

def run_motor_simulation():
    """create and run motor simulation  application."""
    app = MotorSimulationApp()
    app.run()

if __name__ == '__main__':
    run_motor_simulation()
