


import dearpygui.dearpygui as dpg
from dataclasses import dataclass
import threading
import time
import numpy as np

@dataclass
class SimulationState:
    """State container for simulation data."""
    running: bool = False
    time: float = 0.0
    speed_rpm: float = 0.0
    speed_cmd_rpm: float = 0.0
    torque: float = 0.0
    id_current: float = 0.0
    iq_current: float = 0.0
    ud_voltage: float = 0.0
    uq_voltage: float = 0.0

    # data history for plotting
    time_history: list | None = None
    speed_history: list | None  = None
    torque_history: list | None = None

    def __post_init__(self):
        if self.time_history is None:
            self.time_history = []
        if self.speed_history is None:
            self.speed_history = []
        if self.torque_history is None:
            self.torque_history = []

class DearPyGuiApp:
    def __init__(self, title: str = "acmzzz motor simulation"):
        self.title = title
        self.state = SimulationState()
        self.simulation_thread = None
        self.callback = None
        self.stop_event = threading.Event()

        # gui item IDs
        self.plot_ids = {}
        self.series_ids = {}
        self.control_ids = {}
    
    def setup(self):
        """setup DearPyGui context and windows."""
        dpg.create_context()

        # create viewport (maximized)
        dpg.create_viewport(title=self.title, width=1920, height=1080)
        dpg.set_viewport_resize_callback(self._on_resize)

        # Setup themes
        self._setup_themes()

        # Setup main window (fill viewport)
        with dpg.window(tag="MainWindow", label="Motor Control", width=-1, height=-1):
            self._setup_menu_bar()
            self._setup_control_panel()
            self._setup_plots()

        # Setup viewport and maximize
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.maximize_viewport()

    def _setup_themes(self):
        """Setup color themes for components."""
        with dpg.theme(tag="plot_theme"):
            with dpg.theme_component(dpg.mvLineSeries):
                dpg.add_theme_color(dpg.mvPlot_LineColor, (0, 200, 255))
                dpg.add_theme_style(dpg.mvPlot_LineWeight, 2)

        with dpg.theme(tag="control_theme"):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvButton_ButtonColor, (70, 130, 180))

    def _setup_menu_bar(self):
        """Setup menu bar."""
        with dpg.menu_bar():
            with dpg.menu(label="File"):
                dpg.add_menu_item(label="Save Data", callback=self._save_data)
                dpg.add_menu_item(label="Exit", callback=self._exit)

            with dpg.menu(label="View"):
                dpg.add_menu_item(label="Reset Layout", callback=self._reset_layout)

            with dpg.menu(label="Help"):
                dpg.add_menu_item(label="About", callback=self._show_about)

    def _setup_control_panel(self):
        """Setup control panel with parameters and buttons."""
        with dpg.group(horizontal=True):
            # Left: Control buttons
            with dpg.group():
                dpg.add_text("Simulation Control")
                dpg.add_separator()

                self.control_ids['start_btn'] = dpg.add_button(
                    label="Start", callback=self._start_simulation, width=100
                )
                self.control_ids['stop_btn'] = dpg.add_button(
                    label="Stop", callback=self._stop_simulation, width=100
                )
                self.control_ids['reset_btn'] = dpg.add_button(
                    label="Reset", callback=self._reset_simulation, width=100
                )

                dpg.add_separator()
                dpg.add_text("Status:")
                self.control_ids['status_text'] = dpg.add_text("Stopped", color=(200, 200, 200))

            # Right: Parameters
            with dpg.group():
                dpg.add_text("Motor Parameters")
                dpg.add_separator()

                # Speed command
                dpg.add_text("Speed Command [rpm]:")
                self.control_ids['speed_cmd'] = dpg.add_slider_float(
                    label="Speed Cmd",
                    default_value=500.0,
                    min_value=0.0,
                    max_value=2000.0,
                    width=200,
                    callback=self._on_speed_change
                )

                # Load torque
                dpg.add_text("Load Torque [Nm]:")
                self.control_ids['load_torque'] = dpg.add_slider_float(
                    label="Load",
                    default_value=0.0,
                    min_value=0.0,
                    max_value=10.0,
                    width=200
                )

                dpg.add_separator()
                dpg.add_text("Controller Gains")

                self.control_ids['speed_kp'] = dpg.add_slider_float(
                    label="Speed Kp",
                    default_value=0.038,
                    min_value=0.001,
                    max_value=0.5,
                    width=200
                )

                self.control_ids['speed_ki'] = dpg.add_slider_float(
                    label="Speed Ki",
                    default_value=30.5,
                    min_value=1.0,
                    max_value=200.0,
                    width=200
                )

    def _setup_plots(self):
        """Setup plot windows for real-time data visualization."""
        # Speed plot
        with dpg.group():
            dpg.add_text("Speed Response")
            with dpg.plot(label="Speed vs Time", height=200, width=-1):
                dpg.add_plot_legend()
                dpg.add_plot_axis(dpg.mvXAxis, label="Time [s]", tag="x_axis_speed")
                with dpg.plot_axis(dpg.mvYAxis, label="Speed [rpm]", tag="y_axis_speed"):
                    self.series_ids['speed_actual'] = dpg.add_line_series(
                        [], [], label="Actual", parent="y_axis_speed"
                    )
                    self.series_ids['speed_cmd'] = dpg.add_line_series(
                        [], [], label="Command", parent="y_axis_speed"
                    )

        dpg.add_separator()

        # Torque plot
        with dpg.group():
            dpg.add_text("Electromagnetic Torque")
            with dpg.plot(label="Torque vs Time", height=150, width=-1):
                dpg.add_plot_legend()
                dpg.add_plot_axis(dpg.mvXAxis, label="Time [s]", tag="x_axis_torque")
                with dpg.plot_axis(dpg.mvYAxis, label="Torque [Nm]", tag="y_axis_torque"):
                    self.series_ids['torque'] = dpg.add_line_series(
                        [], [], label="Tem", parent="y_axis_torque"
                    )

        dpg.add_separator()

        # Currents plot
        with dpg.group():
            dpg.add_text("DQ Currents")
            with dpg.plot(label="Currents vs Time", height=150, width=-1):
                dpg.add_plot_legend()
                dpg.add_plot_axis(dpg.mvXAxis, label="Time [s]", tag="x_axis_current")
                with dpg.plot_axis(dpg.mvYAxis, label="Current [A]", tag="y_axis_current"):
                    self.series_ids['id'] = dpg.add_line_series(
                        [], [], label="iD", parent="y_axis_current"
                    )
                    self.series_ids['iq'] = dpg.add_line_series(
                        [], [], label="iQ", parent="y_axis_current"
                    )

    def _start_simulation(self, sender, app_data):
        """Start simulation thread."""
        if not self.state.running:
            self.state.running = True
            self.stop_event.clear()
            dpg.set_value(self.control_ids['status_text'], "Running")
            dpg.configure_item(self.control_ids['status_text'], color=(0, 255, 0))

            # Start simulation thread
            self.simulation_thread = threading.Thread(
                target=self._simulation_loop, daemon=True
            )
            self.simulation_thread.start()

    def _stop_simulation(self, sender, app_data):
        """Stop simulation."""
        self.state.running = False
        self.stop_event.set()
        dpg.set_value(self.control_ids['status_text'], "Stopped")
        dpg.configure_item(self.control_ids['status_text'], color=(200, 200, 200))

    def _reset_simulation(self, sender, app_data):
        """Reset simulation state."""
        self._stop_simulation(sender, app_data)

        # Clear state
        self.state.time = 0.0
        self.state.speed_rpm = 0.0
        self.state.torque = 0.0
        self.state.time_history.clear()
        self.state.speed_history.clear()
        self.state.torque_history.clear()

        # Clear plots
        for series_id in self.series_ids.values():
            dpg.set_value(series_id, [[], []])

    def _simulation_loop(self):
        """Background simulation loop."""
        dt = 1e-4  # Simulation timestep
        update_interval = 0.05  # GUI update interval

        last_update = time.time()

        while self.state.running and not self.stop_event.is_set():
            # Get parameters from GUI
            speed_cmd = dpg.get_value(self.control_ids['speed_cmd'])

            # Call external simulation callback if set
            if self.callback:
                self.callback(self.state, dt)
            else:
                # Default simulation: simple ramp
                self.state.time += dt
                if self.state.speed_rpm < speed_cmd:
                    self.state.speed_rpm += 10 * dt  # Simple ramp

            # Update history
            self.state.time_history.append(self.state.time)
            self.state.speed_history.append(self.state.speed_rpm)
            self.state.torque_history.append(self.state.torque)

            # Limit history length
            max_points = 1000
            if len(self.state.time_history) > max_points:
                self.state.time_history = self.state.time_history[-max_points:]
                self.state.speed_history = self.state.speed_history[-max_points:]
                self.state.torque_history = self.state.torque_history[-max_points:]

            # Update GUI periodically
            if time.time() - last_update >= update_interval:
                self._update_plots()
                last_update = time.time()

            # Small sleep to prevent blocking
            time.sleep(dt * 10)

    def _update_plots(self):
        """Update plot series with current data."""
        if len(self.state.time_history) > 0:
            # Speed plot
            speed_cmd = dpg.get_value(self.control_ids['speed_cmd'])
            speed_cmd_history = [speed_cmd] * len(self.state.time_history)

            dpg.set_value(
                self.series_ids['speed_actual'],
                [self.state.time_history, self.state.speed_history]
            )
            dpg.set_value(
                self.series_ids['speed_cmd'],
                [self.state.time_history, speed_cmd_history]
            )

            # Torque plot
            dpg.set_value(
                self.series_ids['torque'],
                [self.state.time_history, self.state.torque_history]
            )

            # Current plot (placeholder)
            id_history = [0.0] * len(self.state.time_history)
            iq_history = [self.state.speed_rpm * 0.01] * len(self.state.time_history)
            dpg.set_value(self.series_ids['id'], [self.state.time_history, id_history])
            dpg.set_value(self.series_ids['iq'], [self.state.time_history, iq_history])

    def _on_speed_change(self, sender, app_data):
        """Handle speed command change."""
        self.state.speed_cmd_rpm = app_data

    def _on_resize(self, sender, app_data):
        """Handle viewport resize."""
        pass

    def _save_data(self, sender, app_data):
        """Save simulation data to file."""
        print("Save data callback (not implemented)")

    def _exit(self, sender, app_data):
        """Exit application."""
        self._stop_simulation(sender, app_data)
        dpg.destroy_context()

    def _reset_layout(self, sender, app_data):
        """Reset window layout."""
        print("Reset layout callback")

    def _show_about(self, sender, app_data):
        """Show about dialog."""
        with dpg.window(label="About", modal=True, width=300, height=200):
            dpg.add_text("ACMSimPy - Motor Simulation GUI")
            dpg.add_text("Version: 0.1.0")
            dpg.add_separator()
            dpg.add_text("A Python-based AC motor simulation")
            dpg.add_text("and control visualization tool.")
            dpg.add_separator()
            dpg.add_button(label="Close", callback=lambda: dpg.delete_item("About"))

    def set_simulation_callback(self, callback):
        """Set external simulation function callback.

        Args:
            callback: Function taking (state, dt) as arguments
        """
        self.callback = callback

    def run(self):
        """Run the DearPyGui application."""
        self.setup()

        # Main loop
        while dpg.is_dearpygui_running():
            dpg.render_dearpygui_frame()

        dpg.destroy_context()


def run_app(title: str = "ACMSimPy Motor Simulation"):
    """Create and run DearPyGui application.

    Args:
        title: Window title
    """
    app = DearPyGuiApp(title=title)
    app.run()


if __name__ == '__main__':
    run_app()
