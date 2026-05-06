import dearpygui.dearpygui as dpg

class DearPyGuiApp:
    def __init__(self, title: str = "acmzzz motor simulation"):
        self.title = title
        self.window_tag = "MainWindow"


        # GUI item IDs
        self.series_ids = {}

    def setup(self):
        dpg.create_context()

        # create viewport
        dpg.create_viewport(title=self.title, width=1200, height=800)
        dpg.set_viewport_resize_callback(self._on_viewport_resize)

        # setup main window (autosize + fill viewport initially)
        with dpg.window(tag=self.window_tag, label="Motor Control",
                        width=1200, height=800, autosize=False):
            # self._setup_menu_bar()
            self._setup_polts()

        # setup viewport
        print("setup viewport")
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.maximize_viewport()

        # Don't use set_primary_window - use autosize instead
        # dpg.set_primary_window("MainWindow", True)

    def _on_viewport_resize(self, sender, app_data):
        """Update window size when viewport resizes."""
        # width = app_data[0]
        # height = app_data[1]
        # dpg.configure_item(self.window_tag, width=width, height=height - 30)  # minus menu bar height
        pass

    def _setup_menu_bar(self):
        with dpg.menu_bar():
            with dpg.menu(label="File"):
                dpg.add_menu_item(label="Save Data", callback=self._exit)
                dpg.add_menu_item(label="Exit", callback=self._exit)

    def _setup_polts(self):
        # speed plot - 定义主 x 轴
        with dpg.group(width=-1):
            dpg.add_text("Speed Response")
            with dpg.plot(label="Speed vs Time", height=200, width=-1):
                dpg.add_plot_legend()
                # 创建 x 轴
                dpg.add_plot_axis(dpg.mvXAxis, label="Time [s]", tag="x_axis_speed")
                with dpg.plot_axis(dpg.mvYAxis, label="Speed [rpm]", tag="y_axis_speed"):
                    self.series_ids['speed_actual'] = dpg.add_line_series([], [], label="Actual", parent="y_axis_speed")
                    self.series_ids['speed_cmd'] = dpg.add_line_series([], [], label="Command", parent="y_axis_speed")
        dpg.add_separator()

        # Torque plot
        with dpg.group(width=-1):
            dpg.add_text("Electromagnetic Torque")
            with dpg.plot(label="Torque vs Time", height=200, width=-1):
                dpg.add_plot_legend()
                dpg.add_plot_axis(dpg.mvXAxis, label="Time [s]", tag="x_axis_torque")
                with dpg.plot_axis(dpg.mvYAxis, label="Torque [Nm]", tag="y_axis_torque"):
                    self.series_ids['torque'] = dpg.add_line_series([], [], label="Tem", parent="y_axis_torque")

        # 设置 x 轴自动范围
        dpg.set_axis_limits_auto("x_axis_speed")
        dpg.set_axis_limits_auto("x_axis_torque")

    def _sync_x_axis(self):
        """同步两个 plot 的 x 轴范围"""
        try:
            limits_speed = dpg.get_axis_limits("x_axis_speed")
            limits_torque = dpg.get_axis_limits("x_axis_torque")
            # 只有范围不同时才更新，避免循环
            if limits_speed != limits_torque:
                dpg.set_axis_limits("x_axis_torque", limits_speed[0], limits_speed[1])
        except Exception:
            pass  # 轴可能还未初始化

    def _exit(self):
        dpg.destroy_context()

    def run(self):
        self.setup()

        # main loop
        while dpg.is_dearpygui_running():
            # 同步 x 轴范围
            self._sync_x_axis()
            dpg.render_dearpygui_frame()

        dpg.destroy_context()


def run_app(title: str = "acmzzz motor simulation"):
    app = DearPyGuiApp(title=title)
    app.run()

if __name__ == '__main__':
    run_app()