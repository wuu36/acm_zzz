
import dearpygui.dearpygui as dpg

class DearPyGuiApp:
    def __init__(self, title: str = "acmzzz motor simulation"):
        self.title = title

    def setup(self):
        dpg.create_context()

        # create viewport
        dpg.create_viewport(title=self.title, width=1200, height=800)
        
        # dpg.add_viewport_menu_bar()
        with dpg.viewport_menu_bar():
          with dpg.menu(label="File"):
              dpg.add_menu_item(label="New")
              dpg.add_menu_item(label="Open")
              dpg.add_menu_item(label="Save")
              dpg.add_separator()
              dpg.add_menu_item(label="Exit", callback=self._exit)

          with dpg.menu(label="View"):
              dpg.add_menu_item(label="Reset Layout")

          with dpg.menu(label="Help"):
              dpg.add_menu_item(label="About")
        # setup themes

        # setup main window

        # setup viewport
        print("setup viewport")
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.maximize_viewport()

    def _exit(self):
        pass





    def run(self):
        self.setup()

        # main loop
        while dpg.is_dearpygui_running():
            dpg.render_dearpygui_frame()
        
        dpg.destroy_context()



def run_app(title: str = "acmzzz motor simulation"):
    app = DearPyGuiApp(title=title)
    app.run()

if __name__ == '__main__':
    run_app()