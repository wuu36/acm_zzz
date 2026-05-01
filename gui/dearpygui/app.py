
import dearpygui.dearpygui as dpg

class DearPyGuiApp:
    def __init__(self, title: str = "acmzzz motor simulation"):
        self.title = title

    def setup(self):
        dpg.create_context()

        # create viewport
        dpg.create_viewport(title=self.title, width=-1, height=-1)
        
        # setup themes

        # setup main window

        # setup viewport
        print("setup viewport")
        dpg.setup_dearpygui()
        dpg.show_viewport()






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