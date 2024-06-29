from nicegui import ui

class ToggleButton:
    """Togglebutton class"""
    def __init__(self, text:str, icon:str = None, disable=False, on_change=lambda a:None, tooltip:str=None):
        self.text = text
        self.color = 'rgb(15 23 42)'
        self.pressedColor = 'rgb(187 247 208)'
        self.pressed = False
        self.icon = icon        
        self.disable = disable
        self.tooltip = tooltip
        self.toggle()
        self.onchange = on_change
        
    @ui.refreshable
    def toggle(self):
        """Function to setup the toggle element itself"""
        with ui.button(text=self.text, icon=self.icon, on_click=self.handlePress, color=self.color if not self.pressed else self.pressedColor) as btn:
            btn.classes('px-5 m-[-0.2em] text-white')
            if not self.tooltip is None:
                btn.tooltip(self.tooltip)
    def handlePress(self, state=None, suppress = False):
        """Function to handle the press of the button or to silently change the state of the button with the suppress argument"""
        if state is None and not self.disable:
            self.pressed = not self.pressed
        if state == self.pressed:
            return
        elif not state is None:
            self.pressed = state
        if not suppress:
            self.onchange(self.pressed)
        self.toggle.refresh()
