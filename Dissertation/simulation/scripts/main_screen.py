import kivy
from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout
from kivy.uix.textinput import TextInput
from kivy.uix.button import Button
# from kivy.uix.checkbox import CheckBox
from kivy.uix.dropdown import DropDown
from kivy.base import runTouchApp

class Main_GridLayout(GridLayout):
    def __init__(self, **kwargs):
        super(Main_GridLayout,self).__init__(**kwargs)

        self.cols = 1

        self.top_grid = GridLayout()
        self.top_grid.cols = 2

        self.top_grid.add_widget(Label(text="Robot Name: "))
        self.robot_name = TextInput(multiline=False)
        self.top_grid.add_widget(self.robot_name)

        self.top_grid.add_widget(Label(text="Quantity: "))
        self.qnt = TextInput(multiline=False)
        self.top_grid.add_widget(self.qnt)

        self.add_widget(self.top_grid)

        # self.topics = DropDown()
        # self.add_widget(self.topics)

        self.submit = Button(text="Submit",font_size=20)
        self.submit.bind(on_press=self.press)
        self.add_widget(self.submit)

    def press(self, instance):
        robot_name = self.robot_name.text
        qnt = self.qnt.text
        self.robot_name.text = ""
        self.qnt.text = ""

class main_gui(App):
    def build(self):
        return Main_GridLayout()

if __name__ == '__main__':
    main_gui().run()