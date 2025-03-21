import board
import time
from joystick_xl.inputs import Button
from joystick_xl.joystick import Joystick
from digitalio import DigitalInOut, Direction, Pull

class StrangeButton:
    def __init__(self, button, io, light=None, invert=True):
        self.io = DigitalInOut(io)
        self.io.direction = Direction.INPUT
        self.io.pull = Pull.UP
        self.invert = invert
        self.button = button
        if light is not None:
            self.light = DigitalInOut(light)
            self.light.direction = Direction.OUTPUT
        else:
            self.light = None

    def get(self):
        if self.invert:
            return not self.io.value
        else:
            return self.io.value

    def set(self, value):
        if self.light is not None:
            self.light.value = value

class ButtonArray:
    def __init__(self, bList):
        self.bList = bList
        self.pressed = None

    def update(self):
        for i in self.bList:
            if i.get():
                self.pressed = i.button
                break
        for i in self.bList:
            if i.button == self.pressed:
                i.set(True)
                js.update_button((i.button, True))
            else:
                i.set(False)
                js.update_button((i.button, False))

class SingleButton:
    def __init__(self, button):
        self.button = button

    def update(self):
        if self.button.get():
            self.button.set(True)
            js.update_button((self.button.button, True))
        else:
            self.button.set(False)
            js.update_button((self.button.button, False))

js = Joystick()
heightButtons = [
    StrangeButton(0, board.D7, board.D6),
    StrangeButton(1, board.D30, board.D2),
    StrangeButton(2, board.D29, board.D3),
    StrangeButton(3, board.D28, board.D4),
    StrangeButton(4, board.D32, board.D17),
]
ejectCButton = StrangeButton(5, board.D39)
ejectAButton = StrangeButton(6, board.D38)
sideButtons = [
    StrangeButton(7, board.D36, board.D47),
    StrangeButton(8, board.D35, board.D45)
]
algaeButton = StrangeButton(9, board.D40, invert=False)
climbButton = StrangeButton(10, board.D33, board.D8)
climbAuth = StrangeButton(11, board.D37)
climbDown = StrangeButton(12, board.D34, board.D9)
forceElevator = StrangeButton(13, board.D11, board.D12)
buttons = [
    ButtonArray(heightButtons),
    SingleButton(ejectCButton),
    SingleButton(ejectAButton),
    ButtonArray(sideButtons),
    SingleButton(algaeButton),
    SingleButton(climbButton),
    SingleButton(climbAuth),
    SingleButton(climbDown),
    SingleButton(forceElevator),
]

while True:
    for i in buttons:
        i.update()
    js.update()
