# Luke Orioli
from machine import Pin

class DigitalSensor:
    def __init__(self, input_pin: Pin, active_low: bool = False):
        self.pin = input_pin
        self.active_low = active_low
    
    def value(self):
        val = self.pin.value()
        if self.active_low:
            return val ^ 1
        return val
    