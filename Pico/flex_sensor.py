# Luke Orioli
from machine import ADC

class FlexSensor:
    def __init__(self, pin: ADC, supply_V: float, divider_R: float, sensor_calibration_table: dict[float, float] = None):
        self.pin = pin
        self.supply_V = supply_V
        self.divider_R = divider_R
        self.sensor_calibration_data = []
        if sensor_calibration_table is not None:
            sorted_calibration_data_by_flex = sorted(sensor_calibration_table.items())
            for flex_val, res in sorted_calibration_data_by_flex:
                self.sensor_calibration_data.append((res, flex_val))

    def get_resistance(self) -> float:
        # Get the raw ADC value from the sensor.
        adc_val = self.pin.read_u16()
        # Convert the ADC value into a voltage.
        adc_volt = self.supply_V * adc_val / 65535
        # Determine the sensor's current resistance using the voltage divider equation.
        if adc_volt == 0.0:
            adc_volt = 0.00001
        sensor_R = self.divider_R * ((self.supply_V / adc_volt) - 1.0)
        return sensor_R
    
    def get_flex(self) -> float:
        # The current flex value can only be determined if the calibration table is populated with atleast two values.
        assert(len(self.sensor_calibration_data) >= 2)
        # Get the flex sensor's current resistance.
        sensor_R = self.get_resistance()
        # Use the calibration table to determine the non-linear flex value by interpolating adjacent resistance entries.
        # Iterate over each resistance range in the table until the current resistance falls within the range.
        for index in range(len(self.sensor_calibration_data) - 1):
            upper_R = self.sensor_calibration_data[index][0]
            lower_R = self.sensor_calibration_data[index + 1][0]
            if (sensor_R >= lower_R) and (sensor_R <= upper_R):
                # Get the flex values that correspond to this resistance range.
                upper_R_flex = self.sensor_calibration_data[index][1]
                lower_R_flex = self.sensor_calibration_data[index + 1][1]
                # Interpolate the flex values to get the estimated flex value for the current resistance.
                m = (upper_R_flex - lower_R_flex) / (upper_R - lower_R)
                R_flex = m * (sensor_R - lower_R) + lower_R_flex
                return R_flex
        return None
    