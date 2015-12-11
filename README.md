# thermostat_thermocouple
Arduino-based thermostat with heating and cooling outputs using a MAX6675 &amp; K-Type thermocouple with LCD, heat, cool, keyboard and non-volatile temperature store.

If temperature is below "Min" value, it turns the heater on. If the temperature is above the "Max" value it turns the cooler on. There is 0.5 degree C of hysteresis to avoid chatter.

A standard LCD/keyboard module can be fitted to allow in-flight control, and the settings can be saved to EEPROM with the "SELECT" button.

If the thermocouple cannot be read, the heat & cool outputs are turned off and the program halts.
