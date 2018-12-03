This program is used to control the environment created using a 12V fan, a 5V Voltage Regulator, and a thermistor on an MSP430F5529.
The way that the program works is first the circuit using the fan, regulator, and thermistor must be set up. 
The thermistor is set up in a voltage divider where the input output goes to the ADC on the MSP430. 
The fan is set up to be controlled using a mosfet, a feedback diode, and 2 resistors. 
The regulator is set up using a circuit with a combination of parallel resistors with the capabality to disperse 2 Watts. 
First the program stops the watchdog timer, then the ADC, UART, and PWM are all initialized. 
Then in the while loop, the ADC starts sampling, the global interupts are enabled and low power mode is set, and setTemp is called.
setTemp is used to calculate the temperature using the reading from the ADC.
When the sampling begins, it begins storing the readings into ADC12MEM0, and when a goaltemp is set, it runs controlFan().
controlFan() is used to calculate the duty cycle of the fan using deltaTemp created using goalTemp and currentTemp.
Once data is sent through uart, the interupt vector goes off, and stores the recieve buffer into goalTemp and sends the current temp into the transmit buffer.
Then the program continues to change the PWM duty cycle to have it stay steady on the desired temperature.
