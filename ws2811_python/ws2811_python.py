# WS2811 Python 3 script controlling an Arduino via the specified COM that sends the WS2811 data to the WS2811 modules. Upload the 
# accompanying ws2811_arduino project onto an Arduino for it to communicate with it over a COM port, and then start the Python script with 
# the name of the Arduino's COM port as its only argument (e.g. "python ws2811_python.py COM3").
# Original repository: https://github.com/gdecarpentier/DIYRE01_UsingTheWS2811ForControllingLights
#
# Feel free to use, copy, merge or modify this software in any way you like provided you accept that this software comes without 
# any warranty, support or guarantees, and that its author is in no way liable for any potential or actual damage whatsoever.
# Attribution is very much appreciated but not required. Giliam de Carpentier, 2021. https://www.decarpentier.nl.

# IMPORTS - Import any software dependencies.
import serial					# The pySerial module is used for communication over COM ports. 
								# Install using: python -m pip install pyserial

import sys						# The sys module is used for retrieving the command line arguments

import time						# The time module is used for getting the currrent (high precision) time


# SETTINGS - Tweakable global settings
real_seconds_per_sim_hour = 0.5	# Real seconds in which 1 simulated hour will have passed.

real_filter_halflife = 0.02		# Real seconds after which 50% of a light value will have been filtered in/out.

refresh_rate = 60				# Times per real the second the light values are recalculated, and sent when possible.


# FUNCTION - Get the light states based on the given time, specified as the (floating point) hour of the day.
def get_light_floats_from_hour(hour):
	lights = []
		
	# Calculate the triplet of building light values based on the time of day. This assumes the building is the very first ws2811 the 
	# Arduino's data signal passes.
	lights.append(0.25 if ((hour > 15.0 or hour < 9.0)) else 0)                                    		
	lights.append(0.25 if ((hour > 17.0 and hour < 19.5) or (hour > 5.5 and hour < 8.0)) else 0)	
	lights.append(1.00 if ((hour > 4.0 and hour < 21.0)) else 0)    								
	
	# Calculate the colors in the sky dome. This assumes the dome is connected to the building's ws2811 data output.
	for i in range(8*6):
		# The light dome consists of a light strip serpentining from one side to the other, coming back to the same 'angle' every 16 chips.
		# The angle is represented by j, going 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5*, 7.5*, 5.5, 4.5, 3.5, 2.5, 1.5, 0.5, -0.5*, -1.5*. 
		# The values marked with a * are all roughly on both sides of the horizon, but area all 
		# slightly different to a more varied look.
		j = (i + 0.5) % 16
		if j >= 8: j = 14 - j;
		# Calculate the intensities of the 3 components in the sky: starlight, skylight and sunlight. Light values are randomly jittered 
		# later, when turning into 8-bit values, to replace banding artifacts with noise artifacts. But this can also be abused to 
		# generate a random starfield.
		stars = 0.01
		sky = (max(0, min(1, 0.15 * (9.5 - abs(hour - 12)))) ** 2)
		sun = max(0, 1.0 - 0.125 * (abs((12 - hour) * 1 + 1 * (j - 3)))) ** 2
		# Turn the intensities into a summed color and add it to the values to process and send out. Colors aren't very transferable 
		# from one set of LED strips to another, so this is just something that worked for my strip in particular, which is in 
		# Blue-Red-Green order.
		lights.append((min(1, stars + 0.3 * sky - 0.20 * sun)) ** 2) # Blue 	
		lights.append((min(1, stars + 0.3 * sky + 0.60 * sun)) ** 2) # Red
		lights.append((min(1, stars + 0.3 * sky + 0.15 * sun)) ** 2) # Green
	
	# Return the array of all scalar light values 
	return lights


# FUNCTION - Get a list of clamped 0...255 values from a list of floats
def float_to_clamped_byte(value, dither):
	return max(0, min(255, int(255 * value + dither)))


# FUNCTION - Turn the given list of 0...1 values into 0...255 values, adding some jittering per three to hide banding.
def get_bytes_from_floats(values):
	data = []
	num_triplets = int(len(values) / 3)
	dither = 0
	for i in range(num_triplets):
		# Update the dither pattern. There's many ways to do this, but this one is based on the golden ratio. 
		# This replaces the banding/snapping effect to 8-bit values with arguably less objectionable noise patterns.
		dither -= 0.61803398875
		if dither < 0: dither += 1
		
		# Prepare the next 3 float values as bytes
		data.append(float_to_clamped_byte(values[3 * i + 0], dither)) # Convert third in triplet as a 0-255 'green' value.
		data.append(float_to_clamped_byte(values[3 * i + 1], dither)) # Convert first in triplet as a 0-255 'red' value.
		data.append(float_to_clamped_byte(values[3 * i + 2], dither)) # Convert second in triplet as a 0-255 'blue' value.
		
	return data		


# FUNCTION - Turn the given list of bytes (in groups of 3) into a serial data stream that the Arduino understands.
def get_stream_from_bytes(bytes):
	data = []
	num_triplets = int(len(bytes) / 3)
	for i in range(num_triplets):
		
		# Get the next 3 bytes representing X, Y and Z, respectively. And XYZ could be RGB, GRB, BRG, etc.
		x = bytes[3 * i + 0]
		y = bytes[3 * i + 1]
		z = bytes[3 * i + 2]
				
		# Pack the three bytes as four 6-bit values, leaving bit 6 and 7 for message control.
		message_end = 1 if i == num_triplets - 1 else 0		# 1 if this is the very last triplet to send. 0 otherwise.
		data.append((x >> 2) & 0x3f)						# Packed byte 0: '00xxxxxx', MSB first
		data.append(((x & 0x03) << 4) | ((y >> 4) & 0x0f))	# Packed byte 1: '00xxyyyy', MSB first,
		data.append(((y & 0x0f) << 2) | ((z >> 6) & 0x03))	# Packed byte 2: '00yyyyzz', MSB first.
		data.append((z & 0x3f) | (message_end << 7))		# Packed byte 3: 'e0zzzzzz', MSB first, e being 'message_end'
		
	return data		
	
	
# FUNCTION - Generate text describing the current state in one human-readable string.
def get_human_readable_state(hour, bytes):
	# Make time string
	hours_only = int(hour)
	minutes_only = int((hour - hours_only) * 60)
	text = " Clock: {:02d}:{:02d}. Lights: |".format(hours_only, minutes_only)

	# Append the formatted light float values to 'text'
	for i in range(int(len(bytes) / 3)):
		text += " {:02x} {:02x} {:02x}.".format(bytes[i * 3 + 0], bytes[i * 3 + 1], bytes[i * 3 + 2])
	
	return text
		
	
# FUNCTION - Main function doing the day-night cycle and light update
def main_loop(serial_connection):
	sim_hour = 0							# simulated time at startup, in hours.

	# Calculate how much of the filtered light value will remain per refresh
	real_filter_weight = 0.5 ** (1.0 / (refresh_rate * real_filter_halflife))

	# Prepare state variables updated in the loop below
	next_refresh_time = time.perf_counter()
	filtered_floats = []

	while True:
		# Wait until the real time arrived to refresh the state again
		time.sleep(max(0, next_refresh_time - time.perf_counter()));
		
		# Calculate as many updates to get back to the current time
		while next_refresh_time - time.perf_counter() < 0:
			next_refresh_time += 1.0 / refresh_rate;
			sim_hour = (sim_hour + 1.0 / (refresh_rate * real_seconds_per_sim_hour)) % 24.0
							
			# Calculate the target value for each light, and also use it the 'old' value is there aren't any old values yet.
			unfiltered_floats = get_light_floats_from_hour(sim_hour)
			if len(filtered_floats) != len(unfiltered_floats):
				filtered_floats = unfiltered_floats
				
			# Ease in/out each target light value based on real_filter_halflife. This makes LEDs turn on/off more slowly, adding an 
			# 'incandescent light bulb' feel to them.
			for i in range(len(filtered_floats)):
				filtered_floats[i] = unfiltered_floats[i] + (filtered_floats[i] - unfiltered_floats[i]) * real_filter_weight

		# Convert the filtered light values into a byte stream and send it to the Arduino for further processing.
		bytes = get_bytes_from_floats(filtered_floats)
		stream = get_stream_from_bytes(bytes)
		serial_connection.write(stream)
		
		# Wait for the Arduino to send back a value, acknowlegding the data has been processed and it's ready to receive more.
		serial_connection.flush()
		serial_connection.read(1)
		
		# Use the following line to dump the time and light states on the same TTY line
		print(get_human_readable_state(sim_hour, bytes), end='\r', flush=True)
		
	
	
# ENTRY POINT - Try to open the communication channel to the Arduino and use that to start the main loop	
if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("To run, specify the name of the COM port of the Arduino (e.g. COM3, /dev/ttyUSB2, ...) as the first argument.")
		exit(1)
		
	# Try to open the specified COM port to the Arduino
	serial_name = sys.argv[1]
	serial_connection = None
	try:
		serial_connection = serial.Serial(port=serial_name, baudrate=115200, timeout=0.25, write_timeout=0.25)	
		print("Succesfully connected to COM port " + serial_name)
	except Exception as e:
		print("Failed to connect to COM port" + serial_name + ": " + str(e))
		exit(1)
	
	# Pass on the initialized COM connection to the main loop
	main_loop(serial_connection)
	