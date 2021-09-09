// WS2811 Arduino sketch translating and relaying the commands from the WS2811 Python script to the connected WS2811 chips.
// Original repository: https://github.com/gdecarpentier/DIYRE01_UsingTheWS2811ForControllingLights
//
// Feel free to use, copy, merge or modify this software in any way you like provided you accept that this software comes without 
// any warranty, support or guarantees, and that its author is in no way liable for any potential or actual damage whatsoever.
// Attribution is very much appreciated but not required. Written by Giliam de Carpentier, 2021. http://www.decarpentier.nl.

#include <Adafruit_NeoPixel.h>  // Include the required 'NeoPixel' library from Adafruit to control the pin toggling
                                // This can be downloaded from: https://github.com/adafruit/Adafruit_NeoPixel

 
#define WS2811_COUNT    8       // Number of WS2811 chips to send data to
#define PIN             7       // Pin on which to send the WS2811 chip data over


byte channels[3*WS2811_COUNT];  // Received 8-bit value per channel
int channel_index       = 0;    // Channel index into the message stream
int byte_index          = 0;    // Byte index into the packed message stream
byte prev_byte          = 0;    // Byte received last


// Neopixel API object
Adafruit_NeoPixel pixels(WS2811_COUNT, PIN, NEO_GRB + NEO_KHZ800);


// Set up the NeoPixel API and Serial port
void setup() 
{
  pixels.begin();

  // Start serial port at 115200 bps and wait for port to open:
  Serial.begin(115200);
  while (!Serial) { }
}


// Process the incoming data and send it out to the connected WS2811 chips
void loop() 
{ 
  // Process any byte received
  while (Serial.available() > 0) 
  {
    // Get the next byte on the COM port
    byte curr_byte = Serial.read();

    // Interpret a high MSB as message restart, starting from channel 0 again
    if (curr_byte & 128)
    {
      channel_index = 0;
      byte_index = 0;
    }

    // Unpack the 6 lowest bits received as bits completing the previous 8-bit channel and/or current 8-bit channel.
    if (channel_index < 3*WS2811_COUNT)
    {
      switch (byte_index & 3)
      {
        default:  // First of four-byte pattern, containing '?0rrrrrr' (MSB first).
          break;
        case 1:   // Second of four-byte pattern, containing '00rrgggg' (MSB first).
          channels[channel_index++] = (prev_byte << 2) | ((curr_byte >> 4) & 0x03);
          break;  
        case 2:   // Third of four-byte pattern, containing '00ggggbb' (MSB first).
          channels[channel_index++] = (prev_byte << 4) | ((curr_byte >> 2) & 0x0f);
          break;
        case 3:   // Fourth of four-byte pattern, containing '00bbbbbb' (MSB first).
          channels[channel_index++] = (prev_byte << 6) | ((curr_byte     ) & 0x3f);
          break;
      }

      // Keep track of what to process next
      ++byte_index;
      prev_byte = curr_byte;
    }

    // Force an update of the LEDs before continuing any further.
    if (curr_byte & 128)
      break;
  } 
  
  // Send out the latest data to the WS2811 chips
  for (int i = 0; i < WS2811_COUNT; ++i)
    pixels.setPixelColor(i, pixels.Color(channels[i * 3 + 0], channels[i * 3 + 1], channels[i * 3 + 2]));
  pixels.show();
  
}
