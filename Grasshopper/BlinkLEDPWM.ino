/* LED Blink, Teensyduino Tutorial #1
   http://www.pjrc.com/teensy/tutorial.html
 
   This example code is in the public domain.
*/
//  use PWM-capable pins on Grasshopper
const uint8_t ledPin1 = A3; // Red
const uint8_t ledPin2 = A2; // Green
const uint8_t ledPin3 = A4; // Blue

const boolean invert = false; // set true if common anode, false if common cathode

uint8_t color = 0;        // a value from 0 to 255 representing the hue
uint8_t R, G, B;          // the Red Green and Blue color components
uint8_t brightness = 255; // 255 is maximum brightness

int ii = 0;

void setup() 
{

  for(ii = 0; ii < 256; ii++) {
  analogWrite(ledPin1, ii);
  delay(10);
  }
 
  for(ii = 0; ii < 256; ii++) {
  analogWrite(ledPin1, 255 - ii);
  delay(10);
  }

  for(ii = 0; ii < 256; ii++) {
  analogWrite(ledPin2, ii);
  delay(10);
  }
 
  for(ii = 0; ii < 256; ii++) {
  analogWrite(ledPin2, 255 - ii);
  delay(10);
  }

  for(ii = 0; ii < 256; ii++) {
  analogWrite(ledPin3, ii);
  delay(10);
  }
 
  for(ii = 0; ii < 256; ii++) {
  analogWrite(ledPin3, 255 - ii);
  delay(10);
  }
  
}

void loop() 
{
 for (color = 0; color < 256; color++) { // Slew through the color spectrum

  hueToRGB(color, brightness);  // call function to convert hue to RGB

  // write the RGB values to the pins
  analogWrite(ledPin1, R);
  analogWrite(ledPin2, G);
  analogWrite(ledPin3, B);

  delay(100);
 }
 
}


// Courtesy http://www.instructables.com/id/How-to-Use-an-RGB-LED/?ALLSTEPS
// function to convert a color to its Red, Green, and Blue components.

void hueToRGB(uint8_t hue, uint8_t brightness)
{
    uint16_t scaledHue = (hue * 6);
    uint8_t segment = scaledHue / 256; // segment 0 to 5 around the
                                            // color wheel
    uint16_t segmentOffset =
      scaledHue - (segment * 256); // position within the segment

    uint8_t complement = 0;
    uint16_t prev = (brightness * ( 255 -  segmentOffset)) / 256;
    uint16_t next = (brightness *  segmentOffset) / 256;

    if(invert)
    {
      brightness = 255 - brightness;
      complement = 255;
      prev = 255 - prev;
      next = 255 - next;
    }

    switch(segment ) {
    case 0:      // red
        R = brightness;
        G = next;
        B = complement;
    break;
    case 1:     // yellow
        R = prev;
        G = brightness;
        B = complement;
    break;
    case 2:     // green
        R = complement;
        G = brightness;
        B = next;
    break;
    case 3:    // cyan
        R = complement;
        G = prev;
        B = brightness;
    break;
    case 4:    // blue
        R = next;
        G = complement;
        B = brightness;
    break;
   case 5:      // magenta
    default:
        R = brightness;
        G = complement;
        B = prev;
    break;
    }
}

