#include <Adafruit_NeoPixel.h>

// Pins
//
#ifdef __AVR_ATtiny85__
#define PIN_NEOPIXEL 1
#define PIN_IR       2
#define PIN_SWITCH   3
#define PIN_JUMPER_1 0
#define PIN_JUMPER_2 4
#else
#define PIN_IR_VCC   2
#define PIN_IR_GND   3
#define PIN_IR       4
#define PIN_NEOPIXEL 5
#define PIN_SWITCH   6
#endif

#define NEOPIXEL_COUNT 12

#define MODE_SOLID   0
#define MODE_HUE     1
#define MODE_RANDOM 2
#define MODE_RAINBOW 3
#define MODE_MAX 3

#define JUMPER_OPEN 0
#define JUMPER_A    1
#define JUMPER_B    2

// Remote control codes
#define IR_DFROBOT_POWER            0xFD00FF
#define IR_DFROBOT_VOLUME_UP        0xFD807F
#define IR_DFROBOT_FUNCTION_STOP    0xFD40BF
#define IR_DFROBOT_REVERSE          0xFD20DF
#define IR_DFROBOT_PLAY_PAUSE       0xFDA05F
#define IR_DFROBOT_FORWARD          0xFD609F
#define IR_DFROBOT_DOWN             0xFD10EF
#define IR_DFROBOT_VOLUME_DOWN      0xFD906F
#define IR_DFROBOT_UP               0xFD50AF
#define IR_DFROBOT_0                0xFD30CF
#define IR_DFROBOT_EQ               0xFDB04F
#define IR_DFROBOT_ST_REPEAT        0xFD708F
#define IR_DFROBOT_1                0xFD08F7
#define IR_DFROBOT_2                0xFD8877
#define IR_DFROBOT_3                0xFD48B7
#define IR_DFROBOT_4                0xFD28D7
#define IR_DFROBOT_5                0xFDA857
#define IR_DFROBOT_6                0xFD6897
#define IR_DFROBOT_7                0xFD18E7
#define IR_DFROBOT_8                0xFD9867
#define IR_DFROBOT_9                0xFD58A7

#define IR_LED44_BRIGHTER           0xFF3AC5
#define IR_LED44_DIMMER             0xFFBA45
#define IR_LED44_PLAY               0xFF827D
#define IR_LED44_POWER              0xFF02FD
#define IR_LED44_RED                0xFF1AE5
#define IR_LED44_RED_ORANGE         0xFF2AD5
#define IR_LED44_ORANGE             0xFF0AF5
#define IR_LED44_YELLOW_ORANGE      0xFF38C7
#define IR_LED44_YELLOW             0xFF18E7
#define IR_LED44_GREEN              0xFF9A65
#define IR_LED44_GREEN_BLUE         0xFFAA55
#define IR_LED44_AQUA               0xFF8A75
#define IR_LED44_BLUE_GREEN         0xFFB847
#define IR_LED44_CYAN               0xFF9867
#define IR_LED44_BLUE               0xFFA25D
#define IR_LED44_PERIWINKLE         0xFF926D
#define IR_LED44_PURPLE             0xFFB24D
#define IR_LED44_PURPLE_RED         0xFF7887
#define IR_LED44_MAGENTA            0xFF58A7
#define IR_LED44_WHITE              0xFF22DD
#define IR_LED44_PINK1              0xFF12ED
#define IR_LED44_PINK2              0xFF32CD
#define IR_LED44_PALE_BLUE1         0xFFF807
#define IR_LED44_PALE_BLUE2         0xFFD827
#define IR_LED44_RED_UP             0xFF28D7
#define IR_LED44_RED_DOWN           0xFF08F7
#define IR_LED44_GREEN_UP           0xFFA857
#define IR_LED44_GREEN_DOWN         0xFF8877
#define IR_LED44_BLUE_UP            0xFF6897
#define IR_LED44_BLUE_DOWN          0xFF48B7
#define IR_LED44_DIY1               0xFF30CF
#define IR_LED44_DIY2               0xFFB04F
#define IR_LED44_DIY3               0xFF708F
#define IR_LED44_DIY4               0xFF10EF
#define IR_LED44_DIY5               0xFF906F
#define IR_LED44_DIY6               0xFF50AF
#define IR_LED44_QUICK              0xFFE817
#define IR_LED44_SLOW               0xFFC837
#define IR_LED44_AUTO               0xFFF00F
#define IR_LED44_FLASH              0xFFD02F
#define IR_LED44_JUMP3              0xFF20DF
#define IR_LED44_JUMP7              0xFFA05F
#define IR_LED44_FADE3              0xFF609F
#define IR_LED44_FADE7              0xFFE01F

#define IR_APPLETV_UP               0x77E1D05A
#define IR_APPLETV_DOWN             0x77E1B05A
#define IR_APPLETV_RIGHT            0x77E1E05A
#define IR_APPLETV_LEFT             0x77E1105A
#define IR_APPLETV_PLAY_PAUSE       0x77E17A5A
#define IR_APPLETV_MENU             0x77E1405A
#define IR_APPLETV_SELECT           0x77E1BA5A

// Globals
//
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
volatile unsigned long irValue = 0;
volatile bool irAvailable = false;

// IR receive interrupt
//
#ifdef __AVR_ATtiny85__
ISR(PCINT0_vect)       // interrupt service routine
#else // Assume regular Arduino
ISR (PCINT2_vect)
#endif
{          // called when PCINT0 changes state
  static unsigned long irIntermediate = 0;
  static int irBits = 0;
  static unsigned long lastTime = 0;
  unsigned long now = micros();
  unsigned long delta = now - lastTime;
  lastTime = now;
  if (delta > 20000) {
    irIntermediate = 0;
    irBits = 0;
  } else if (digitalRead(PIN_IR) == LOW) { // Low-to-high transition
    irIntermediate <<= 1;
    irIntermediate |= (delta > 800) ? 1 : 0;
    ++irBits;
    if (irBits == 33) {
      irValue = irIntermediate;
      irAvailable = true;
    }
  }
   return;
}

// Application variables class (includes global variables)
//
class App {
public:
  App();

  void setup(void);
  void loop(void);
  
  void setDefaults();
  bool readFromEEPROM();
  bool writeToEEPROM();
  
  unsigned char mode() { return mMode; }
  void setMode(int value) { mMode = bound(value, MODE_MAX); }
  void addMode(int value) { mMode += value; mMode %= (MODE_MAX + 1); }
  int hue() { return mHue; }
  void setHue(int value) { mHue = value % 360; }
  void addHue(int value) { setHue(mHue + value); }
  int saturation() { return mSaturation; }
  void setSaturation(int value) { mSaturation = bound(value); }
  void addSaturation(int value) { setSaturation(mSaturation + value); }
  int brightness() { return mBrightness; }
  void setBrightness(int value) { mBrightness = bound(value); }
  void addBrightness(int value) { setBrightness(mBrightness + value); }
  int speed() { return mSpeed; }
  void setSpeed(int value) { mSpeed = bound(value, 240, 60); }
  void addSpeed(int value) { setSpeed(mSpeed + value); }
  
  void setHsv(int hue, int saturation = -1, int brightness = -1);
  void addRgb(int red, int green, int blue);
private:
  int bound(int value, int max = 255, int min = 0);
  void hsvToRgb(int hue, int saturation, int brightness, int& red, int& green, int& blue);
  void rgbToHsv(int red, int green, int blue, int& hue, int& saturation, int& brightness);
  void setPixelColor(int n, unsigned char red, unsigned char green, unsigned char blue, unsigned int brightness = 256);
  void setPixelHueSat(int index, int hue, int saturation, int brightness);
  int checkJumper(int pin);
protected:
  unsigned char mMode;
  int mHue;
  int mSaturation;
  int mBrightness;
  int mSpeed;
};

App::App()
{
  if (!readFromEEPROM()) {
    setDefaults();
  }
}

void App::setup(void) {
#ifdef PIN_JUMPER_1  
  pinMode(PIN_JUMPER_1, INPUT);
  digitalWrite(PIN_JUMPER_1, HIGH);
#endif
#ifdef PIN_JUMPER_2  
  pinMode(PIN_JUMPER_2, INPUT);
  digitalWrite(PIN_JUMPER_2, HIGH);
#endif

  pinMode(PIN_SWITCH, INPUT);     // Switch is input
  digitalWrite(PIN_SWITCH, HIGH); // enable internal pullup

#ifdef PIN_IR_GND
  pinMode(PIN_IR_GND, OUTPUT);
  digitalWrite(PIN_IR_GND, LOW);
#endif
#ifdef PIN_IR_VCC
  pinMode(PIN_IR_VCC, OUTPUT);
  digitalWrite(PIN_IR_VCC, HIGH);
#endif

#ifdef PIN_JUMPER_1
  setMode(((checkJumper(PIN_JUMPER_1) == JUMPER_B) ? 1 : 0) + (checkJumper(PIN_JUMPER_2) == JUMPER_B ? 2 : 0));
#endif

  strip.begin();
  strip.show();
  
  pinMode(PIN_IR, INPUT);

#ifdef __AVR_ATtiny85__
  PCMSK |= (1<<PCINT2); // pin change mask: listen to portb bit 3
  GIMSK |= (1<<PCIE); // enable PCINT interrupt 
  sei();      // enable all interrupts
#else // Assume full Arduino
  *digitalPinToPCMSK(PIN_IR) |= bit (digitalPinToPCMSKbit(PIN_IR));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(PIN_IR)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(PIN_IR)); // enable interrupt for the group
  Serial.begin(9600);
  Serial.println("\nStarting...\n");
#endif
}

void App::loop(void) {
  static int lastButton = HIGH;
  static int timeIndex = 0;
  static bool power = true;
  
  int red;
  int green;
  int blue;  
  
  if (irAvailable) {
    irAvailable = false;
    switch (irValue) {
      // IPOD Remote
      case 0x77E1D05A: addBrightness(17); break;     // UP         77E1D05A
      case 0x77E1B05A: addBrightness(-17); break;       // DOWN       77E1B05A
      case 0x77E1E05A: addHue(-15); break;   // RIGHT      77E1E05A
      case 0x77E1105A: addHue(15); break;     // LEFT       77E1105A
      case 0x77E17A5A: addSpeed(20); break; // PLAY/PAUSE 77E17A5A / 77E1205A
      case 0x77E1405A: addSpeed(-20); break;   // MENU       77E1405A
      case 0x77E1BA5A: addMode(1); break; // SELECT     77E1BA5A / 77E1205A

      case IR_LED44_BRIGHTER: addBrightness(5); break;
      case IR_LED44_DIMMER: addBrightness(-5); break;
      case IR_LED44_PLAY: break; // TODO
      case IR_DFROBOT_POWER:
      case IR_LED44_POWER: power = !power; break;
      case IR_LED44_RED: setHsv(0, 255); break;
      case IR_LED44_RED_ORANGE: setHsv(15, 255); break;
      case IR_LED44_ORANGE: setHsv(30, 255); break;
      case IR_LED44_YELLOW_ORANGE: setHsv(45, 255); break;
      case IR_LED44_YELLOW: setHsv(60, 255); break;
      case IR_LED44_GREEN: setHsv(120, 255); break;
      case IR_LED44_GREEN_BLUE: setHsv(135, 255); break;
      case IR_LED44_AQUA: setHsv(150, 255); break;
      case IR_LED44_BLUE_GREEN: setHsv(165, 255); break;
      case IR_LED44_CYAN: setHsv(180, 255); break;
      case IR_LED44_BLUE: setHsv(240, 255); break;
      case IR_LED44_PERIWINKLE: setHsv(255, 255); break;
      case IR_LED44_PURPLE: setHsv(270, 255); break;
      case IR_LED44_PURPLE_RED: setHsv(285, 255); break;
      case IR_LED44_MAGENTA: setHsv(300, 255); break;
      case IR_LED44_WHITE: setSaturation(0); break;
      case IR_LED44_PINK1:
      case IR_LED44_PINK2: setHsv(0, 127); break;
      case IR_LED44_PALE_BLUE1:
      case IR_LED44_PALE_BLUE2: setHsv(240, 127); break;
      case IR_LED44_RED_UP: addRgb(17, 0, 0); break;
      case IR_LED44_RED_DOWN: addRgb(-17, 0, 0); break;
      case IR_DFROBOT_UP:
      case IR_LED44_GREEN_UP: addRgb(0, 17, 0); break;
      case IR_DFROBOT_DOWN:
      case IR_LED44_GREEN_DOWN: addRgb(0, -17, 0); break;
      case IR_LED44_BLUE_UP: addRgb(0, 0, 17); break;
      case IR_LED44_BLUE_DOWN: addRgb(0, 0, -17); break;
      case IR_LED44_DIY1: setMode(MODE_SOLID); break;
      case IR_LED44_DIY2: setMode(MODE_HUE); break;
      case IR_LED44_DIY3: setMode(MODE_RANDOM); break;
      case IR_LED44_DIY4: setMode(MODE_RAINBOW); break;
      case IR_LED44_DIY5: break;
      case IR_LED44_DIY6: break;
      case IR_LED44_QUICK: addSpeed(-20); break;
      case IR_LED44_SLOW: addSpeed(20); break;
      case IR_DFROBOT_FUNCTION_STOP:
      case IR_LED44_AUTO: addMode(1); break;
      case IR_LED44_FLASH: break;
      case IR_LED44_JUMP3: break;
      case IR_LED44_JUMP7: break;
      case IR_LED44_FADE3: break;
      case IR_LED44_FADE7: break;    
    }

    #define MODE_SOLID   0
#define MODE_HUE     1
#define MODE_RANDOM 2
#define MODE_RAINBOW 3
#ifndef __AVR_ATtiny85__
    Serial.print("0x");
    Serial.println(irValue, HEX);
//    Serial.print(" (");
//    Serial.print(hue());
//    Serial.print(",");
//    Serial.print(saturation());
//    Serial.print(",");
//    Serial.print(brightness());
//    Serial.println(")");
//    hsvToRgb(hue(), saturation(), brightness(), red, green, blue);
//    Serial.print(" --> (");
//    Serial.print(red);
//    Serial.print(",");
//    Serial.print(green);
//    Serial.print(",");
//    Serial.print(blue);
//    Serial.println(")");
#endif
  }
  
  strip.clear();
  if (power) {
    int i;
    if (mode() == MODE_HUE) {
      for (i = 0; i < NEOPIXEL_COUNT; ++i) {
        setPixelHueSat(i, hue(), saturation(), brightness());
      }
      addHue(5);
    } else if (mode() == MODE_SOLID) {
      for (i = 0; i < NEOPIXEL_COUNT; ++i) {
        setPixelHueSat(i, hue(), saturation(), brightness());
      }    
    } else if (mode() == MODE_RAINBOW) {
      for (i = 0; i < NEOPIXEL_COUNT; ++i) {
        setPixelHueSat((i + timeIndex) % NEOPIXEL_COUNT, i * 360 / NEOPIXEL_COUNT, saturation(), brightness()); 
      }
      if (++timeIndex >= NEOPIXEL_COUNT) timeIndex = 0;
    } else if (mode() == MODE_RANDOM) {
      setPixelHueSat(random(12), random(360), saturation(), brightness());
    }
  }
  strip.show();
  
  // Handle button pushes
  if (digitalRead(PIN_SWITCH) == LOW) {
    if (lastButton == HIGH) {
      addMode(1);
    }
    lastButton = LOW;
  } else {
    lastButton = HIGH;
  }

  delay(speed());
}

void App::setDefaults()
{
  mMode = MODE_SOLID;
  mHue = 0;
  mSaturation = 255;
  mBrightness = 127;
  mSpeed = 100;
}

bool App::readFromEEPROM()
{
  return false;
}

bool App::writeToEEPROM()
{
  return false;
}

void App::setHsv(int hue, int saturation, int brightness)
{
  setHue(hue);
  if (saturation >= 0) setSaturation(saturation);
  if (brightness >= 0) setBrightness(brightness);
}

int App::bound(int value, int max, int min)
{
    return (value > max) ? max : (value < min) ? min : value;
}

void App::addRgb(int red, int green, int blue)
{
  int r, g, b;
  hsvToRgb(mHue, mSaturation, mBrightness, r, g, b);
  r = bound(r + red);
  g = bound(g + green);
  b = bound(b + blue);
  rgbToHsv(r, g, b, mHue, mSaturation, mBrightness);
}

void App::hsvToRgb(int hue, int saturation, int brightness, int& red, int& green, int& blue) {
  int base;
  int hexant;
  int shade;
 
  if (saturation == 0) { // Acromatic color (gray). Hue doesn't matter.
    red = green = blue = brightness;
  } else {
    base = ((255 - saturation) * brightness) >> 8;
    hexant = hue / 60;
    shade = (hexant & 1)
      ? (((brightness - base) * (60 - (hue % 60))) / 60) + base
      : (((brightness - base) * (hue % 60)) / 60) + base;
 
    switch(hexant) {
      case 0:
        red = brightness;
        green = shade;
        blue = base;
        break;
      case 1:
        red = shade;
        green = brightness;
        blue = base;
        break;
      case 2:
        red = base;
        green = brightness;
        blue = shade;
        break;
      case 3:
        red = base;
        green = shade;
        blue = brightness;
        break;
      case 4:
        red = shade;
        green = base;
        blue = brightness;
        break;
      case 5:
        red = brightness;
        green = base;
        blue = shade;
        break;
    }
  }
}

void App::rgbToHsv(int red, int green, int blue, int& hue, int& saturation, int& brightness) {
  int min;
  int max;
  int delta;
  
  min = (red <= green & red <= blue) ? red : (green <= blue) ? green : blue;
  max = (red >= green & red >= blue) ? red : (green >= blue) ? green : blue;
  brightness = max; // Brightness

  delta = max - min;
  if (max != 0) {
    saturation = (255 * (unsigned int)delta) / max; // Saturation
  } else {
    // r = g = b = 0    // saturation = 0, brightness is undefined
    saturation = 0;
    hue = 0;
    return;
  }
  
  if (red == max) {
    hue = 60 * (green - blue) / delta;    // between yellow & magenta
  } else if (green == max) {
    hue = 120 + 60 * (blue - red) / delta;  // between cyan & yellow
  } else {
    hue = 240 + (red - green) / delta;  // between magenta & cyan
  }
  if (hue < 0) {
    hue += 360;
  }
}

void App::setPixelColor(int n, unsigned char red, unsigned char green, unsigned char blue, unsigned int brightness) {
  if (brightness >= 256) {
    strip.setPixelColor(n, red, green, blue);
  } else {
    strip.setPixelColor(n,
      (((unsigned int)red) * brightness) >> 8,
      (((unsigned int)green) * brightness) >> 8,
      (((unsigned int)blue) * brightness) >> 8
    );
  }
}

void App::setPixelHueSat(int index, int hue, int saturation, int brightness) { 
  int red;
  int green;
  int blue;
  hsvToRgb(hue, saturation, brightness, red, green, blue); 
  strip.setPixelColor(index, red, green, blue); 
}

int App::checkJumper(int pin) {
  int result = JUMPER_OPEN;
  if (digitalRead(pin) == LOW) {
    result = JUMPER_B;
  } else {
    pinMode(PIN_SWITCH, OUTPUT);
    digitalWrite(PIN_SWITCH, LOW);
    if (digitalRead(pin) == LOW) {
      result = JUMPER_A;
    }
    pinMode(PIN_SWITCH, INPUT);
    digitalWrite(PIN_SWITCH, HIGH);
  }
  return result;
}

// Application instance
App app;

void setup() {
  app.setup();
}

void loop() {
  app.loop();
}
