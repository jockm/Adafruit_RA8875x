/*!
 * @file     Adafruit_RA8875.cpp
 *
 * @mainpage Adafruit RA8875 TFT Driver
 *
 * @author   Limor Friend/Ladyada, K.Townsend/KTOWN for Adafruit Industries
 *
 * @section intro_sec Introduction
 *
 * This is the library for the Adafruit RA8875 Driver board for TFT displays
 * ---------------> http://www.adafruit.com/products/1590
 * The RA8875 is a TFT driver for up to 800x480 dotclock'd displays
 * It is tested to work with displays in the Adafruit shop. Other displays
 * may need timing adjustments and are not guanteed to work.
 *
 * Adafruit invests time and resources providing this open
 * source code, please support Adafruit and open-source hardware
 * by purchasing products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, check license.txt for more information.
 * All text above must be included in any redistribution.
 *
 * @section  HISTORY
 *
 * v1.0 - First release
 *
 */

#include "Adafruit_RA8875x.h"

/// @cond DISABLE
#if defined(EEPROM_SUPPORTED)
/// @endcond
#include <EEPROM.h>
/// @cond DISABLE
#endif
/// @endcond

#include <SPI.h>

/// @cond DISABLE
#if defined(ARDUINO_ARCH_ARC32)
/// @endcond
uint32_t spi_speed = 12000000; /*!< 12MHz */
/// @cond DISABLE
#else
/// @endcond
uint32_t spi_speed = 4000000; /*!< 4MHz */
                              /// @cond DISABLE
#endif
/// @endcond

// If the SPI library has transaction support, these functions
// establish settings and protect from interference from other
// libraries.  Otherwise, they simply do nothing.
#ifdef SPI_HAS_TRANSACTION
static inline void spi_begin(void) __attribute__((always_inline));
static inline void spi_begin(void) {
  // max speed!
  SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
}
static inline void spi_end(void) __attribute__((always_inline));
static inline void spi_end(void) { SPI.endTransaction(); }
#else
#define spi_begin() ///< Create dummy Macro Function
#define spi_end()   ///< Create dummy Macro Function
#endif

/**************************************************************************/
/*!
      Constructor for a new RA8875 instance

      @param CS  Location of the SPI chip select pin
      @param RST Location of the reset pin
*/
/**************************************************************************/
Adafruit_RA8875x::Adafruit_RA8875x(uint8_t CS, uint8_t RST)
    : Adafruit_GFX(800, 480) {
  _cs = CS;
  _rst = RST;
}

/**************************************************************************/
/*!
      Initialises the LCD driver and any HW required by the display

      @param s The display size, which can be either:
                  'RA8875_480x80'  (3.8" displays) or
                  'RA8875_480x128' (3.9" displays) or
                  'RA8875_480x272' (4.3" displays) or
                  'RA8875_800x480' (5" and 7" displays)

      @return True if we reached the end
*/
/**************************************************************************/
boolean Adafruit_RA8875x::begin(enum RA8875sizes s) {
  _size = s;

  if (_size == RA8875_480x80) {
    _width = 480;
    _height = 80;
  } else if (_size == RA8875_480x128) {
    _width = 480;
    _height = 128;
  } else if (_size == RA8875_480x272) {
    _width = 480;
    _height = 272;
  } else if (_size == RA8875_800x480) {
    _width = 800;
    _height = 480;
  } else {
    return false;
  }
  _rotation = 0;
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);
  pinMode(_rst, OUTPUT);

  SPI.begin();

  if(_rst == 0xFF) {
	  this->softReset();
  } else {
	  digitalWrite(_rst, LOW);
	  delay(100);
	  digitalWrite(_rst, HIGH);
	  delay(100);
  }

#ifdef SPI_HAS_TRANSACTION
/// @cond DISABLE
#if defined(ARDUINO_ARCH_ARC32)
  /// @endcond
  spi_speed = 2000000;
/// @cond DISABLE
#else
  /// @endcond
  spi_speed = 125000;
/// @cond DISABLE
#endif
/// @endcond
#else
#ifdef __AVR__
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.setDataMode(SPI_MODE0);
#endif
#endif

  uint8_t x = readReg(0);
  //    Serial.print("x = 0x"); Serial.println(x,HEX);
  if (x != 0x75) {
    Serial1.println(x, HEX);
    return false;
  }

  initialize();

#ifdef SPI_HAS_TRANSACTION
/// @cond DISABLE
#	if defined(ARDUINO_ARCH_ARC32)
/// @endcond
	  spi_speed = 12000000L;
#	else
	  spi_speed = 4000000L;
#	endif
#else
#	ifdef __AVR__
	  SPI.setClockDivider(SPI_CLOCK_DIV4);
#	endif
#endif

  // Just setting this manually
  // spi_speed = 12000000L;

  return true;
}

/************************* Initialization *********************************/

/**************************************************************************/
/*!
      Performs a SW-based reset of the RA8875
*/
/**************************************************************************/
void Adafruit_RA8875x::softReset(void) {
  writeCommand(RA8875_PWRR);
  writeData(RA8875_PWRR_SOFTRESET);
  writeData(RA8875_PWRR_NORMAL);
  delay(1);
}

/**************************************************************************/
/*!
      Initialise the PLL
*/
/**************************************************************************/
void Adafruit_RA8875x::PLLinit(void) {
	if (_size == RA8875_480x80 || _size == RA8875_480x128 || _size == RA8875_480x272) {
		writeReg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 10);
		delay(1);
		writeReg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
		delay(1);
	} else /* (_size == RA8875_800x480) */ {
		writeReg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 11);
		delay(1);
		writeReg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
		delay(1);
	}
}

/**************************************************************************/
/*!
      Initialises the driver IC (clock setup, etc.)
*/
/**************************************************************************/
void Adafruit_RA8875x::initialize(void) {
  // WAS PLLinit();
  // WAS writeReg(RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);
  // WAS writeReg(RA8875_SYSR, 0x0C);

  /* Timing values */
  uint8_t pixclk;
  uint8_t hsync_start;
  uint8_t hsync_pw;
  uint8_t hsync_finetune;
  uint8_t hsync_nondisp;
  uint8_t vsync_pw;
  uint16_t vsync_nondisp;
  uint16_t vsync_start;

  /* Set the correct values for the display being used */
  if (_size == RA8875_480x80) {
    pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
    hsync_nondisp = 10;
    hsync_start = 8;
    hsync_pw = 48;
    hsync_finetune = 0;
    vsync_nondisp = 3;
    vsync_start = 8;
    vsync_pw = 10;
    _voffset = 192; // This uses the bottom 80 pixels of a 272 pixel controller
  } else if (_size == RA8875_480x128 || _size == RA8875_480x272) {
    pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
    hsync_nondisp = 10;
    hsync_start = 8;
    hsync_pw = 48;
    hsync_finetune = 0;
    vsync_nondisp = 3;
    vsync_start = 8;
    vsync_pw = 10;
    _voffset = 0;
  } else // (_size == RA8875_800x480)
  {
    // was  pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
    pixclk = 0x81;
    hsync_nondisp = 26;
    hsync_start = 32;
    hsync_pw = 96;
    hsync_finetune = 0;
    vsync_nondisp = 32;
    vsync_start = 23;
    vsync_pw = 2;
    _voffset = 0;
  }

  writeReg(RA8875_PCSR, pixclk);
  delay(1);

#if 0
  // This is adafruit's code
  
  /* Horizontal settings registers */
  writeReg(RA8875_HDWR, (_width / 8) - 1); // H width: (HDWR + 1) * 8 = 480
  writeReg(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
  writeReg(RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2) /
                            8); // H non-display: HNDR * 8 + HNDFTR + 2 = 10
  writeReg(RA8875_HSTR, hsync_start / 8 - 1); // Hsync start: (HSTR + 1)*8
  writeReg(RA8875_HPWR,
           RA8875_HPWR_LOW +
               (hsync_pw / 8 - 1)); // HSync pulse width = (HPWR+1) * 8

  /* Vertical settings registers */
  writeReg(RA8875_VDHR0, (uint16_t)(_height - 1 + _voffset) & 0xFF);
  writeReg(RA8875_VDHR1, (uint16_t)(_height - 1 + _voffset) >> 8);
  writeReg(RA8875_VNDR0, vsync_nondisp - 1); // V non-display period = VNDR + 1
  writeReg(RA8875_VNDR1, vsync_nondisp >> 8);
  writeReg(RA8875_VSTR0, vsync_start - 1); // Vsync start position = VSTR + 1
  writeReg(RA8875_VSTR1, vsync_start >> 8);
  writeReg(RA8875_VPWR,
           RA8875_VPWR_LOW + vsync_pw - 1); // Vsync pulse width = VPWR + 1
#else
if (_rst == 255) {//soft reset
	writeCommand(RA8875_PWRR);
	writeData(RA8875_PWRR_SOFTRESET);
	writeData(RA8875_PWRR_NORMAL);
	delay(200);
}
  
  // Derived from buydisplay's code
  const uint8_t initCodes[] = {0x10,0x02,0x81,0x63,0x00,0x03,0x03,0x0B,0xDF,0x01,0x1F,0x00,0x16,0x00,0x01};
  
  writeReg(RA8875_PLLC1, initCodes[0]);////PLL Control Register 1
  delay(1);
  writeReg(RA8875_PLLC2, initCodes[1]);////PLL Control Register 2
  delay(1);
  
  writeReg(RA8875_PCSR,  initCodes[2]);//Pixel Clock Setting Register
  delay(1);
  writeReg(RA8875_SYSR,  0x0C);//we are working ALWAYS at 65K color space!!!!
  
  writeReg(RA8875_HDWR,  initCodes[3]);//LCD Horizontal Display Width Register
  writeReg(RA8875_HNDFTR,initCodes[4]);//Horizontal Non-Display Period Fine Tuning Option Register
  writeReg(RA8875_HNDR,  initCodes[5]);////LCD Horizontal Non-Display Period Register
  writeReg(RA8875_HSTR,  initCodes[6]);////HSYNC Start Position Register
  writeReg(RA8875_HPWR,  initCodes[7]);////HSYNC Pulse Width Register
  writeReg(RA8875_VDHR0, initCodes[8]);////LCD Vertical Display Height Register0
  writeReg(RA8875_VDHR1, initCodes[9]);////LCD Vertical Display Height Register1
  writeReg(RA8875_VNDR0, initCodes[10]);////LCD Vertical Non-Display Period Register 0
  writeReg(RA8875_VNDR1, initCodes[11]);////LCD Vertical Non-Display Period Register 1
  writeReg(RA8875_VSTR0, initCodes[12]);////VSYNC Start Position Register 0
  writeReg(RA8875_VSTR1, initCodes[13]);////VSYNC Start Position Register 1
  writeReg(RA8875_VPWR,  initCodes[14]);////VSYNC Pulse Width Register
  
#endif

  this->displayOn(true);
  /* Set active window X */
  writeReg(RA8875_HSAW0, 0); // horizontal start point
  writeReg(RA8875_HSAW1, 0);
  writeReg(RA8875_HEAW0, (uint16_t)(_width - 1) & 0xFF); // horizontal end point
  writeReg(RA8875_HEAW1, (uint16_t)(_width - 1) >> 8);

  /* Set active window Y */
  writeReg(RA8875_VSAW0, 0 + _voffset); // vertical start point
  writeReg(RA8875_VSAW1, 0 + _voffset);
  writeReg(RA8875_VEAW0,
           (uint16_t)(_height - 1 + _voffset) & 0xFF); // vertical end point
  writeReg(RA8875_VEAW1, (uint16_t)(_height - 1 + _voffset) >> 8);

  /* ToDo: Setup touch panel? */

  /* Clear the entire window */
  writeReg(RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
  
  delay(500);
}

/**************************************************************************/
/*!
      Returns the display width in pixels

      @return  The 1-based display width in pixels
*/
/**************************************************************************/
uint16_t Adafruit_RA8875x::width(void) { return _width; }

/**************************************************************************/
/*!
      Returns the display height in pixels

      @return  The 1-based display height in pixels
*/
/**************************************************************************/
uint16_t Adafruit_RA8875x::height(void) { return _height; }

/**************************************************************************/
/*!
 Returns the current rotation (0-3)

 @return  The Rotation Setting
 */
/**************************************************************************/
int8_t Adafruit_RA8875x::getRotation(void) { return _rotation; }

/**************************************************************************/
/*!
 Sets the current rotation (0-3)

 @param rotation The Rotation Setting
 */
/**************************************************************************/
void Adafruit_RA8875x::setRotation(int8_t rotation) {
  switch (rotation) {
  case 2:
    _rotation = rotation;
    break;
  default:
    _rotation = 0;
    break;
  }
}

/************************* Text Mode ***********************************/

/**************************************************************************/
/*!
      Sets the display in text mode (as opposed to graphics mode)
*/
/**************************************************************************/
void Adafruit_RA8875x::textMode(void) {
  /* Set text mode */
  writeCommand(RA8875_MWCR0);
  uint8_t temp = readData();
  temp |= RA8875_MWCR0_TXTMODE; // Set bit 7
  writeData(temp);

  /* Select the internal (ROM) font */
  writeCommand(0x21);
  temp = readData();
  temp &= ~((1 << 7) | (1 << 5)); // Clear bits 7 and 5
  writeData(temp);
}

/**************************************************************************/
/*!
      Sets the display in text mode (as opposed to graphics mode)

      @param x The x position of the cursor (in pixels, 0..1023)
      @param y The y position of the cursor (in pixels, 0..511)
*/
/**************************************************************************/
void Adafruit_RA8875x::textSetCursor(uint16_t x, uint16_t y) {
  x = applyRotationX(x);
  y = applyRotationY(y);

  /* Set cursor location */
  writeCommand(0x2A);
  writeData(x & 0xFF);
  writeCommand(0x2B);
  writeData(x >> 8);
  writeCommand(0x2C);
  writeData(y & 0xFF);
  writeCommand(0x2D);
  writeData(y >> 8);
}

/**************************************************************************/
/*!
      Sets the fore and background color when rendering text

      @param foreColor The RGB565 color to use when rendering the text
      @param bgColor   The RGB565 colot to use for the background
*/
/**************************************************************************/
void Adafruit_RA8875x::textColor(uint16_t foreColor, uint16_t bgColor) {
  /* Set Fore Color */
  writeCommand(0x63);
  writeData((foreColor & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((foreColor & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((foreColor & 0x001f));

  /* Set Background Color */
  writeCommand(0x60);
  writeData((bgColor & 0xf800) >> 11);
  writeCommand(0x61);
  writeData((bgColor & 0x07e0) >> 5);
  writeCommand(0x62);
  writeData((bgColor & 0x001f));

  /* Clear transparency flag */
  writeCommand(0x22);
  uint8_t temp = readData();
  temp &= ~(1 << 6); // Clear bit 6
  writeData(temp);
}

/**************************************************************************/
/*!
      Sets the fore color when rendering text with a transparent bg

      @param foreColor The RGB565 color to use when rendering the text
*/
/**************************************************************************/
void Adafruit_RA8875x::textTransparent(uint16_t foreColor) {
  /* Set Fore Color */
  writeCommand(0x63);
  writeData((foreColor & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((foreColor & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((foreColor & 0x001f));

  /* Set transparency flag */
  writeCommand(0x22);
  uint8_t temp = readData();
  temp |= (1 << 6); // Set bit 6
  writeData(temp);
}

/**************************************************************************/
/*!
      Sets the text enlarge settings, using one of the following values:

      0 = 1x zoom
      1 = 2x zoom
      2 = 3x zoom
      3 = 4x zoom

      @param scale   The zoom factor (0..3 for 1-4x zoom)
*/
/**************************************************************************/
void Adafruit_RA8875x::textEnlarge(uint8_t scale) {
  if (scale > 3)
    scale = 3; // highest setting is 3

  /* Set font size flags */
  writeCommand(0x22);
  uint8_t temp = readData();
  temp &= ~(0xF); // Clears bits 0..3
  temp |= scale << 2;
  temp |= scale;

  writeData(temp);

  _textScale = scale;
}

/**************************************************************************/
/*!
     Enable Cursor Visibility and Blink
     Here we set bits 6 and 5 in 40h
     As well as the set the blink rate in 44h
     The rate is 0 through max 255
     the lower the number the faster it blinks (00h is 1 frame time,
     FFh is 256 Frames time.
     Blink Time (sec) = BTCR[44h]x(1/Frame_rate)

     @param rate The frame rate to blink
 */
/**************************************************************************/

void Adafruit_RA8875x::cursorBlink(uint8_t rate) {

  writeCommand(RA8875_MWCR0);
  uint8_t temp = readData();
  temp |= RA8875_MWCR0_CURSOR;
  writeData(temp);

  writeCommand(RA8875_MWCR0);
  temp = readData();
  temp |= RA8875_MWCR0_BLINK;
  writeData(temp);

  if (rate > 255)
    rate = 255;
  writeCommand(RA8875_BTCR);
  writeData(rate);
}

/**************************************************************************/
/*!
      Renders some text on the screen when in text mode

      @param buffer    The buffer containing the characters to render
      @param len       The size of the buffer in bytes
*/
/**************************************************************************/
void Adafruit_RA8875x::textWrite(const char *buffer, uint16_t len) {
  if (len == 0)
    len = strlen(buffer);
  writeCommand(RA8875_MRWC);
  for (uint16_t i = 0; i < len; i++) {
    writeData(buffer[i]);
/// @cond DISABLE
#if defined(__arm__)
    /// @endcond
    // This delay is needed with textEnlarge(1) because
    // Teensy 3.X is much faster than Arduino Uno
    if (_textScale > 0)
      delay(1);
/// @cond DISABLE
#else
    /// @endcond
    // For others, delay starting with textEnlarge(2)
    if (_textScale > 1)
      delay(1);
/// @cond DISABLE
#endif
    /// @endcond
  }
}

/************************* Graphics ***********************************/

/**************************************************************************/
/*!
      Sets the display in graphics mode (as opposed to text mode)
*/
/**************************************************************************/
void Adafruit_RA8875x::graphicsMode(void) {
  writeCommand(RA8875_MWCR0);
  uint8_t temp = readData();
  temp &= ~RA8875_MWCR0_TXTMODE; // bit #7
  writeData(temp);
}

/**************************************************************************/
/*!
      Waits for screen to finish by polling the status!

      @param regname The register name to check
      @param waitflag The value to wait for the status register to match

      @return True if the expected status has been reached
*/
/**************************************************************************/
boolean Adafruit_RA8875x::waitPoll(uint8_t regname, uint8_t waitflag) {
  /* Wait for the command to finish */
  while (1) {
    uint8_t temp = readReg(regname);
    if (!(temp & waitflag))
      return true;
  }
  return false; // MEMEFIX: yeah i know, unreached! - add timeout?
}

/**************************************************************************/
/*!
      Sets the current X/Y position on the display before drawing

      @param x The 0-based x location
      @param y The 0-base y location
*/
/**************************************************************************/
void Adafruit_RA8875x::setXY(uint16_t x, uint16_t y) {
  writeReg(RA8875_CURH0, x);
  writeReg(RA8875_CURH1, x >> 8);
  writeReg(RA8875_CURV0, y);
  writeReg(RA8875_CURV1, y >> 8);
}

/**************************************************************************/
/*!
      HW accelerated function to push a chunk of raw pixel data

      @param num The number of pixels to push
      @param p   The pixel color to use
*/
/**************************************************************************/
void Adafruit_RA8875x::pushPixels(uint32_t num, uint16_t p) {
  digitalWrite(_cs, LOW);
  SPI.transfer(RA8875_DATAWRITE);
  while (num--) {
    SPI.transfer(p >> 8);
    SPI.transfer(p);
  }
  digitalWrite(_cs, HIGH);
}

/**************************************************************************/
/*!
    Fill the screen with the current color
*/
/**************************************************************************/
void Adafruit_RA8875x::fillRect(void) {
  writeCommand(RA8875_DCR);
  writeData(RA8875_DCR_LINESQUTRI_STOP | RA8875_DCR_DRAWSQUARE);
  writeData(RA8875_DCR_LINESQUTRI_START | RA8875_DCR_FILL |
            RA8875_DCR_DRAWSQUARE);
}

/**************************************************************************/
/*!
    Apply current rotation in the X direction

    @return the X value with current rotation applied
 */
/**************************************************************************/
int16_t Adafruit_RA8875x::applyRotationX(int16_t x) {
  switch (_rotation) {
  case 2:
    x = _width - 1 - x;
    break;
  }

  return x;
}

/**************************************************************************/
/*!
    Apply current rotation in the Y direction

    @return the Y value with current rotation applied
 */
/**************************************************************************/
int16_t Adafruit_RA8875x::applyRotationY(int16_t y) {
  switch (_rotation) {
  case 2:
    y = _height - 1 - y;
    break;
  }

  return y + _voffset;
}

/**************************************************************************/
/*!
      Draws a single pixel at the specified location

      @param x     The 0-based x location
      @param y     The 0-base y location
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875x::drawPixel(int16_t x, int16_t y, uint16_t color) {
  x = applyRotationX(x);
  y = applyRotationY(y);

  writeReg(RA8875_CURH0, x);
  writeReg(RA8875_CURH1, x >> 8);
  writeReg(RA8875_CURV0, y);
  writeReg(RA8875_CURV1, y >> 8);
  writeCommand(RA8875_MRWC);
  digitalWrite(_cs, LOW);
  SPI.transfer(RA8875_DATAWRITE);
  SPI.transfer(color >> 8);
  SPI.transfer(color);
  digitalWrite(_cs, HIGH);
}

/**************************************************************************/
/*!
 Draws a series of pixels at the specified location without the overhead

 @param p     An array of RGB565 color pixels
 @param num   The number of the pixels to draw
 @param x     The 0-based x location
 @param y     The 0-base y location
 */
/**************************************************************************/
void Adafruit_RA8875x::drawPixels(uint16_t *p, uint32_t num, int16_t x,
                                 int16_t y) {
  x = applyRotationX(x);
  y = applyRotationY(y);

  writeReg(RA8875_CURH0, x);
  writeReg(RA8875_CURH1, x >> 8);
  writeReg(RA8875_CURV0, y);
  writeReg(RA8875_CURV1, y >> 8);

  uint8_t dir = RA8875_MWCR0_LRTD;
  if (_rotation == 2) {
    dir = RA8875_MWCR0_RLTD;
  }
  writeReg(RA8875_MWCR0, (readReg(RA8875_MWCR0) & ~RA8875_MWCR0_DIRMASK) | dir);

  writeCommand(RA8875_MRWC);
  digitalWrite(_cs, LOW);
  SPI.transfer(RA8875_DATAWRITE);
  while (num--) {
    SPI.transfer16(*p++);
  }
  digitalWrite(_cs, HIGH);
}

/**************************************************************************/
/*!
      Draws a HW accelerated line on the display

      @param x0    The 0-based starting x location
      @param y0    The 0-base starting y location
      @param x1    The 0-based ending x location
      @param y1    The 0-base ending y location
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875x::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                               uint16_t color) {
  x0 = applyRotationX(x0);
  y0 = applyRotationY(y0);
  x1 = applyRotationX(x1);
  y1 = applyRotationY(y1);

  /* Set X */
  writeCommand(0x91);
  writeData(x0);
  writeCommand(0x92);
  writeData(x0 >> 8);

  /* Set Y */
  writeCommand(0x93);
  writeData(y0);
  writeCommand(0x94);
  writeData(y0 >> 8);

  /* Set X1 */
  writeCommand(0x95);
  writeData(x1);
  writeCommand(0x96);
  writeData((x1) >> 8);

  /* Set Y1 */
  writeCommand(0x97);
  writeData(y1);
  writeCommand(0x98);
  writeData((y1) >> 8);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(RA8875_DCR);
  writeData(0x80);

  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
    Draw a vertical line

    @param x The X position
    @param y The Y position
    @param h Height
    @param color The color
*/
/**************************************************************************/
void Adafruit_RA8875x::drawFastVLine(int16_t x, int16_t y, int16_t h,
                                    uint16_t color) {
  drawLine(x, y, x, y + h, color);
}

/**************************************************************************/
/*!
     Draw a horizontal line

     @param x The X position
     @param y The Y position
     @param w Width
     @param color The color
*/
/**************************************************************************/
void Adafruit_RA8875x::drawFastHLine(int16_t x, int16_t y, int16_t w,
                                    uint16_t color) {
  drawLine(x, y, x + w, y, color);
}

/**************************************************************************/
/*!
      Draws a HW accelerated rectangle on the display

      @param x     The 0-based x location of the top-right corner
      @param y     The 0-based y location of the top-right corner
      @param w     The rectangle width
      @param h     The rectangle height
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875x::drawRect(int16_t x, int16_t y, int16_t w, int16_t h,
                               uint16_t color) {
  rectHelper(x, y, x + w - 1, y + h - 1, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled rectangle on the display

      @param x     The 0-based x location of the top-right corner
      @param y     The 0-based y location of the top-right corner
      @param w     The rectangle width
      @param h     The rectangle height
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875x::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                               uint16_t color) {
  rectHelper(x, y, x + w - 1, y + h - 1, color, true);
}

/**************************************************************************/
/*!
      Fills the screen with the spefied RGB565 color

      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875x::fillScreen(uint16_t color) {
  rectHelper(0, 0, _width - 1, _height - 1, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated circle on the display

      @param x     The 0-based x location of the center of the circle
      @param y     The 0-based y location of the center of the circle
      @param r     The circle's radius
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875x::drawCircle(int16_t x, int16_t y, int16_t r,
                                 uint16_t color) {
  circleHelper(x, y, r, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled circle on the display

      @param x     The 0-based x location of the center of the circle
      @param y     The 0-based y location of the center of the circle
      @param r     The circle's radius
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875x::fillCircle(int16_t x, int16_t y, int16_t r,
                                 uint16_t color) {
  circleHelper(x, y, r, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated triangle on the display

      @param x0    The 0-based x location of point 0 on the triangle
      @param y0    The 0-based y location of point 0 on the triangle
      @param x1    The 0-based x location of point 1 on the triangle
      @param y1    The 0-based y location of point 1 on the triangle
      @param x2    The 0-based x location of point 2 on the triangle
      @param y2    The 0-based y location of point 2 on the triangle
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875x::drawTriangle(int16_t x0, int16_t y0, int16_t x1,
                                   int16_t y1, int16_t x2, int16_t y2,
                                   uint16_t color) {
  triangleHelper(x0, y0, x1, y1, x2, y2, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled triangle on the display

      @param x0    The 0-based x location of point 0 on the triangle
      @param y0    The 0-based y location of point 0 on the triangle
      @param x1    The 0-based x location of point 1 on the triangle
      @param y1    The 0-based y location of point 1 on the triangle
      @param x2    The 0-based x location of point 2 on the triangle
      @param y2    The 0-based y location of point 2 on the triangle
      @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875x::fillTriangle(int16_t x0, int16_t y0, int16_t x1,
                                   int16_t y1, int16_t x2, int16_t y2,
                                   uint16_t color) {
  triangleHelper(x0, y0, x1, y1, x2, y2, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated ellipse on the display

      @param xCenter   The 0-based x location of the ellipse's center
      @param yCenter   The 0-based y location of the ellipse's center
      @param longAxis  The size in pixels of the ellipse's long axis
      @param shortAxis The size in pixels of the ellipse's short axis
      @param color     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875x::drawEllipse(int16_t xCenter, int16_t yCenter,
                                  int16_t longAxis, int16_t shortAxis,
                                  uint16_t color) {
  ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled ellipse on the display

      @param xCenter   The 0-based x location of the ellipse's center
      @param yCenter   The 0-based y location of the ellipse's center
      @param longAxis  The size in pixels of the ellipse's long axis
      @param shortAxis The size in pixels of the ellipse's short axis
      @param color     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875x::fillEllipse(int16_t xCenter, int16_t yCenter,
                                  int16_t longAxis, int16_t shortAxis,
                                  uint16_t color) {
  ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated curve on the display

      @param xCenter   The 0-based x location of the ellipse's center
      @param yCenter   The 0-based y location of the ellipse's center
      @param longAxis  The size in pixels of the ellipse's long axis
      @param shortAxis The size in pixels of the ellipse's short axis
      @param curvePart The corner to draw, where in clock-wise motion:
                            0 = 180-270°
                            1 = 270-0°
                            2 = 0-90°
                            3 = 90-180°
      @param color     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875x::drawCurve(int16_t xCenter, int16_t yCenter,
                                int16_t longAxis, int16_t shortAxis,
                                uint8_t curvePart, uint16_t color) {
  curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled curve on the display

      @param xCenter   The 0-based x location of the ellipse's center
      @param yCenter   The 0-based y location of the ellipse's center
      @param longAxis  The size in pixels of the ellipse's long axis
      @param shortAxis The size in pixels of the ellipse's short axis
      @param curvePart The corner to draw, where in clock-wise motion:
                            0 = 180-270°
                            1 = 270-0°
                            2 = 0-90°
                            3 = 90-180°
      @param color     The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875x::fillCurve(int16_t xCenter, int16_t yCenter,
                                int16_t longAxis, int16_t shortAxis,
                                uint8_t curvePart, uint16_t color) {
  curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated rounded rectangle on the display

      @param x   The 0-based x location of the rectangle's upper left corner
      @param y   The 0-based y location of the rectangle's upper left corner
      @param w   The size in pixels of the rectangle's width
      @param h   The size in pixels of the rectangle's height
      @param r   The radius of the curves in the corners of the rectangle
      @param color  The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void Adafruit_RA8875x::drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                    int16_t r, uint16_t color) {
  roundRectHelper(x, y, x + w, y + h, r, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled rounded rectangle on the display

      @param x   The 0-based x location of the rectangle's upper left corner
      @param y   The 0-based y location of the rectangle's upper left corner
      @param w   The size in pixels of the rectangle's width
      @param h   The size in pixels of the rectangle's height
      @param r   The radius of the curves in the corners of the rectangle
      @param color  The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void Adafruit_RA8875x::fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                    int16_t r, uint16_t color) {
  roundRectHelper(x, y, x + w, y + h, r, color, true);
}

/**************************************************************************/
/*!
      Helper function for higher level circle drawing code
*/
/**************************************************************************/
void Adafruit_RA8875x::circleHelper(int16_t x, int16_t y, int16_t r,
                                   uint16_t color, bool filled) {
  x = applyRotationX(x);
  y = applyRotationY(y);

  /* Set X */
  writeCommand(0x99);
  writeData(x);
  writeCommand(0x9a);
  writeData(x >> 8);

  /* Set Y */
  writeCommand(0x9b);
  writeData(y);
  writeCommand(0x9c);
  writeData(y >> 8);

  /* Set Radius */
  writeCommand(0x9d);
  writeData(r);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(RA8875_DCR);
  if (filled) {
    writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_FILL);
  } else {
    writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_NOFILL);
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_CIRCLE_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level rectangle drawing code
*/
/**************************************************************************/
void Adafruit_RA8875x::rectHelper(int16_t x, int16_t y, int16_t w, int16_t h,
                                 uint16_t color, bool filled) {
  x = applyRotationX(x);
  y = applyRotationY(y);
  w = applyRotationX(w);
  h = applyRotationY(h);

  /* Set X */
  writeCommand(0x91);
  writeData(x);
  writeCommand(0x92);
  writeData(x >> 8);

  /* Set Y */
  writeCommand(0x93);
  writeData(y);
  writeCommand(0x94);
  writeData(y >> 8);

  /* Set X1 */
  writeCommand(0x95);
  writeData(w);
  writeCommand(0x96);
  writeData((w) >> 8);

  /* Set Y1 */
  writeCommand(0x97);
  writeData(h);
  writeCommand(0x98);
  writeData((h) >> 8);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(RA8875_DCR);
  if (filled) {
    writeData(0xB0);
  } else {
    writeData(0x90);
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level triangle drawing code
*/
/**************************************************************************/
void Adafruit_RA8875x::triangleHelper(int16_t x0, int16_t y0, int16_t x1,
                                     int16_t y1, int16_t x2, int16_t y2,
                                     uint16_t color, bool filled) {
  x0 = applyRotationX(x0);
  y0 = applyRotationY(y0);
  x1 = applyRotationX(x1);
  y1 = applyRotationY(y1);
  x2 = applyRotationX(x2);
  y2 = applyRotationY(y2);

  /* Set Point 0 */
  writeCommand(0x91);
  writeData(x0);
  writeCommand(0x92);
  writeData(x0 >> 8);
  writeCommand(0x93);
  writeData(y0);
  writeCommand(0x94);
  writeData(y0 >> 8);

  /* Set Point 1 */
  writeCommand(0x95);
  writeData(x1);
  writeCommand(0x96);
  writeData(x1 >> 8);
  writeCommand(0x97);
  writeData(y1);
  writeCommand(0x98);
  writeData(y1 >> 8);

  /* Set Point 2 */
  writeCommand(0xA9);
  writeData(x2);
  writeCommand(0xAA);
  writeData(x2 >> 8);
  writeCommand(0xAB);
  writeData(y2);
  writeCommand(0xAC);
  writeData(y2 >> 8);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(RA8875_DCR);
  if (filled) {
    writeData(0xA1);
  } else {
    writeData(0x81);
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level ellipse drawing code
*/
/**************************************************************************/
void Adafruit_RA8875x::ellipseHelper(int16_t xCenter, int16_t yCenter,
                                    int16_t longAxis, int16_t shortAxis,
                                    uint16_t color, bool filled) {
  xCenter = applyRotationX(xCenter);
  yCenter = applyRotationY(yCenter);

  /* Set Center Point */
  writeCommand(0xA5);
  writeData(xCenter);
  writeCommand(0xA6);
  writeData(xCenter >> 8);
  writeCommand(0xA7);
  writeData(yCenter);
  writeCommand(0xA8);
  writeData(yCenter >> 8);

  /* Set Long and Short Axis */
  writeCommand(0xA1);
  writeData(longAxis);
  writeCommand(0xA2);
  writeData(longAxis >> 8);
  writeCommand(0xA3);
  writeData(shortAxis);
  writeCommand(0xA4);
  writeData(shortAxis >> 8);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(0xA0);
  if (filled) {
    writeData(0xC0);
  } else {
    writeData(0x80);
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level curve drawing code
*/
/**************************************************************************/
void Adafruit_RA8875x::curveHelper(int16_t xCenter, int16_t yCenter,
                                  int16_t longAxis, int16_t shortAxis,
                                  uint8_t curvePart, uint16_t color,
                                  bool filled) {
  xCenter = applyRotationX(xCenter);
  yCenter = applyRotationY(yCenter);
  curvePart = (curvePart + _rotation) % 4;

  /* Set Center Point */
  writeCommand(0xA5);
  writeData(xCenter);
  writeCommand(0xA6);
  writeData(xCenter >> 8);
  writeCommand(0xA7);
  writeData(yCenter);
  writeCommand(0xA8);
  writeData(yCenter >> 8);

  /* Set Long and Short Axis */
  writeCommand(0xA1);
  writeData(longAxis);
  writeCommand(0xA2);
  writeData(longAxis >> 8);
  writeCommand(0xA3);
  writeData(shortAxis);
  writeCommand(0xA4);
  writeData(shortAxis >> 8);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(0xA0);
  if (filled) {
    writeData(0xD0 | (curvePart & 0x03));
  } else {
    writeData(0x90 | (curvePart & 0x03));
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level rounded rectangle drawing code
 */
/**************************************************************************/
void Adafruit_RA8875x::roundRectHelper(int16_t x, int16_t y, int16_t w,
                                      int16_t h, int16_t r, uint16_t color,
                                      bool filled) {
  x = applyRotationX(x);
  y = applyRotationY(y);
  w = applyRotationX(w);
  h = applyRotationY(h);
  if (x > w)
    swap(x, w);
  if (y > h)
    swap(y, h);

  /* Set X */
  writeCommand(0x91);
  writeData(x);
  writeCommand(0x92);
  writeData(x >> 8);

  /* Set Y */
  writeCommand(0x93);
  writeData(y);
  writeCommand(0x94);
  writeData(y >> 8);

  /* Set X1 */
  writeCommand(0x95);
  writeData(w);
  writeCommand(0x96);
  writeData((w) >> 8);

  /* Set Y1 */
  writeCommand(0x97);
  writeData(h);
  writeCommand(0x98);
  writeData((h) >> 8);

  writeCommand(0xA1);
  writeData(r);
  writeCommand(0xA2);
  writeData((r) >> 8);

  writeCommand(0xA3);
  writeData(r);
  writeCommand(0xA4);
  writeData((r) >> 8);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(RA8875_ELLIPSE);
  if (filled) {
    writeData(0xE0);
  } else {
    writeData(0xA0);
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_ELLIPSE, RA8875_DCR_LINESQUTRI_STATUS);
}
/**************************************************************************/
/*!
      Set the scroll window

      @param x  X position of the scroll window
      @param y  Y position of the scroll window
      @param w  Width of the Scroll Window
      @param h  Height of the Scroll window
      @param mode Layer to Scroll

 */
/**************************************************************************/
void Adafruit_RA8875x::setScrollWindow(int16_t x, int16_t y, int16_t w,
                                      int16_t h, uint8_t mode) {
  // Horizontal Start point of Scroll Window
  writeCommand(0x38);
  writeData(x);
  writeCommand(0x39);
  writeData(x >> 8);

  // Vertical Start Point of Scroll Window
  writeCommand(0x3a);
  writeData(y);
  writeCommand(0x3b);
  writeData(y >> 8);

  // Horizontal End Point of Scroll Window
  writeCommand(0x3c);
  writeData(x + w);
  writeCommand(0x3d);
  writeData((x + w) >> 8);

  // Vertical End Point of Scroll Window
  writeCommand(0x3e);
  writeData(y + h);
  writeCommand(0x3f);
  writeData((y + h) >> 8);

  // Scroll function setting
  writeCommand(0x52);
  writeData(mode);
}

/**************************************************************************/
/*!
    Scroll in the X direction

    @param dist The distance to scroll

 */
/**************************************************************************/
void Adafruit_RA8875x::scrollX(int16_t dist) {
  writeCommand(0x24);
  writeData(dist);
  writeCommand(0x25);
  writeData(dist >> 8);
}

/**************************************************************************/
/*!
     Scroll in the Y direction

     @param dist The distance to scroll

 */
/**************************************************************************/
void Adafruit_RA8875x::scrollY(int16_t dist) {
  writeCommand(0x26);
  writeData(dist);
  writeCommand(0x27);
  writeData(dist >> 8);
}

/************************* Mid Level ***********************************/

/**************************************************************************/
/*!
    Set the Extra General Purpose IO Register

    @param on Whether to turn Extra General Purpose IO on or not

 */
/**************************************************************************/
void Adafruit_RA8875x::GPIOX(boolean on) {
  if (on)
    writeReg(RA8875_GPIOX, 1);
  else
    writeReg(RA8875_GPIOX, 0);
}

/**************************************************************************/
/*!
    Set the duty cycle of the PWM 1 Clock

    @param p The duty Cycle (0-255)
*/
/**************************************************************************/
void Adafruit_RA8875x::PWM1out(uint8_t p) { writeReg(RA8875_P1DCR, p); }

/**************************************************************************/
/*!
     Set the duty cycle of the PWM 2 Clock

     @param p The duty Cycle (0-255)
*/
/**************************************************************************/
void Adafruit_RA8875x::PWM2out(uint8_t p) { writeReg(RA8875_P2DCR, p); }

/**************************************************************************/
/*!
    Configure the PWM 1 Clock

    @param on Whether to enable the clock
    @param clock The Clock Divider
*/
/**************************************************************************/
void Adafruit_RA8875x::PWM1config(boolean on, uint8_t clock) {
  if (on) {
    writeReg(RA8875_P1CR, RA8875_P1CR_ENABLE | (clock & 0xF));
  } else {
    writeReg(RA8875_P1CR, RA8875_P1CR_DISABLE | (clock & 0xF));
  }
}

/**************************************************************************/
/*!
     Configure the PWM 2 Clock

     @param on Whether to enable the clock
     @param clock The Clock Divider
*/
/**************************************************************************/
void Adafruit_RA8875x::PWM2config(boolean on, uint8_t clock) {
  if (on) {
    writeReg(RA8875_P2CR, RA8875_P2CR_ENABLE | (clock & 0xF));
  } else {
    writeReg(RA8875_P2CR, RA8875_P2CR_DISABLE | (clock & 0xF));
  }
}

/**************************************************************************/
/*!
      Enables or disables the on-chip touch screen controller

      @param on Whether to turn touch sensing on or not
*/
/**************************************************************************/
void Adafruit_RA8875x::touchEnable(boolean on) {
  uint8_t adcClk = (uint8_t)RA8875_TPCR0_ADCCLK_DIV4;

  if (_size == RA8875_800x480) // match up touch size with LCD size
    adcClk = (uint8_t)RA8875_TPCR0_ADCCLK_DIV16;

  if (on) {
    /* Enable Touch Panel (Reg 0x70) */
    writeReg(RA8875_TPCR0, RA8875_TPCR0_ENABLE | RA8875_TPCR0_WAIT_4096CLK |
                               RA8875_TPCR0_WAKEENABLE | adcClk); // 10mhz max!
    /* Set Auto Mode      (Reg 0x71) */
    writeReg(RA8875_TPCR1, RA8875_TPCR1_AUTO |
                               // RA8875_TPCR1_VREFEXT |
                               RA8875_TPCR1_DEBOUNCE);
    /* Enable TP INT */
    writeReg(RA8875_INTC1, readReg(RA8875_INTC1) | RA8875_INTC1_TP);
  } else {
    /* Disable TP INT */
    writeReg(RA8875_INTC1, readReg(RA8875_INTC1) & ~RA8875_INTC1_TP);
    /* Disable Touch Panel (Reg 0x70) */
    writeReg(RA8875_TPCR0, RA8875_TPCR0_DISABLE);
  }
}

/**************************************************************************/
/*!
      Checks if a touch event has occured

      @return  True is a touch event has occured (reading it via
               touchRead() will clear the interrupt in memory)
*/
/**************************************************************************/
boolean Adafruit_RA8875x::touched(void) {
  if (readReg(RA8875_INTC2) & RA8875_INTC2_TP)
    return true;
  return false;
}

/**************************************************************************/
/*!
      Reads the last touch event

      @param x  Pointer to the uint16_t field to assign the raw X value
      @param y  Pointer to the uint16_t field to assign the raw Y value

      @return True if successful

      @note Calling this function will clear the touch panel interrupt on
            the RA8875, resetting the flag used by the 'touched' function
*/
/**************************************************************************/
boolean Adafruit_RA8875x::touchRead(uint16_t *x, uint16_t *y) {
  uint16_t tx, ty;
  uint8_t temp;

  tx = readReg(RA8875_TPXH);
  ty = readReg(RA8875_TPYH);
  temp = readReg(RA8875_TPXYL);
  tx <<= 2;
  ty <<= 2;
  tx |= temp & 0x03;        // get the bottom x bits
  ty |= (temp >> 2) & 0x03; // get the bottom y bits

  *x = tx;
  *y = ty;

  /* Clear TP INT Status */
  writeReg(RA8875_INTC2, RA8875_INTC2_TP);

  return true;
}

/**************************************************************************/
/*!
      Turns the display on or off

      @param on Whether to turn the display on or not
*/
/**************************************************************************/
void Adafruit_RA8875x::displayOn(boolean on) {
  if (on)
    writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);
  else
    writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
}

/**************************************************************************/
/*!
    Puts the display in sleep mode, or disables sleep mode if enabled

    @param sleep Whether to sleep or not
*/
/**************************************************************************/
void Adafruit_RA8875x::sleep(boolean sleep) {
  if (sleep)
    writeReg(RA8875_PWRR, RA8875_PWRR_DISPOFF | RA8875_PWRR_SLEEP);
  else
    writeReg(RA8875_PWRR, RA8875_PWRR_DISPOFF);
}

/************************* Low Level ***********************************/

/**************************************************************************/
/*!
    Write data to the specified register

    @param reg Register to write to
    @param val Value to write
*/
/**************************************************************************/
void Adafruit_RA8875x::writeReg(uint8_t reg, uint8_t val) {
  writeCommand(reg);
  writeData(val);
}

/**************************************************************************/
/*!
    Set the register to read from

    @param reg Register to read

    @return The value
*/
/**************************************************************************/
uint8_t Adafruit_RA8875x::readReg(uint8_t reg) {
  writeCommand(reg);
  return readData();
}

/**************************************************************************/
/*!
    Write data to the current register

    @param d Data to write
*/
/**************************************************************************/
void Adafruit_RA8875x::writeData(uint8_t d) {
  digitalWrite(_cs, LOW);
  spi_begin();
  SPI.transfer(RA8875_DATAWRITE);
  SPI.transfer(d);
  spi_end();
  digitalWrite(_cs, HIGH);
}

/**************************************************************************/
/*!
    Read the data from the current register

    @return The Value
*/
/**************************************************************************/
uint8_t Adafruit_RA8875x::readData(void) {
  digitalWrite(_cs, LOW);
  spi_begin();

  SPI.transfer(RA8875_DATAREAD);
  uint8_t x = SPI.transfer(0x0);
  spi_end();

  digitalWrite(_cs, HIGH);
  return x;
}

/**************************************************************************/
/*!
    Write a command to the current register

    @param d The data to write as a command
 */
/**************************************************************************/
void Adafruit_RA8875x::writeCommand(uint8_t d) {
  digitalWrite(_cs, LOW);
  spi_begin();

  SPI.transfer(RA8875_CMDWRITE);
  SPI.transfer(d);
  spi_end();

  digitalWrite(_cs, HIGH);
}

/**************************************************************************/
/*!
    Read the status from the current register

    @return The value
 */
/**************************************************************************/
uint8_t Adafruit_RA8875x::readStatus(void) {
  digitalWrite(_cs, LOW);
  spi_begin();
  SPI.transfer(RA8875_CMDREAD);
  uint8_t x = SPI.transfer(0x0);
  spi_end();

  digitalWrite(_cs, HIGH);
  return x;
}

/// @cond DISABLE
#if defined(EEPROM_SUPPORTED)
/// @endcond
/**************************************************************************/
/*!
    Read from the EEPROM location

    @param location The location of the EEPROM to read

    @return The value
*/
/**************************************************************************/

uint32_t Adafruit_RA8875x::eepromReadS32(int location) {
  uint32_t value = ((uint32_t)EEPROM.read(location)) << 24;
  value = value | ((uint32_t)EEPROM.read(location + 1)) << 16;
  value = value | ((uint32_t)EEPROM.read(location + 2)) << 8;
  value = value | ((uint32_t)EEPROM.read(location + 3));
  return value;
}

/**************************************************************************/
/*!
    Write to the EEPROM location

    @param location The location of the EEPROM to write to
    @param value The value to write
 */
/**************************************************************************/
void Adafruit_RA8875x::eepromWriteS32(int location, int32_t value) {
  EEPROM.write(location, (value >> 24) & 0xff);
  EEPROM.write(location + 1, (value >> 16) & 0xff);
  EEPROM.write(location + 2, (value >> 8) & 0xff);
  EEPROM.write(location + 3, (value)&0xff);
}

/**************************************************************************/
/*!
     Read Calibration Data from the EEPROM location

     @param location The location of the EEPROM to read from
     @param matrixPtr The pointer to the Matrix Variable

     @return success
 */
/**************************************************************************/
bool Adafruit_RA8875x::readCalibration(int location, tsMatrix_t *matrixPtr) {
  if (location + sizeof(tsMatrix_t) > EEPROMSIZE) {
    return false; // readCalibration::Calibration location outside of EEPROM
                  // memory bound
  }
  if (EEPROM.read(location + CFG_EEPROM_TOUCHSCREEN_CALIBRATED) == 1) {
    matrixPtr->An = eepromReadS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_AN);
    matrixPtr->Bn = eepromReadS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_BN);
    matrixPtr->Cn = eepromReadS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_CN);
    matrixPtr->Dn = eepromReadS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_DN);
    matrixPtr->En = eepromReadS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_EN);
    matrixPtr->Fn = eepromReadS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_FN);
    matrixPtr->Divider =
        eepromReadS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_DIVIDER);
    return true;
  }
  return false;
}

/**************************************************************************/
/*!
     Write Calibration Data to the EEPROM location

     @param location The location of the EEPROM to write to
     @param matrixPtr The pointer to the Matrix Variable
 */
/**************************************************************************/
void Adafruit_RA8875x::writeCalibration(int location, tsMatrix_t *matrixPtr) {
  if (location + sizeof(tsMatrix_t) <
      EEPROMSIZE) { // Check to see it calibration location outside of EEPROM
                    // memory bound
    eepromWriteS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_AN, matrixPtr->An);
    eepromWriteS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_BN, matrixPtr->Bn);
    eepromWriteS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_CN, matrixPtr->Cn);
    eepromWriteS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_DN, matrixPtr->Dn);
    eepromWriteS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_EN, matrixPtr->En);
    eepromWriteS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_FN, matrixPtr->Fn);
    eepromWriteS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_DIVIDER,
                   matrixPtr->Divider);
    EEPROM.write(location + CFG_EEPROM_TOUCHSCREEN_CALIBRATED, 1);
  }
}
/// @cond DISABLE
#endif
/// @endcond


//////////////////////////////////////////////////
// Enhancements

void Adafruit_RA8875x::targetCGRAM()
{
	uint8_t mwcr1 = this->readReg(RA8875_MWCR1);
	uint8_t fncr0 = this->readReg(RA8875_FNCR0);
	
	fncr0 = bitClear(fncr0, 7);
	
	mwcr1 = bitClear(mwcr1, 3);
	mwcr1 = bitSet(mwcr1, 2);
	
	this->writeReg(RA8875_FNCR0, fncr0);
	this->writeReg(RA8875_MWCR1, mwcr1);
	// uint8_t mwcr1 = this->readReg(RA8875_MWCR1);
	//
	// mwcr1 = bitClear(mwcr1, 3);
	// mwcr1 = bitSet(mwcr1, 2);
	//
	// uint8_t fontReg = this->readReg(RA8875_FNCR0);
	//
	// if(bitRead(fontReg,7)) {
	// 	fontReg = bitClear(fontReg, 7);
	//
	// 	this->writeReg(RA8875_FNCR0, fontReg);
	// 	this->writeReg(RA8875_MWCR1, mwcr1);
	// } else {
	// 	this->writeData(mwcr1);
	// }
}



void Adafruit_RA8875x::textSetFont(Adafruit_RA8875x::FontType type, Adafruit_RA8875x::FontSet set)
{
	uint8_t fncr0 = this->readReg(RA8875_FNCR0);
	uint8_t mwcr1 = this->readReg(RA8875_MWCR1);
	
	if(type == Adafruit_RA8875x::UserFont) {
		fncr0 = bitSet(fncr0, 7);
		fncr0 = bitSet(fncr0, 5);
	} else if(type == Adafruit_RA8875x::InternalFont) {
		fncr0 = bitClear(fncr0, 7);
		fncr0 = bitClear(fncr0, 5);
	} else if(type == Adafruit_RA8875x::ExternalFont) {
		fncr0 = bitClear(fncr0, 7);
		fncr0 = bitSet(fncr0, 5);
	}
	
	fncr0 &= 0b11111100;
	fncr0 |= set;
	
	this->writeReg(RA8875_FNCR0, fncr0);
	
	mwcr1 = bitClear(mwcr1, 3);
	mwcr1 = bitClear(mwcr1, 2);
	
	this->writeReg(RA8875_MWCR1, mwcr1);
}




void Adafruit_RA8875x::textSetUserChar(uint8_t charIdx, uint8_t charData[16])
{
	uint8_t mwcr1 = this->readReg(RA8875_MWCR1);

	this->graphicsMode();

	this->writeReg(RA8875_CGSR, charIdx);

	this->targetCGRAM();

	this->writeCommand(RA8875_MRWC);

	for (uint8_t i=0;i<16;i++){
		this->writeData(charData[i]);
	}

	this->writeReg(RA8875_MWCR1, mwcr1);//restore register
}


void Adafruit_RA8875x::textSetUserFont(uint8_t data[][16], uint8_t firstChar, uint8_t lastChar)
{
	for(int charNo = firstChar; charNo <= lastChar; ++charNo){
		this->textSetUserChar(charNo, data[charNo]);
	}
}


void Adafruit_RA8875x::setWindow(uint16_t fromX, uint16_t fromY, uint16_t toX, uint16_t toY)
{
	uint8_t hsaw0 = fromX & 0xFF;
	uint8_t hsaw1 = (fromX >> 8) & 0xFF;
	uint8_t vsaw0 = fromY & 0xFF;;
	uint8_t vsaw1 = (fromY >> 8) & 0xFF;;
	uint8_t heaw0 = toX & 0xFF;;
	uint8_t heaw1 = (toX >> 8) & 0xFF;;
	uint8_t veaw0 = toY & 0xFF;;
	uint8_t veaw1 = (toY >> 8) & 0xFF;;

	this->writeReg(RA8875_HSAW0, hsaw0);
	this->writeReg(RA8875_HSAW1, hsaw1);
	this->writeReg(RA8875_VSAW0, vsaw0);
	this->writeReg(RA8875_VSAW1, vsaw1);
	this->writeReg(RA8875_HEAW0, heaw0);
	this->writeReg(RA8875_HEAW1, heaw1);
	this->writeReg(RA8875_VEAW0, veaw0);
	this->writeReg(RA8875_VEAW1, veaw1);
	
}


void Adafruit_RA8875x::getWindow(uint16_t *fromX, uint16_t *fromY, uint16_t *toX, uint16_t *toY)
{
	 uint8_t hsaw0 = this->readReg(RA8875_HSAW0);
	 uint8_t hsaw1 = this->readReg(RA8875_HSAW1);
	 uint8_t vsaw0 = this->readReg(RA8875_VSAW0);
	 uint8_t vsaw1 = this->readReg(RA8875_VSAW1);
	 uint8_t heaw0 = this->readReg(RA8875_HEAW0);
	 uint8_t heaw1 = this->readReg(RA8875_HEAW1);
	 uint8_t veaw0 = this->readReg(RA8875_VEAW0);
	 uint8_t veaw1 = this->readReg(RA8875_VEAW1);
	 
	 *fromX = (hsaw1 << 8) | hsaw0;
	 *fromY = (vsaw1 << 8) | vsaw0;

	 *toX = (heaw1 << 8) | heaw0;
	 *toY = (veaw1 << 8) | veaw0;
}


void Adafruit_RA8875x::pushPixels(uint32_t num, uint16_t p[]) {
	uint32_t i = 0;
	
	writeReg(RA8875_CURH0, 0);
	writeReg(RA8875_CURH1, 0);
	writeReg(RA8875_CURV0, 0);
	writeReg(RA8875_CURV1, 0);

	writeCommand(RA8875_MRWC);
	
	digitalWrite(_cs, LOW);
	
	SPI.transfer(RA8875_DATAWRITE);
	spi_begin();
	while (num--) {
		SPI.transfer(p[i] >> 8);
		SPI.transfer(p[i]);
		++i;
	}
	spi_end();
	
	digitalWrite(_cs, HIGH);
}


void Adafruit_RA8875x::drawCanvas1(uint16_t x, uint16_t y, uint16_t fg, uint16_t bg, GFXcanvas1 *canvas) {
	uint16_t r;
	uint16_t g;
	uint16_t b;
	
	uint8_t becr0;
	uint8_t becr1;
	
	uint16_t   w = canvas->width();
	uint16_t   h = canvas->height();
	uint8_t  *bmp = canvas->getBuffer();
	uint16_t  w2 = (w + 7) / 8;
	uint16_t  len = w2 * h;
	
	// The numbered comments are taken from The datasheet "7-6-4-9 Color Expansion"
	// 1. Setting destination position           --> HDBE0/1, VDBE0/1
	this->writeReg(RA8875_HDBE0, x & 0xFF);
	this->writeReg(RA8875_HDBE1, x >> 8);
	this->writeReg(RA8875_VDBE0, y & 0xFF);
	this->writeReg(RA8875_VDBE1, y >> 8);

	// 2. Setting BTE width register             --> BEWR0/1
	this->writeReg(RA8875_BEWR0, w & 0xFF);
	this->writeReg(RA8875_BEWR1, w >> 8);
	
	// 3. Setting BTE height register            --> BEHR0/1
	this->writeReg(RA8875_BEHR0, h & 0xFF);
	this->writeReg(RA8875_BEHR1, h >> 8);
	
	// 4. Setting Background Color               --> BGCR0[bits 4-0], BGCR1[bits 5-0], BGCR2[bits 4-0]
	//        rrrrrggggggbbbbb
	r = (bg & 0b1111100000000000) >> 11;
	g = (bg & 0b0000011111100000) >> 5;
	b = (bg & 0b0000000000011111) >> 0;
	this->writeReg(RA8875_BGCR0, r);
	this->writeReg(RA8875_BGCR1, g);
	this->writeReg(RA8875_BGCR2, b);
	
	// 5. Setting Foreground Color               --> FGCR0[bits 4-0], FGCR1[bits 5-0], FGCR2[bits 4-0]
	//        rrrrrggggggbbbbb
	r = (fg & 0b1111100000000000) >> 11;
	g = (fg & 0b0000011111100000) >> 5;
	b = (fg & 0b0000000000011111) >> 0;
	this->writeReg(RA8875_FGCR0, r);
	this->writeReg(RA8875_FGCR1, g);
	this->writeReg(RA8875_FGCR2, b);
	
	// 6. Setting BTE operation and ROP function --> BECR1[bits 3-0] = 08h
	becr1 = this->readReg(RA8875_BECR1);
	becr1 &= 0xFFF0;
	becr1 |= 0x0008;
	this->writeReg(RA8875_BECR1, becr1);
	
	// 7. Enable BTE function                    --> BECR0[bit 7] = 1
	becr0 = this->readReg(RA8875_BECR0);
	bitSet(becr0, 7);
	this->writeReg(RA8875_BECR0, becr0);
	
	// 8, 9, 10. Write Image Data
	writeCommand(RA8875_MRWC);
	
	digitalWrite(_cs, LOW);
	
	SPI.transfer(RA8875_DATAWRITE);

	// For reasons I can't figure out, we need to write a padding 
	// character at the end of each line, UNLESS the width of the 
	// line modulus 8 is 1.
	//
	// Neither the need for the padding character, nor the mod 8 
	// exception is documented anywhere in the datasheet that I can
	// find.
	bool pad = w % 8 != 1;
	
	for(int y = 0; y < len / w2; ++y) {
		for(int16_t x = 0; x < w2; ++x) {
			SPI.transfer(*(bmp++));				
		}
		
		if(pad) {
			SPI.transfer(0x00);
		}
	}
	
	digitalWrite(_cs, HIGH);
	
}
