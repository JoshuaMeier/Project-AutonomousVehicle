/*
 * From the rising of the sun to its setting, the name of the LORD is to be praised!
 * Psalms 113:3, ESV
 */

#include "fsl_common.h"
#include "fsl_slcd.h"


/*  Segment names: clockwise, from the top, A, B, .., F, with G in the middle

	MCU pins connected to back plane pins:

  	1 LCD_P59 (COM0) will use phase A (1)
	2 LCD_P60 (COM1) will use phase B (2)
	3 LCD_P14 (COM2) will use phase C (4)
	4 LCD_P15 (COM3) will use phase D (8)

	MCU pins connected to front plane pins:

	5 LCD_P20 (1D/1E/1G/1F)
	6 LCD_P24 (DP1/1C/1B/1A)
	7 LCD_P26 (2D/2E/2G/2F)
	8 LCD_P27 (DP2/2C/2B/2A)
	9 LCD_P40 (3D/3E/3G/3F)
	10 LCD_P42 (DP3/3C/3B/3A)
	11 LCD_P43 (4D/4E/4G/4F)
	12 LCD_P44 (COL/4C/4B/4A)

The segment representation of a character uses one byte as follows:
bit:	7 6 5 4 3 2 1 0
		a b c d e f g dot
 */

// define the front plane MCU pins
static const uint8_t pin[] = {20, 24, 26, 27, 40, 42, 43, 44};

// define the back plane MCU pins
static const uint8_t bpin[] = {59, 60, 14, 15};

// The following are global variables in case they are needed
// beyond the context of the initialization function.
static slcd_config_t config;
static slcd_clock_config_t clkConfig =
{
    kSLCD_AlternateClk1,
    kSLCD_AltClkDivFactor256,
    kSLCD_ClkPrescaler01
#if FSL_FEATURE_SLCD_HAS_FAST_FRAME_RATE
    ,
    false
#endif
};

/*!
 * @brief Initializes the LCD. If using LCDSetSegments and Print2LCD,
 * it does not have to be called in advance, since these functions will
 * call it anyways.
 */
void InitializeLCD() {
	static bool ready = 0;
	if(ready) return;

	int i;
	uint32_t low = 0, high = 0, blow = 0, bhigh = 0;

	for(i = 0; i < 4; i++) {
		if(bpin[i] < 32)
			blow |= 1<<bpin[i];
		else
			bhigh |= 1<<(bpin[i] - 32);
	}
	for(i = 0; i < 8; i++) {
		if(pin[i] < 32)
			low |= 1<<pin[i];
		else
			high |= 1<<(pin[i] - 32);
	}

	// The following is based on the initialization code of the slcd.c example

    MCG->C1 |= MCG_C1_IRCLKEN_MASK; // Enable MCGIRCLK
    SLCD_GetDefaultConfig(&config);
    config.clkConfig = &clkConfig;
    config.loadAdjust = kSLCD_HighLoadOrSlowestClkSrc;
    config.dutyCycle = kSLCD_1Div4DutyCycle;
    config.slcdLowPinEnabled = low | blow;  /* LCD_P27/26/24/20 -> b27/26/24/20 = 1. */
    config.slcdHighPinEnabled = high | bhigh; /* LCD_P44/43/42/40 -> b12/11/10/8 = 1. */
    config.backPlaneLowPin = blow;    /* LCD_P15/P14 -> b15/b14 = 1. */
    config.backPlaneHighPin = bhigh;   /* LCD_P60/P59 -> b28/27 = 1. */
    config.faultConfig = NULL;
    SLCD_Init(LCD, &config);

    SLCD_SetBackPlanePhase(LCD, bpin[0], kSLCD_PhaseAActivate); /* SLCD COM1 --- LCD_P59. */
    SLCD_SetBackPlanePhase(LCD, bpin[1], kSLCD_PhaseBActivate); /* SLCD COM2 --- LCD_P60. */
    SLCD_SetBackPlanePhase(LCD, bpin[2], kSLCD_PhaseCActivate); /* SLCD COM3 --- LCD_P14. */
    SLCD_SetBackPlanePhase(LCD, bpin[3], kSLCD_PhaseDActivate); /* SLCD COM4 --- LCD_P15. */

    ready = 1;
}


// segments represented by the bits 7,6, ..., 0 in this order abcdefg dot
// list the numbers 0, 1, ..., 9
static const uint8_t num[] = {	0b11111100, //0
								0b01100000, //1
								0b11011010, //2
								0b11110010, //3
								0b01100110, //4
								0b10110110, //5
								0b10111110, //6
								0b11100000, //7
								0b11111110, //8
								0b11110110, //9
};
static const uint8_t abc[] = {	0b11101110, //A
								0b00111110, //b
								0b00011010, //c
								0b01111010, //d
								0b10011110, //E
								0b10001110, //F
};
static const uint8_t ABC[] = {	0b11101110, //A
								0b00111110, //b
								0b10011100, //C
								0b01111010, //d
								0b10011110, //E
								0b10001110, //F
};


/*!
 * @brief Displays the specified characters on the LCD.
 *
 * Characters are specified in terms of the segments of the display.
 * The format is:
 * bit:		7 6 5 4 3 2 1 0
 *			a b c d e f g dot/colon
 * Note that the bit 0 of the 4th character is used for the colon.
 * @code
 *   uint8_t chars[] = {0b01100000, 0b11110011, 0b11111110, 0b01100110};
 *   LCDSetSegments(chars); // displays 13.84
 *   uint8_t chars2[] = {0b00000000, 0b11110010, 0b01100000, 0b10111111};
 *   LCDSetSegments(chars2); // displays 3:16
 * @endcode
 *
 * @param chars     An array of four characters.
 */
void LCDSetSegments(uint8_t *chars) {
	int j, k;
	uint8_t wave1, wave2;

	InitializeLCD(); // does nothing if already initialized
	for(k = 0; k < 4; k++) { // 4 is half of the number of LCD front plane pins
		// Each iteration writes one of the four digits
		wave1 = 0; wave2 = 0;
		j = chars[k];
		if(j & 1<<4)
			wave1 |= kSLCD_PhaseAActivate;
		if(j & 1<<3)
			wave1 |= kSLCD_PhaseBActivate;
		if(j & 2)
			wave1 |= kSLCD_PhaseCActivate;
		if(j & 4)
			wave1 |= kSLCD_PhaseDActivate;
		if(j & 1)
			wave2 |= kSLCD_PhaseAActivate;
		if(j & 1<<5)
			wave2 |= kSLCD_PhaseBActivate;
		if(j & 1<<6)
			wave2 |= kSLCD_PhaseCActivate;
		if(j & 1<<7)
			wave2 |= kSLCD_PhaseDActivate;
	    SLCD_SetFrontPlaneSegments(LCD, pin[2*k], wave1);
	    SLCD_SetFrontPlaneSegments(LCD, pin[2*k+1], wave2);
	}
    SLCD_StartDisplay(LCD);
}


/*!
 * @brief Displays text from left to right on the LCD.
 *
 * The text must end with a zero character. Undefined characters
 * are displayed as '-'. If there are no dots and no middle colon,
 * only the first 4 characters will be displayed.
 * @code
 *  Print2LCD("123");
 *  Print2LCD(" 1 .3");
 *  Print2LCD("12:07");
 *  Print2LCD("-.234");
 * @endcode
 *
 * @param str     A zero terminated array of characters.
 */
void Print2LCD(char *str) {
	uint8_t chars[4];
	uint16_t i, charNum = 0;

	for(i = 0; i < 4; i++) {
		chars[i] = 0;
	}
	for(i = 0; str[i] && charNum < 4; i++) {
		if('0' <= str[i] && str[i] <= '9')
			chars[charNum++] |= num[str[i]-'0'];
		else if('a' <= str[i] && str[i] <= 'f')
			chars[charNum++] |= abc[str[i]-'a'];
		else if('A' <= str[i] && str[i] <= 'F')
			chars[charNum++] |= ABC[str[i]-'A'];
		else if(str[i] == '.') {
			if(!charNum) {
				chars[0] |= 1;
				charNum++;
			}
			else
				chars[charNum-1] |= 1;
		}
		else if(str[i] == ' ')
			chars[charNum++] &= 1;
		else if(str[i] == ':' && charNum == 2)
			chars[3] |= 1;
		else  // display '-'
			chars[charNum++] |= 0b010;
	}
	LCDSetSegments(chars);
}

