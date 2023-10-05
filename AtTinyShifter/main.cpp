/**
 * \file AtTinyShifter.cpp
 * \brief Using the AtTiny, reads an ADC reading and sends it out onto shift registers
 * \author Tim Robbins
 */

//Headers, Macros, and Definitions-----------------------------------------------

#define F_CPU					1000000UL

#include <avr/io.h>
#include <util/delay.h>

///Shift register definitions
#define SHIFT_REG_OUT_REG		PORTB
#define SHIFT_REG_DIR_REG		DDRB
#define SHIFT_REG_LATCH			3
#define SHIFT_REG_DATA			0
#define SHIFT_REG_CLK			4

//Optional out enable
#define SHIFT_REG_OUT_EN		1

///The pin connected to the ADC reading pin
#define ADC_A_PIN				2

///The channel connected to the adc reading pin
#define ADC_A					1

///The pin mask for the ADC
#define ADC_PIN_MASK			(1 << ADC_A_PIN)

///The pin used as a select pin
#define SELECT_PIN				1

///The amount of times to sample the ADC readings
#define ADC_SAMPLES				10


//-------------------------------------------------------------------------------


//Data types---------------------------------------------------------------------

//-------------------------------------------------------------------------------


//Variables----------------------------------------------------------------------

///Lowest ADC reading
static uint16_t adcLowest = 0xffff;

///largest ADC reading
static uint16_t adcHighest = 1;

///The cutout point for an ADC reading to be considered high or low
static uint16_t adcCutout = 0;

//-------------------------------------------------------------------------------


//Functions----------------------------------------------------------------------
void SystemInit();
void AdcSetup(uint8_t pinMask, bool disableDigital);
void ShiftOut(uint8_t valueOut, volatile uint8_t *dataOutputRegister, uint8_t dataPin, volatile uint8_t *clockPinOutRegister, uint8_t clockPin, bool LSBFirst);
uint16_t SampleAdc(uint8_t adcChannel, uint8_t sampleCount);
void AdcToByteShift();
void AdcToBitShift();
bool SampleNext();
unsigned char SampleByteOverMicro(uint16_t adcForActive, uint16_t microDelayPerReading, bool msbFirst);
void AdcToShiftout();
//-------------------------------------------------------------------------------


//Interrupts---------------------------------------------------------------------

//-------------------------------------------------------------------------------



/**
* \brief Entry point to program
*
*/
int main(void)
{
	//Initialize the system
	SystemInit();
	
	//Enter program loop...
	while (1)
	{
		//Run the current modes function
		AdcToByteShift();
	}
}



/**
* \brief Initializes all controller systems and peripherals
*
*/
void SystemInit()
{
	//Initialize pins and directions
	PORTB = 0;
	DDRB = 0xff;
	
	//Initialize ADC
	AdcSetup(ADC_PIN_MASK, true);
}



/**
* \brief Initializes ADC
*/
void AdcSetup(uint8_t pinMask, bool disableDigital)
{
	//Set as input low, internal pull up disabled
	PORTB &= ~(pinMask);
	DDRB &= ~(pinMask);
	
	//If we've been asked to disable the digital functionalities on the pin mask...
	if(disableDigital)
	{
		//Disable digital input
		DIDR0 |= (pinMask);
	}
	
	//Set admux mode to REF mode 1
	ADMUX = 1 << REFS1;
	
	//Set for free running mode
	ADCSRB &= ~(1 << ADTS0 | 1 << ADTS1 | 1 << ADTS2);
	
	//Turn on ADC
	ADCSRA |= (1 << ADEN);
}



/**
* \brief Shifts out output data to a connected shift register
*
* \param valueOut The value to shift out
* \param dataOutputRegister The output register connected to the data pin
* \param dataPin the pin connected to the shift registers data pin
* \param clockPinOutRegister The output register for the clock pin
* \param clockPin The pin the clock pin is connected to
* \param LSBFirst If we're shifting in the least significant bit first
*/
void ShiftOut(uint8_t valueOut, volatile uint8_t *dataOutputRegister, uint8_t dataPin, volatile uint8_t *clockPinOutRegister, uint8_t clockPin, bool LSBFirst)
{
	uint8_t i = 0;
	
	//If we're running LSB first(avoiding using an if statement every loop)...
	if(LSBFirst == true)
	{
		for (i = 0; i < 8; i++)
		{
			if(valueOut & (1 << (i)))
			{
				*dataOutputRegister |= (1 << dataPin);
			}
			else
			{
				*dataOutputRegister &= ~(1 << dataPin);
			}
			
			*clockPinOutRegister |= (1 << clockPin);
			*clockPinOutRegister &= ~(1 << clockPin);
		}
	}
	//else...
	else
	{
		
		for (i = 0; i < 8; i++)
		{
			if(valueOut & (1 << (7-i)))
			{
				*dataOutputRegister |= (1 << dataPin);
			}
			else
			{
				*dataOutputRegister &= ~(1 << dataPin);
			}
			*clockPinOutRegister |= (1 << clockPin);
			*clockPinOutRegister &= ~(1 << clockPin);
		}
	}
	
}



/**
* \brief Samples ADC for sampleCount Counts on channel adcChannel, returning the average
*
*/
uint16_t SampleAdc(uint8_t adcChannel, uint8_t sampleCount)
{
	//Variables
	uint16_t adcResult = 0; //Return value from the ADC
	
	//If the sample count is greater than 0, avoid divide by 0 errors,...
	if(sampleCount > 0)
	{
		//Create a variable for the loop
		uint8_t i = 0;

		//Select the passed channel
		ADMUX |= adcChannel;

		//While the index is less than the passed sample count...
		while(i < sampleCount)
		{
			i++;

			//Start our conversion
			ADCSRA |= (1 << ADSC);
			
			//Wait until the conversion is finished
			while(((ADCSRA >> ADSC) & 0x01));
			
			//Add our result
			adcResult += ADC;
		}
		
		//Divide our result by the passed sample count
		adcResult /= sampleCount;

		//Make sure the clear the selected channel on the way out
		ADMUX &= ~adcChannel;

	}
	
	//Return our result
	return adcResult;
}



/**
* \brief Shifts out an entire byte from sampling over x counts
*
*/
void AdcToByteShift()
{
	//Variables
	uint8_t canStart = 0; //If we can shift
	static uint8_t currentOutput; //Our current output value
	
	//Set our read pin to read
	PORTB |= (1 << SELECT_PIN);
	DDRB &= ~(1 << SELECT_PIN);
	
	//Read the pin
	canStart = !(PINB & (1 << SELECT_PIN));
	
	//Reset the pin back to output low
	DDRB |= (1 << SELECT_PIN);
	PORTB &= ~(1 << SELECT_PIN);
	
	//If we can being sampling...
	if(canStart)
	{
		//Get our current output byte
		currentOutput = SampleByteOverMicro(adcCutout,10, true);
		
		//Ground our latch
		SHIFT_REG_OUT_REG &= ~(1 << SHIFT_REG_LATCH);
		
		//Shift out the value
		ShiftOut(currentOutput,&SHIFT_REG_OUT_REG, SHIFT_REG_DATA, &SHIFT_REG_OUT_REG, SHIFT_REG_CLK,false);
		
		//Disable our latch
		SHIFT_REG_OUT_REG |= (1 << SHIFT_REG_LATCH);
	}
}



/**
* \brief Shifts out a bit based on an ADC sample and whether or the appropriate select pin is grounded
*
*/
void AdcToBitShift()
{
	//Variables
	uint8_t canStart = 0; //If we can shift
	static uint8_t currentOutput; //Our current output value
	
	//Set our read pin to read
	PORTB |= (1 << SELECT_PIN);
	DDRB &= ~(1 << SELECT_PIN);
	
	//Read the pin
	canStart = !(PINB & (1 << SELECT_PIN));
	
	//Reset the pin back to output low
	DDRB |= (1 << SELECT_PIN);
	PORTB &= ~(1 << SELECT_PIN);
	
	//Delay for a bit
	_delay_us(50);
	
	//If we can go to the next bit...
	if(canStart)
	{
		//Shift our current value over to the left
		currentOutput <<= 1;
		
		//If our sample returns true...
		if(SampleNext())
		{
			//Set the first bit
			currentOutput |= 1;
		}
		
		//Ground our latch
		SHIFT_REG_OUT_REG &= ~(1 << SHIFT_REG_LATCH);
		
		//Shift out the value
		ShiftOut(currentOutput,&SHIFT_REG_OUT_REG, SHIFT_REG_DATA, &SHIFT_REG_OUT_REG, SHIFT_REG_CLK,false);
		
		//Disable our latch
		SHIFT_REG_OUT_REG |= (1 << SHIFT_REG_LATCH);
	}
}



/**
* \brief Determines if the next bit can be set or not by evaluating an ADC reading
* \return True if bit is high, false if bit is low
*/
bool SampleNext()
{
	//Variables
	bool isSet = false; //If the bit is set or not
	
	//Get our ADC reading
	volatile uint16_t adcReading = SampleAdc(ADC_A, ADC_SAMPLES);
	
	//Set our lowest adc value
	adcLowest = (adcReading < adcLowest) ? adcReading : adcLowest;
	
	//Set our highest adc value
	adcHighest = (adcReading > adcHighest) ? adcReading : adcHighest;
	
	//If our highest is greater than our lowest...
	if(adcHighest > adcLowest)
	{
		//Set our cutout to the mid point
		adcCutout = ((adcHighest - adcLowest) / 2);
	}
	
	//If our adc reading is greater than or equal to the cutout...
	if(adcReading >= adcCutout)
	{
		//Set the bit
		isSet = true;
	}
	
	
	
	return isSet;
}



/**
* \brief Samples a byte over the adc for a max of up the x micro delay
* \param adcForActive The value that's required for a bit to be registered as high
* \param microDelayPerReading The amount counts with a us delay of 1 to read a bit for
* \param msbFirst most significant bit first? yes or no?
* \return The sampled byte
*/
unsigned char SampleByteOverMicro(uint16_t adcForActive, uint16_t microDelayPerReading, bool msbFirst)
{
	//Variables
	uint8_t outVal = 0; //The final value read
	int8_t index = 7; //The bit index
	uint16_t adcReading = 0; //The currently active reading
	
	//Do this...
	do
	{
		
		//Loop through the passed counts
		for(uint16_t j = 0; j < microDelayPerReading; j++)
		{
			//Delay
			_delay_us(1);
			
			//Read the adc
			adcReading = SampleAdc(ADC_A, ADC_SAMPLES);
			
			//Set our saved lowest adc value received
			adcLowest = (adcReading < adcLowest) ? adcReading : adcLowest;
			
			//Set our saved highest adc value received
			adcHighest = (adcReading > adcHighest) ? adcReading : adcHighest;
			
			//If the reading is greater than the passed active value...
			if(adcReading >= adcForActive)
			{
				//If msb first...
				if(msbFirst)
				{
					//Set the MSB bit
					outVal |= (1 << index);
				}
				//else...
				else
				{
					//Set the LSB bit
					outVal |= (1 << (7 - index));
				}
			}
		}
		
	}
	//While our index is greater than or equal to 0
	while (--index >= 0);
	
	//If our highest adc is greater than our lowest...
	if(adcHighest > adcLowest)
	{
		//Set our cutout to the mid point
		adcCutout = ((adcHighest - adcLowest) / 2);
	}
	
	//Return our out value
	return outVal;
}



/**
* \brief Samples an adc byte and shifts it onto the shift register at all times, no select pin used
*
*/
void AdcToShiftout()
{
	//Variables
	static uint8_t currentOutput; //The value to put out onto the shift register
	
	//Get our current output value
	currentOutput = SampleByteOverMicro(adcCutout, 10, true);
	
	//Ground our latch
	SHIFT_REG_OUT_REG &= ~(1 << SHIFT_REG_LATCH);
	
	//Shift out the value
	ShiftOut(currentOutput,&SHIFT_REG_OUT_REG, SHIFT_REG_DATA, &SHIFT_REG_OUT_REG, SHIFT_REG_CLK,false);
	
	//Disable our latch
	SHIFT_REG_OUT_REG |= (1 << SHIFT_REG_LATCH);
}

