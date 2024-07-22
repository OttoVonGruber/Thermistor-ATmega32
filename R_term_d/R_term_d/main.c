#define F_CPU 1000000 // Set the CPU frequency to 1 MHz
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

//[-------------Global---------------]
#define RS 7 // Register Select pin for LCD
#define E 6  // Enable pin for LCD
#define CS PB4  // Chip Select pin for SPI

//[-------------Prototyp---------------]
void lcd_init(void); // Initializes the LCD
void lcd_cmd(uint8_t cmd); // Sends a command to the LCD
void lcd_data(char data); // Sends data (a character) to the LCD
void lcd_print(char *str); // Prints a string on the LCD
void lcd_set_cursor(uint8_t row, uint8_t col); // Sets the cursor position on the LCD
void display_temperature(int temperature); // Displays the temperature on the LCD
void int_to_string_fixed(char* buffer, int value, uint8_t width); // Converts an integer to a fixed-width string

void spi_init(void); // Initializes SPI communication
uint8_t spi_transfer(uint8_t data); // Transfers data over SPI and receives the response
uint16_t read_mcp3208(uint8_t channel); // Reads a value from the MCP3208 ADC
int get_temperature(uint16_t adc_value); // Converts ADC value to temperature


int main(void)
{
	lcd_init(); // Initialize the LCD
	spi_init(); // Initialize SPI communication
	
	while (1)
	{
		uint16_t adc_value = read_mcp3208(0); 
		int temperature = get_temperature(adc_value); 
		
		display_temperature(temperature); 
		_delay_ms(1000); 
	}
}


//[------------Function-----------]
void lcd_init(void){
	DDRD = 0xFF; // Set PORTD as output for LCD data
	DDRC |= ((1 << E) | (1 << RS)); // Set RS and E as output
	_delay_ms(100); // Delay for LCD power on
	lcd_cmd(0x38); // Function set: 8-bit, 2 line, 5x7 dots
	lcd_cmd(0x0E); // Display on, cursor on
	lcd_cmd(0x06); // Entry mode, increment cursor
	lcd_cmd(0x01); // Clear display
}

void lcd_cmd(uint8_t cmd){
	PORTC &= ~(1 << RS); // RS = 0 for command
	PORTD = cmd; // Put command on data bus
	PORTC |= (1 << E); // Enable pulse
	_delay_us(1);
	PORTC &= ~(1 << E); // Disable pulse
	_delay_ms(2); // Wait for command to be processed
}

void lcd_data(char data){
	PORTC |= (1 << RS); // RS = 1 for data
	PORTD = data; // Put data on data bus
	PORTC |= (1 << E); // Enable pulse
	_delay_us(1);
	PORTC &= ~(1 << E); // Disable pulse
	_delay_ms(2); // Wait for data to be processed
}

void lcd_print(char *str) {
	while (*str) {
		lcd_data(*str++); // Send characters one by one
	}
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
	uint8_t address = (row == 0 ? 0x80 : 0xC0) + col; // Set cursor position
	lcd_cmd(address); // Send command to set cursor
}

void int_to_string_fixed(char* buffer, int value, uint8_t width) {
	if (value < 0) {
		*buffer++ = '-'; // Add minus sign for negative values
		value = -value;
		width--;
	}
	buffer += width; // Move to the end of the buffer
	*buffer-- = '\0'; // Null-terminate the string
	for (int i = 0; i < width; i++) {
		if (value > 0) {
			*buffer-- = (value % 10) + '0'; // Convert digit to character
			value /= 10;
			} else {
			*buffer-- = '0'; // Fill remaining width with zeros
		}
	}
}

void display_temperature(int temperature) {
	char buffer[16];
	
	lcd_set_cursor(0, 0);  // Set cursor to the beginning of the first line
	int_to_string_fixed(buffer, temperature, 4); // Convert temperature to string
	lcd_print("Temp:"); 
	lcd_print(buffer); 
	lcd_print("_C"); 
}

void spi_init(void) {
	DDRB = (1 << PB7) | (1 << PB4) | (1 << PB5); // Set MOSI, SCK, and CS as output
	DDRB &= ~(1 << PB6); // Set MISO as input
	SPCR = (1 << SPE) | (1 << MSTR); // Enable SPI, set as master
	SPSR = (1 << SPI2X);  // Enable double speed
}

uint8_t spi_transfer(uint8_t data) {
	SPDR = data; // Start transmission
	while (!(SPSR & (1 << SPIF))); // Wait for transmission complete
	return SPDR; // Return received data
}

uint16_t read_mcp3208(uint8_t channel) {
	uint8_t command = 0x18 | (channel >> 1); // Construct command byte
	uint8_t msb, lsb;

	PORTB &= ~(1 << CS);  // CS Low to start communication
	spi_transfer(command); // Send command
	msb = spi_transfer(channel << 6); // Send channel information
	lsb = spi_transfer(0x00); // Receive lower byte
	PORTB |= (1 << CS);  // CS High to end communication

	return ((msb & 0x0F) << 8) | lsb; // Combine bytes to get 12-bit result
}

int get_temperature(uint16_t adc_value) {
	float voltage = adc_value * (5.0 / 4096.0); // Convert ADC value to voltage
	float resistance = (voltage * 1000.0) / (5.0 - voltage); // Calculate RTD resistance
	
	float calibration_offset = 0.0; 
	float calibration_slope = 1.0;  

	int temperature = (int)(((resistance - 100.0) / 0.385) * calibration_slope + calibration_offset); // Calculate temperature

	return temperature;
}
