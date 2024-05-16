#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>
#include <stdint.h>
#include "hardware/i2c.h"
#include "pico/binary_info.h"

const int LCD_CLEARDISPLAY = 0x01;
const int LCD_RETURNHOME = 0x02;
const int LCD_ENTRYMODESET = 0x04;
const int LCD_DISPLAYCONTROL = 0x08;
const int LCD_CURSORSHIFT = 0x10;
const int LCD_FUNCTIONSET = 0x20;
const int LCD_SETCGRAMADDR = 0x40;
const int LCD_SETDDRAMADDR = 0x80;

// flags for display entry mode
const int LCD_ENTRYSHIFTINCREMENT = 0x01;
const int LCD_ENTRYLEFT = 0x02;

// flags for display and cursor control
const int LCD_BLINKON = 0x01;
const int LCD_CURSORON = 0x02;
const int LCD_DISPLAYON = 0x04;

// flags for display and cursor shift
const int LCD_MOVERIGHT = 0x04;
const int LCD_DISPLAYMOVE = 0x08;

// flags for function set
const int LCD_5x10DOTS = 0x04;
const int LCD_2LINE = 0x08;
const int LCD_8BITMODE = 0x10;

// flag for backlight control
const int LCD_BACKLIGHT = 0x08;

const int LCD_ENABLE_BIT = 0x04;

static int addr = 0x27;

// Modes for lcd_send_byte
#define LCD_CHARACTER 1
#define LCD_COMMAND 0

#define MAX_LINES 2
#define MAX_CHARS 16

/* Quick helper function for single byte transfers */
void i2c_write_byte(uint8_t val) {
#ifdef i2c_default
    i2c_write_blocking(i2c_default, addr, &val, 1, false);
#endif
}

void lcd_toggle_enable(uint8_t val) {
    // Toggle enable pin on LCD display
    // We cannot do this too quickly or things don't work
#define DELAY_US 600
    sleep_us(DELAY_US);
    i2c_write_byte(val | LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
    i2c_write_byte(val & ~LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
}

// The display is sent a byte as two separate nibble transfers
void lcd_send_byte(uint8_t val, int mode) {
    uint8_t high = mode | (val & 0xF0) | LCD_BACKLIGHT;
    uint8_t low = mode | ((val << 4) & 0xF0) | LCD_BACKLIGHT;

    i2c_write_byte(high);
    lcd_toggle_enable(high);
    i2c_write_byte(low);
    lcd_toggle_enable(low);
}

void lcd_clear(void) {
    lcd_send_byte(LCD_CLEARDISPLAY, LCD_COMMAND);
}

// go to location on LCD
void lcd_set_cursor(int line, int position) {
    int val = (line == 0) ? 0x80 + position : 0xC0 + position;
    lcd_send_byte(val, LCD_COMMAND);
}

static void inline lcd_char(char val) {
    lcd_send_byte(val, LCD_CHARACTER);
}

void lcd_string(const char *s) {
    while (*s) {
        lcd_char(*s++);
    }
}

void lcd_init() {
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x02, LCD_COMMAND);
    lcd_send_byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT, LCD_COMMAND);
    lcd_send_byte(LCD_FUNCTIONSET | LCD_2LINE, LCD_COMMAND);
    lcd_send_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON, LCD_COMMAND);
    lcd_clear();
}



void blink_led(uint delay_ms, uint LED_PIN) {
        gpio_put(LED_PIN, 1); // Turn the LED on
        sleep_ms(delay_ms);   // Wait for 'delay_ms' milliseconds
        gpio_put(LED_PIN, 0); // Turn the LED off
        sleep_ms(delay_ms);   // Wait for 'delay_ms' milliseconds
        return;
}


int main() { 
    

    #if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/lcd_1602_i2c example requires a board with I2C pins
#else
    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    lcd_init();


    lcd_set_cursor(0, 0);
    lcd_string("Hello, World!");
    lcd_set_cursor(1, 0);
    lcd_string("RP2040 by RPi");

    // while(true) {
        
    // }
#endif
    const uint left_button = 0;
    const uint right_button = 2;
    const uint middle_button = 1;
    const uint color_pin = 10;
    const uint pos_pin1 = 11;
    const uint pos_pin2 = 12;

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_init(left_button);
    gpio_init(right_button);
    gpio_init(middle_button);
    gpio_init(color_pin);
    gpio_init(pos_pin1);
    gpio_init(pos_pin2);

    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_set_dir(left_button, GPIO_IN);
    gpio_set_dir(right_button, GPIO_IN);
    gpio_set_dir(middle_button, GPIO_IN);
    gpio_set_dir(color_pin, GPIO_OUT);
    gpio_set_dir(pos_pin1, GPIO_OUT);
    gpio_set_dir(pos_pin2, GPIO_OUT);

    bool display_menu = false;
    bool menu_selected = false;
    bool blue_selected = false;
    gpio_pull_up(left_button);
    gpio_pull_up(middle_button);
    gpio_pull_up(right_button);
    int score= 0;
    while (true) {
        if(!gpio_get(middle_button)) {
        display_menu = true;
    }
    if (!gpio_get(left_button)) {
        while(!gpio_get(left_button)==1);
            score = score +5 ;
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_string("CYBERBEETLE");
            lcd_set_cursor(1, 0);
            lcd_string("Score: ");
            char score_str[10];
            sprintf(score_str, "%d", score);
            lcd_string(score_str);
            while(!gpio_get(left_button) == 1);

            }
        if (!gpio_get(right_button)) {
            while(!gpio_get(right_button)==1);
            score-=1;
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_string("CYBERBEETLE");
            lcd_set_cursor(1, 0);
            lcd_string("Score: ");
            char score_str[10];
            sprintf(score_str, "%d", score);
            lcd_string(score_str);
            
            while(!gpio_get(right_button) == 1);
            }
    if(display_menu) {
        score = 0;
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_string("Menu:");
        lcd_set_cursor(1, 0);
        lcd_string("1. Blue");
        lcd_set_cursor(1, 9);
        lcd_string("2. Yellow");

        // Reset flags
        menu_selected = false;
        blue_selected = false;

        // Reset pins
        gpio_put(color_pin, 0);
        gpio_put(pos_pin1, 0);
        gpio_put(pos_pin2, 0);

        // Wait for button release
        while(!gpio_get(middle_button) == 1);

        while (!menu_selected) {
            if (!gpio_get(left_button)) {
                blue_selected = true;
                menu_selected = true;
                gpio_put(color_pin, 1);

            }

            if (!gpio_get(right_button)) {
                blue_selected = false;
                menu_selected = true;
                gpio_put(color_pin, 0);
            }
        }
        if (blue_selected) {
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_string("You chose:");
            lcd_set_cursor(1, 0);
            lcd_string("Blue");
        } else {
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_string("You chose:");
            lcd_set_cursor(1, 0);
            lcd_string("Yellow");
        }
        sleep_ms(2000);
        // Second menu
        lcd_clear();;
        lcd_set_cursor(0, 0);
        lcd_string("Second Menu:");
        lcd_set_cursor(1, 0);
        lcd_string("1.L");

        lcd_set_cursor(1, 7);

        lcd_string("2.M");
        lcd_set_cursor(1, 13);

        lcd_string("3.R");
        // Reset flags
        bool position_selected = false;
        bool left_selected = false;
        bool right_selected = false;
        bool middle_selected = false;
        // Wait for button release
        
        // Display the selected option
        lcd_clear();
        
        blink_led(100, PICO_DEFAULT_LED_PIN);
        // Add a delay before going back to default display
        sleep_ms(1000);
        // Clear LCD and go back to default display
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_string("CYBERBEETLE");
        lcd_set_cursor(1, 0);
        lcd_string("Score: ");
        // Convert the score to a string
        char score_str[10];
        sprintf(score_str, "%d", score);
        // Write the score on the screen
        lcd_string(score_str);

        display_menu = false;
        }
        
        
    }

    
    




    return 0;
}
