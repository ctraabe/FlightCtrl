#ifndef MCU_PINS_H_
#define MCU_PINS_H_


#include <avr/io.h>


// ADC
#define ADC_DDR (DDRA)
#define ADC_PORT (PORTA)
#define ACCELEROMETER_X_PIN (_BV(PORTA7))
#define ACCELEROMETER_Y_PIN (_BV(PORTA6))
#define ACCELEROMETER_Z_PIN (_BV(PORTA5))
#define BARO_PRESSURE_PIN (_BV(PORTA3))
#define BATTERY_VOLTAGE_PIN (_BV(PORTA4))
#define GYRO_X_PIN (_BV(PORTA1))
#define GYRO_Y_PIN (_BV(PORTA2))
#define GYRO_Z_PIN (_BV(PORTA0))

// Buzzer
#define BUZZER_DDR (DDRC)
#define BUZZER_PORT (PORTC)
#define BUZZER_PIN (_BV(PORTC7))


// LED
#define LED_DDR (DDRB)
#define LED_PORT (PORTB)
#define GREEN_LED_PIN (_BV(PORTB1))
#define RED_LED_PIN (_BV(PORTB0))


// I2C (MOTORS)
#define I2C_DDR (DDRC)
#define I2C_PORT (PORTC)
#define I2C_SCL_PIN (_BV(PORTC0))
#define I2C_SDA_PIN (_BV(PORTC1))


#endif  // MCU_PINS_H_
