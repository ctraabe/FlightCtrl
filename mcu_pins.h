#ifndef MCU_PINS_H_
#define MCU_PINS_H_


#include <avr/io.h>


// ADC
#define ADC_DDR (DDRA)
#define ADC_PORT (PORTA)
#define ACCELEROMETER_X_PIN (_BV(7))
#define ACCELEROMETER_Y_PIN (_BV(6))
#define ACCELEROMETER_Z_PIN (_BV(5))
#define BARO_PRESSURE_PIN (_BV(3))
#define BATTERY_VOLTAGE_PIN (_BV(4))
#define GYRO_X_PIN (_BV(1))
#define GYRO_Y_PIN (_BV(2))
#define GYRO_Z_PIN (_BV(0))

// Buzzer
#define BUZZER_DDR (DDRC)
#define BUZZER_PORT (PORTC)
#define BUZZER_PIN (_BV(7))

// LED
#define LED_DDR (DDRB)
#define LED_PORT (PORTB)
#define GREEN_LED_PIN (_BV(1))
#define RED_LED_PIN (_BV(0))

// I2C (MOTORS)
#define I2C_DDR (DDRC)
#define I2C_PORT (PORTC)
#define I2C_SCL_PIN (_BV(0))
#define I2C_SDA_PIN (_BV(1))

// Pressure Sensor
#define PRESSURE_BIAS_DDR (DDRB)
#define PRESSURE_BIAS_PORT (PORTB)
#define PRESSURE_BIAS_COARSE_PIN (_BV(4))
#define PRESSURE_BIAS_FINE_PIN (_BV(3))

// Board Version
#define VERSION_2_2_DDR (DDRD)
#define VERSION_2_2_PORT (PORTD)
#define VERSION_2_2_PIN (_BV(4))
#define VERSION_2_2 (~PIND & _BV(4))


#endif  // MCU_PINS_H_
