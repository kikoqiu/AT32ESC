#pragma once
#include "main.h"

extern uint8_t eepromBuffer[176];
extern uint16_t TIMER1_MAX_ARR;

extern uint32_t gcr[26];
extern uint16_t adjusted_input;
extern uint32_t dma_buffer[64];
extern uint8_t dshotcommand;
extern char forward;
extern uint8_t running;
extern uint16_t zero_input_count;
extern uint16_t signaltimeout;
extern uint16_t input;
extern uint16_t newinput;
extern char play_tone_flag;
extern uint32_t current_GPIO_PIN;
extern uint32_t current_GPIO_PORT;
extern uint32_t current_EXTI_LINE;
extern char dshot_extended_telemetry;
extern uint16_t send_extended_dshot;

extern uint16_t comp_change_time;
extern uint16_t interrupt_time;



