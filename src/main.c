/* AM32- multi-purpose brushless controller firmware  */


#ifdef AT32F421K8U7
#include "at32f421.h"
#endif
#ifdef AT32F415K8U7
#include "at32f415.h"
#endif

//#include "systick.h"
#include <stdio.h>
#include "main.h"

#include "targets.h"
#include "signal.h"
#include "dshot.h"
#include "phaseouts.h"
#include "eeprom.h"
#include "sounds.h"
#include "ADC.h"
#ifdef USE_SERIAL_TELEMETRY
#include "serial_telemetry.h"
#endif
#include "IO.h"
#include "comparator.h"
#include "functions.h"
#include "peripherals.h"
#include "common.h"
#include "firmwareversion.h"

uint16_t comp_change_time = 0;

#define VERSION_MAJOR 1
#define VERSION_MINOR 93

// firmware build options !! fixed speed and duty cycle modes are not to be used with sinusoidal startup !!

//#define FIXED_DUTY_MODE	// bypasses signal input and arming, uses a set duty cycle. For pumps, slot cars etc
//#define FIXED_DUTY_MODE_POWER 10		 // 0-100 percent not used in fixed speed mode

//#define FIXED_SPEED_MODE	// bypasses input signal and runs at a fixed rpm using the speed control loop PID
//#define FIXED_SPEED_MODE_RPM	1000	// intended final rpm , ensure pole pair numbers are entered correctly in config tool.

//===========================================================================
//=============================	Defaults =============================
//===========================================================================

//uint8_t drive_by_rpm = 0;
uint32_t MAXIMUM_RPM_SPEED_CONTROL = 12000;
uint32_t MINIMUM_RPM_SPEED_CONTROL = 2000;



uint16_t target_e_com_time_high;
uint16_t target_e_com_time_low;

char eeprom_layout_version = 2;
char dir_reversed = 0;
char comp_pwm = 1;
char VARIABLE_PWM = 1;
char bi_direction = 0;
char stuck_rotor_protection = 1; // Turn off for Crawlers
char brake_on_stop = 0;
//char stall_protection = 0;
//char use_sin_start = 0;
char TLM_ON_INTERVAL = 0;
uint8_t telemetry_interval_ms = 30;
uint8_t TEMPERATURE_LIMIT = 255; // degrees 255 to disable
char advance_level = 2;			 // 7.5 degree increments 0 , 7.5, 15, 22.5)
uint16_t motor_kv = 2000;
char motor_poles = 14;
uint16_t CURRENT_LIMIT = 202;
//uint8_t sine_mode_power = 5;
char drag_brake_strength = 10; // Drag Brake Power when brake on stop is enabled
uint8_t driving_brake_strength = 10;
uint8_t dead_time_override = DEAD_TIME;
//char sine_mode_changeover_thottle_level = 5; // Sine Startup Range
uint16_t stall_protect_target_interval = TARGET_STALL_PROTECTION_INTERVAL;
char USE_HALL_SENSOR = 0;
//uint16_t enter_sine_angle = 180;

//============================= Servo Settings ==============================
uint16_t servo_low_threshold = 1100;  // anything below this point considered 0
uint16_t servo_high_threshold = 1900; // anything above this point considered 2000 (max)
uint16_t servo_neutral = 1500;
uint8_t servo_dead_band = 100;

//========================= Battery Cuttoff Settings ========================
char LOW_VOLTAGE_CUTOFF = 0;		 // Turn Low Voltage CUTOFF on or off
uint16_t low_cell_volt_cutoff = 330; // 3.3volts per cell

//=========================== END EEPROM Defaults ===========================

// typedef struct __attribute__((packed)) {
//	uint8_t version_major;
//	uint8_t version_minor;
//	char device_name[12];
// } firmware_info_s;

// firmware_info_s __attribute__ ((section(".firmware_info"))) firmware_info = {
//	version_major: VERSION_MAJOR,
//	version_minor: VERSION_MINOR,
//	device_name: FIRMWARE_NAME
// };

uint8_t EEPROM_VERSION;
// move these to targets folder or peripherals for each mcu
char RC_CAR_REVERSE = 0; // have to set bidirectional, comp_pwm off and stall protection off, no sinusoidal startup
uint16_t ADC_CCR = 30;
uint16_t current_angle = 90;
uint16_t desired_angle = 90;


char boot_up_tune_played = 0;
uint16_t target_e_com_time = 0;
//int16_t Speed_pid_output;
//char use_speed_control_loop = 0;
float input_override = 0;
int16_t use_current_limit_adjust = 2000;
char use_current_limit = 0;
//float stall_protection_adjust = 0;

uint32_t MCU_Id = 0;
uint32_t REV_Id = 0;

uint16_t armed_timeout_count;
uint16_t reverse_speed_threshold = 1500;
uint8_t desync_happened = 0;
char maximum_throttle_change_ramp = 1;

char crawler_mode = 0; // no longer used //
uint16_t velocity_count = 0;
uint16_t velocity_count_threshold = 75;

char low_rpm_throttle_limit = 1;

uint16_t low_voltage_count = 0;
uint16_t telem_ms_count;

char VOLTAGE_DIVIDER = TARGET_VOLTAGE_DIVIDER; // 100k upper and 10k lower resistor in divider
uint16_t battery_voltage;					   // scale in volts * 10.	1260 is a battery voltage of 12.60
char cell_count = 0;
char brushed_direction_set = 0;

uint16_t tenkhzcounter = 0;
float consumed_current = 0;
uint16_t smoothed_raw_current = 0;
uint16_t actual_current = 0;

char lowkv = 0;

uint16_t min_startup_duty = 120;
uint16_t sin_mode_min_s_d = 120;
char bemf_timeout = 10;

char startup_boost = 50;
char reversing_dead_band = 1;

uint16_t low_pin_count = 0;

uint8_t max_duty_cycle_change = 2;
char fast_accel = 1;
uint16_t last_duty_cycle = 0;
char play_tone_flag = 0;

typedef enum
{
	GPIO_PIN_RESET = 0U,
	GPIO_PIN_SET
} GPIO_PinState;

uint16_t startup_max_duty_cycle = 300 + DEAD_TIME;
uint16_t minimum_duty_cycle = DEAD_TIME;
uint16_t stall_protect_minimum_duty = DEAD_TIME;
char desync_check = 0;
char low_kv_filter_level = 20;

uint16_t tim1_arr = TIM1_AUTORELOAD;		   // current auto reset value
uint16_t TIMER1_MAX_ARR = TIM1_AUTORELOAD;	   // maximum auto reset register value
uint16_t duty_cycle_maximum = TIM1_AUTORELOAD; // restricted by temperature or low rpm throttle protect
uint16_t low_rpm_level = 20;				   // thousand erpm used to set range for throttle resrictions
uint16_t high_rpm_level = 70;				   //
uint16_t throttle_max_at_low_rpm = 400;
uint16_t throttle_max_at_high_rpm = TIM1_AUTORELOAD;

uint16_t commutation_intervals[6] = {0};
uint32_t average_interval = 0;
uint32_t last_average_interval;
int e_com_time;

uint16_t ADC_smoothed_input = 0;
uint8_t degrees_celsius;
uint16_t converted_degrees;
uint8_t temperature_offset;
uint16_t ADC_raw_temp;
uint16_t ADC_raw_volts;
uint16_t ADC_raw_current;
uint16_t ADC_raw_input;
uint8_t adc_counter = 0;
char send_telemetry = 0;
char telemetry_done = 0;
char prop_brake_active = 0;

uint8_t eepromBuffer[176] = {0};

char dshot_telemetry = 0;

uint8_t last_dshot_command = 0;
char old_routine = 0;
uint16_t adjusted_input = 0;

#define TEMP30_CAL_VALUE ((uint16_t *)((uint32_t)0x1FFFF7B8))
#define TEMP110_CAL_VALUE ((uint16_t *)((uint32_t)0x1FFFF7C2))

uint16_t smoothedinput = 0;
const uint8_t numReadings = 30; // the readings from the analog input
uint8_t readIndex = 0;			// the index of the current reading
int total = 0;
uint16_t readings[30];

uint8_t bemf_timeout_happened = 0;
uint8_t changeover_step = 5;
uint8_t filter_level = 5;
uint8_t running = 0;
uint16_t advance = 0;
const uint8_t advancedivisor = 4;
char rising = 1;


char forward = 1;
uint16_t gate_drive_offset = DEAD_TIME;

uint8_t stuckcounter = 0;
uint16_t k_erpm;
uint16_t e_rpm; // electrical revolution /100 so,	123 is 12300 erpm

uint16_t adjusted_duty_cycle;

uint8_t bad_count = 0;
uint8_t dshotcommand;
uint16_t armed_count_threshold = 1000;

char armed = 0;
uint16_t zero_input_count = 0;

uint16_t input = 0;
uint16_t newinput = 0;
char inputSet = 0;
char dshot = 0;
char servoPwm = 0;
uint32_t zero_crosses;

uint8_t zcfound = 0;

uint8_t bemfcounter;
uint8_t min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS;
uint8_t min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS;

uint16_t lastzctime;
uint16_t thiszctime;

uint16_t duty_cycle = 0;
char step = 1;
uint16_t commutation_interval = 12500;

uint16_t signaltimeout = 0;
uint8_t ubAnalogWatchdogStatus = RESET;

#include "settings.h"


void checkForHighSignal()
{
	changeToInput();
	gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_INPUT, GPIO_PULL_DOWN, INPUT_PIN);
	delayMicros(100);
	for (int i = 0; i < 1000; i++)
	{
		if (!((INPUT_PIN_PORT->idt & INPUT_PIN)))
		{ // if the pin is low for 5 checks out of 100 in	100ms or more its either no signal or signal. jump to application
			low_pin_count++;
		}
		delayMicros(10);
	}
	gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_NONE, INPUT_PIN);
	if (low_pin_count > 5)
	{
		return; // its either a signal or a disconnected pin
	}
	else
	{
		allOff();
		NVIC_SystemReset();
	}
}


void getSmoothedInput()
{

	total = total - readings[readIndex];
	readings[readIndex] = commutation_interval;
	total = total + readings[readIndex];
	readIndex = readIndex + 1;
	if (readIndex >= numReadings)
	{
		readIndex = 0;
	}
	smoothedinput = total / numReadings;
}

void getBemfState()
{
	uint8_t current_state = 0;
#ifdef MCU_AT415
	current_state = !(CMP->ctrlsts1_bit.cmp1value); // polarity reversed
#else
	current_state = !(CMP->ctrlsts_bit.cmpvalue); // polarity reversed
#endif
	if (rising)
	{
		if (current_state)
		{
			bemfcounter++;
		}
		else
		{
			bad_count++;
			if (bad_count > 2)
			{
				bemfcounter = 0;
			}
		}
	}
	else
	{
		if (!current_state)
		{
			bemfcounter++;
		}
		else
		{
			bad_count++;
			if (bad_count > 2)
			{
				bemfcounter = 0;
			}
		}
	}
}

void commutate()
{
	commutation_intervals[step - 1] = commutation_interval;
	e_com_time = (commutation_intervals[0] + commutation_intervals[1] + commutation_intervals[2] + commutation_intervals[3] + commutation_intervals[4] + commutation_intervals[5]) >> 1; // COMMUTATION INTERVAL IS 0.5US INCREMENTS

	if (forward == 1)
	{
		step++;
		if (step > 6)
		{
			step = 1;
			desync_check = 1;
		}
		rising = step % 2;
	}
	else
	{
		step--;
		if (step < 1)
		{
			step = 6;
			desync_check = 1;
		}
		rising = !(step % 2);
	}
	/***************************************************/
	UTILITY_TIMER->cval = 0;
	if (!prop_brake_active)
	{
		comStep(step);
	}
	comp_change_time = UTILITY_TIMER->cval;
	/****************************************************/
	changeCompInput();
	if (average_interval > 2000 && (RC_CAR_REVERSE))
	{
		old_routine = 1;
	}
	bemfcounter = 0;
	zcfound = 0;	
}




void PeriodElapsedCallback()
{
	COM_TIMER->iden &= ~TMR_OVF_INT; // disable interrupt
	commutate();

	if (zero_crosses < 10000)
	{
		zero_crosses++;
	}
	
	if (!old_routine)
	{
		enableCompInterrupts(); // enable comp interrupt
	}else{
		bad_count = 0;
		if (RC_CAR_REVERSE)
		{
			if (zero_crosses >= 20 && commutation_interval <= 2000)
			{
				old_routine = 0;
				enableCompInterrupts(); 
			}
		}
		else
		{
			if (zero_crosses > 30)
			{
				old_routine = 0;
				enableCompInterrupts();
			}
		}
	}
}

void queueComm(uint16_t wtime){
	COM_TIMER->cval = 0;
	COM_TIMER->pr = wtime;
	COM_TIMER->ists = 0x00;
	COM_TIMER->iden |= TMR_OVF_INT;
}
void zcfoundroutine_nonblock()
{
	thiszctime = INTERVAL_TIMER->cval;
	INTERVAL_TIMER->cval = 0;
	commutation_interval = (thiszctime + (3 * commutation_interval)) / 4;
	advance = commutation_interval / advancedivisor;
	uint16_t waitTime  = commutation_interval / 2 - advance;
	queueComm(waitTime);
}

void interruptRoutine()
{
	if (average_interval > 125)
	{
		if ((INTERVAL_TIMER->cval < 125) && (duty_cycle < 600) && (zero_crosses < 500))
		{ // should be impossible, desync?exit anyway
			return;
		}
		if ((INTERVAL_TIMER->cval < (commutation_interval >> 1)))
		{
			return;
		}
		stuckcounter++; // stuck at 100 interrupts before the main loop happens again.
		if (stuckcounter > 100)
		{
			maskPhaseInterrupts();
			zero_crosses = 0;
			return;
		}
	}
	thiszctime = INTERVAL_TIMER->cval;
	if (rising)
	{
		for (int i = 0; i < filter_level; i++)
		{
			if (COMP_VALUE)
			{
				return;
			}
		}
	}
	else
	{
		for (int i = 0; i < filter_level; i++)
		{
			if (!COMP_VALUE)
			{
				return;
			}
		}
	}

	maskPhaseInterrupts();
	INTERVAL_TIMER->cval = 0;

	commutation_interval = ((3 * commutation_interval) + thiszctime) >> 2;
	advance = (commutation_interval >> 3) * advance_level; // 60 divde 8 7.5 degree increments
	uint16_t waitTime = (commutation_interval >> 1) - advance;
	queueComm(waitTime >> fast_accel);
}

void startMotor()
{
	if (running == 0)
	{
		commutate();
		commutation_interval = 10000;
		INTERVAL_TIMER->cval = 5000;
		running = 1;
	}
	enableCompInterrupts();
}

void tenKhzRoutine()
{
	tenkhzcounter++;

	if (boot_up_tune_played == 0)
	{
		if (tenkhzcounter > 1000)
		{
			playStartupTune();
			boot_up_tune_played = 1;
		}
	}

	if (tenkhzcounter > 10000)
	{ // 1s sample interval
		consumed_current = (float)actual_current / 360 + consumed_current;
		switch (dshot_extended_telemetry)
		{

		case 1:
			send_extended_dshot = 0b0010 << 8 | degrees_celsius;
			dshot_extended_telemetry = 2;
			break;
		case 2:
			send_extended_dshot = 0b0110 << 8 | (uint8_t)actual_current / 50;
			dshot_extended_telemetry = 3;
			break;
		case 3:
			send_extended_dshot = 0b0100 << 8 | (uint8_t)(battery_voltage / 25);
			dshot_extended_telemetry = 1;
			break;
		}
		tenkhzcounter = 0;
	}
	if (!armed)
	{
		if (inputSet)
		{
			if (adjusted_input == 0)
			{
				armed_timeout_count++;
				if (armed_timeout_count > 10000)
				{ // one second
					if (zero_input_count > 30)
					{
						armed = 1;
						//			receiveDshotDma();
#ifdef USE_RGB_LED
						GPIOB->BRR = LL_GPIO_PIN_3;	 // turn on green
						GPIOB->BSRR = LL_GPIO_PIN_8; // turn on green
						GPIOB->BSRR = LL_GPIO_PIN_5;
#endif
						if (cell_count == 0 && LOW_VOLTAGE_CUTOFF)
						{
							cell_count = battery_voltage / 370;
							for (int i = 0; i < cell_count; i++)
							{
								playInputTune();
								delayMillis(100);
								//							 IWDG_ReloadCounter(IWDG);
							}
						}
						else
						{
							playInputTune();
						}
						if (!servoPwm)
						{
							RC_CAR_REVERSE = 0;
						}
					}
					else
					{
						inputSet = 0;
						armed_timeout_count = 0;
					}
				}
			}
			else
			{
				armed_timeout_count = 0;
			}
		}
	}

	if (TLM_ON_INTERVAL)
	{
		telem_ms_count++;
		if (telem_ms_count > telemetry_interval_ms * 10)
		{
			send_telemetry = 1;
			telem_ms_count = 0;
		}
	}

	{
		if (input >= 47  && armed)
		{
			if (running == 0)
			{
				allOff();
				if (!old_routine)
				{
					startMotor();
				}
				running = 1;
				last_duty_cycle = min_startup_duty;
			}
			duty_cycle = map(input, 47, 2047, minimum_duty_cycle, TIMER1_MAX_ARR);
			
			if (tenkhzcounter % 10 == 0)
			{ // 1khz PID loop				
			}
			if (!RC_CAR_REVERSE)
			{
				prop_brake_active = 0;
			}
		}
		if (input < 47 )
		{
			if (play_tone_flag != 0)
			{
				if (play_tone_flag == 1)
				{
					playDefaultTone();
				}
				if (play_tone_flag == 2)
				{
					playChangedTone();
				}
				play_tone_flag = 0;
			}

			if (!comp_pwm)
			{
				duty_cycle = 0;
				if (!running)
				{
					old_routine = 1;
					zero_crosses = 0;
					if (brake_on_stop)
					{
						fullBrake();
					}
					else
					{
						if (!prop_brake_active)
						{
							allOff();
						}
					}
				}
				if (RC_CAR_REVERSE && prop_brake_active)
				{
#ifndef PWM_ENABLE_BRIDGE
					duty_cycle = getAbsDif(1000, newinput) + 1000;
					if (duty_cycle == 2000)
					{
						fullBrake();
					}
					else
					{
						proportionalBrake();
					}
#endif
				}
			}
			else
			{
				if (!running)
				{
					duty_cycle = 0;
					old_routine = 1;
					zero_crosses = 0;
					bad_count = 0;
					if (brake_on_stop)
					{
#ifndef PWM_ENABLE_BRIDGE
							duty_cycle = (TIMER1_MAX_ARR - 19) + drag_brake_strength * 2;
							proportionalBrake();
							prop_brake_active = 1;
#else
							// todo add proportional braking for pwm/enable style bridge.
#endif
					}
					else
					{
						allOff();
						duty_cycle = 0;
					}
				}
			}
		}
		if (!prop_brake_active)
		{
			if (zero_crosses < 20)
			{
				if (duty_cycle < min_startup_duty)
				{
					duty_cycle = min_startup_duty;
				}
				if (duty_cycle > startup_max_duty_cycle)
				{
					duty_cycle = startup_max_duty_cycle;
				}
			}

			if (duty_cycle > duty_cycle_maximum)
			{
				duty_cycle = duty_cycle_maximum;
			}
			if (use_current_limit)
			{
				if (duty_cycle > use_current_limit_adjust)
				{
					duty_cycle = use_current_limit_adjust;
				}
			}

			
			if (maximum_throttle_change_ramp)
			{
				//	max_duty_cycle_change = map(k_erpm, low_rpm_level, high_rpm_level, 1, 40);
				if (average_interval > 500)
				{
					max_duty_cycle_change = 10;
				}
				else
				{
					max_duty_cycle_change = 30;
				}
				if ((duty_cycle - last_duty_cycle) > max_duty_cycle_change)
				{
					duty_cycle = last_duty_cycle + max_duty_cycle_change;
					if (commutation_interval > 500)
					{
						fast_accel = 1;
					}
					else
					{
						fast_accel = 0;
					}
				}
				else if ((last_duty_cycle - duty_cycle) > max_duty_cycle_change)
				{
					duty_cycle = last_duty_cycle - max_duty_cycle_change;
					fast_accel = 0;
				}
				else
				{
					fast_accel = 0;
				}
			}
		}
		if ((armed && running) && input > 47)
		{
			if (VARIABLE_PWM)
			{
				tim1_arr = map(commutation_interval, 96, 200, TIMER1_MAX_ARR / 2, TIMER1_MAX_ARR);
			}
			adjusted_duty_cycle = ((duty_cycle * tim1_arr) / TIMER1_MAX_ARR) + 1;
		}
		else
		{
			if (prop_brake_active)
			{
				adjusted_duty_cycle = TIMER1_MAX_ARR - ((duty_cycle * tim1_arr) / TIMER1_MAX_ARR) + 1;
			}
			else
			{
				adjusted_duty_cycle = DEAD_TIME * running;
			}
		}
		last_duty_cycle = duty_cycle;
		TMR1->pr = tim1_arr;

		TMR1->c1dt = adjusted_duty_cycle;
		TMR1->c2dt = adjusted_duty_cycle;
		TMR1->c3dt = adjusted_duty_cycle;
	}
	average_interval = e_com_time / 3;
	if (desync_check && zero_crosses > 10)
	{
		//	if(average_interval < last_average_interval){
		//
		//	}
		if ((getAbsDif(last_average_interval, average_interval) > average_interval >> 1) && (average_interval < 2000))
		{ // throttle resitricted before zc 20.
			zero_crosses = 0;
			desync_happened++;
			running = 0;
			old_routine = 1;
			if (zero_crosses > 100)
			{
				average_interval = 5000;
			}
			last_duty_cycle = min_startup_duty / 2;
		}
		desync_check = 0;
		//	}
		last_average_interval = average_interval;
	}
	//#ifndef MCU_F031
	// if(commutation_interval > 400){
	//		 NVIC_SetPriority(IC_DMA_IRQ_NAME, 0);
	//		 NVIC_SetPriority(ADC_CMP_IRQn, 1);
	//}else{
	//	NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
	//	NVIC_SetPriority(ADC_CMP_IRQn, 0);
	//}
	//#endif	 //mcu f031



	if (send_telemetry)
	{
#ifdef USE_SERIAL_TELEMETRY
		makeTelemPackage(degrees_celsius,
						 battery_voltage,
						 actual_current,
						 (uint16_t)consumed_current,
						 e_rpm);
		send_telem_DMA();
		send_telemetry = 0;
#endif
	}
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)

//		if(INPUT_PIN_PORT->IDR & INPUT_PIN){
//			signaltimeout ++;
//			if(signaltimeout > 10000){
//				NVIC_SystemReset();
//			}
//		}else{
//			signaltimeout = 0;
//		}
#else

	signaltimeout++;

	if (signaltimeout > 2500 * (servoPwm + 1))
	{ // quarter second timeout when armed half second for servo;
		if (armed)
		{
			allOff();
			armed = 0;
			input = 0;
			inputSet = 0;
			zero_input_count = 0;

			TMR1->c1dt = 0;
			TMR1->c2dt = 0;
			TMR1->c3dt = 0;

			IC_TIMER_REGISTER->div = 0;
			IC_TIMER_REGISTER->cval = 0;

			for (int i = 0; i < 64; i++)
			{
				dma_buffer[i] = 0;
			}
			NVIC_SystemReset();
		}

		if (signaltimeout > 25000)
		{ // 2.5 second
			allOff();
			armed = 0;
			input = 0;
			inputSet = 0;
			zero_input_count = 0;

			TMR1->c1dt = 0;
			TMR1->c2dt = 0;
			TMR1->c3dt = 0;
			IC_TIMER_REGISTER->div = 0;
			IC_TIMER_REGISTER->cval = 0;
			for (int i = 0; i < 64; i++)
			{
				dma_buffer[i] = 0;
			}
			NVIC_SystemReset();
		}
	}
#endif
}



int main(void)
{
	__enable_irq();

	adc_counter = 2;
	initCorePeripherals();

	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1, TRUE);
	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3, TRUE);
	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1C, TRUE);
	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2C, TRUE);
	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3C, TRUE);

	/* Enable tim1 */
	TMR1->ctrl1_bit.tmren = TRUE;
	TMR1->brk_bit.oen = TRUE;

	/* Force update generation */
	TMR1->swevt |= TMR_OVERFLOW_SWTRIG;

	adc_counter = 4;
#ifdef USE_RGB_LED
	LED_GPIO_init();
	GPIOB->BRR = LL_GPIO_PIN_8; // turn on red
	GPIOB->BSRR = LL_GPIO_PIN_5;
	GPIOB->BSRR = LL_GPIO_PIN_3; //
#endif

	// commutation_timer priority 0
	COM_TIMER->ctrl1_bit.tmren = TRUE;
	COM_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
	COM_TIMER->iden &= ~TMR_OVF_INT;


	UTILITY_TIMER->ctrl1_bit.tmren = TRUE;

	INTERVAL_TIMER->ctrl1_bit.tmren = TRUE;

	INTERVAL_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
	adc_counter = 5;

	TEN_KHZ_TIMER->ctrl1_bit.tmren = TRUE;
	TEN_KHZ_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
	TEN_KHZ_TIMER->iden |= TMR_OVF_INT;

#ifdef USE_ADC
	ADC_Init();
#endif

#ifdef USE_ADC_INPUT

#else
	tmr_channel_enable(IC_TIMER_REGISTER, IC_TIMER_CHANNEL, TRUE);
	IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;

#endif

	adc_counter = 6;
	loadEEpromSettings();
	if (VERSION_MAJOR != eepromBuffer[3] || VERSION_MINOR != eepromBuffer[4])
	{
		eepromBuffer[3] = VERSION_MAJOR;
		eepromBuffer[4] = VERSION_MINOR;
		for (int i = 0; i < 12; i++)
		{
			eepromBuffer[5 + i] = (uint8_t)FIRMWARE_NAME[i];
		}
		saveEEpromSettings();
	}

	if (dir_reversed == 1)
	{
		forward = 0;
	}
	else
	{
		forward = 1;
	}
	tim1_arr = TIMER1_MAX_ARR;
	startup_max_duty_cycle = startup_max_duty_cycle * TIMER1_MAX_ARR / 2000 + dead_time_override; // adjust for pwm frequency
	throttle_max_at_low_rpm = throttle_max_at_low_rpm * TIMER1_MAX_ARR / 2000;					  // adjust to new pwm frequency
	throttle_max_at_high_rpm = TIMER1_MAX_ARR;													  // adjust to new pwm frequency

	adc_counter = 7;
	if (RC_CAR_REVERSE)
	{ // overrides a whole lot of things!
		throttle_max_at_low_rpm = 1000;
		bi_direction = 1;
		//use_sin_start = 0;
		low_rpm_throttle_limit = 1;
		VARIABLE_PWM = 0;
		comp_pwm = 0;
		stuck_rotor_protection = 0;
		minimum_duty_cycle = minimum_duty_cycle + 50;
		stall_protect_minimum_duty = stall_protect_minimum_duty + 50;
		min_startup_duty = min_startup_duty + 50;
	}

#ifdef MCU_F031
	GPIOF->BSRR = LL_GPIO_PIN_6; // uncomment to take bridge out of standby mode and set oc level
	GPIOF->BRR = LL_GPIO_PIN_7;	 // out of standby mode
	GPIOA->BRR = LL_GPIO_PIN_11;
#endif

#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
	MX_IWDG_Init();
	WDT->cmd = WDT_CMD_RELOAD;
	inputSet = 1;
	armed = 1;
	adjusted_input = 48;
	newinput = 48;

#else

	zero_input_count = 0;
	MX_IWDG_Init();
	WDT->cmd = WDT_CMD_RELOAD;


#ifdef USE_ADC_INPUT
	armed_count_threshold = 5000;
	inputSet = 1;

#else
	adc_counter = 8;

	UN_TIM_Init();
	receiveDshotDma();

#endif

#endif // end fixed duty mode ifdef

	adc_counter = 9;

	while (1)
	{
		WDT->cmd = WDT_CMD_RELOAD;

		adc_counter++;
		if (adc_counter > 200)
		{ // for testing adc and telemetry

			ADC_raw_temp = ADC_raw_temp - (temperature_offset);
			converted_degrees = (12600 - (int32_t)ADC_raw_temp * 33000 / 4096) / -42 + 25;
			degrees_celsius = (7 * degrees_celsius + converted_degrees) >> 3;
			battery_voltage = ((7 * battery_voltage) + ((ADC_raw_volts * 3300 / 4095 * VOLTAGE_DIVIDER) / 100)) >> 3;
			smoothed_raw_current = ((7 * smoothed_raw_current + (ADC_raw_current)) >> 3);
			actual_current = (smoothed_raw_current * 3300 / 41) / (MILLIVOLT_PER_AMP) + CURRENT_OFFSET;

			adc_ordinary_software_trigger_enable(ADC1, TRUE);

			if (LOW_VOLTAGE_CUTOFF)
			{
				if (battery_voltage < (cell_count * low_cell_volt_cutoff))
				{
					low_voltage_count++;
					if (low_voltage_count > (1000))
					{
						input = 0;
						allOff();
						maskPhaseInterrupts();
						running = 0;
						zero_input_count = 0;
						armed = 0;
					}
				}
				else
				{
					low_voltage_count = 0;
				}
			}
			adc_counter = 0;
#ifdef USE_ADC_INPUT
			if (ADC_raw_input < 10)
			{
				zero_input_count++;
			}
			else
			{
				zero_input_count = 0;
			}
#endif
		}
#ifdef USE_ADC_INPUT

		signaltimeout = 0;
		ADC_smoothed_input = (((10 * ADC_smoothed_input) + ADC_raw_input) / 11);
		newinput = ADC_smoothed_input / 2;
		if (newinput > 2000)
		{
			newinput = 2000;
		}
#endif
		stuckcounter = 0;

		if (bi_direction == 1 && dshot == 0)
		{
			if (RC_CAR_REVERSE)
			{
				if (newinput > (1000 + (servo_dead_band << 1)))
				{
					if (forward == dir_reversed)
					{
						adjusted_input = 0;
						if (running)
						{
							prop_brake_active = 1;
						}
						else
						{
							forward = 1 - dir_reversed;
						}
					}
					if (prop_brake_active == 0)
					{
						adjusted_input = map(newinput, 1000 + (servo_dead_band << 1), 2000, 47, 2047);
					}
				}
				if (newinput < (1000 - (servo_dead_band << 1)))
				{
					if (forward == (1 - dir_reversed))
					{
						if (running)
						{
							prop_brake_active = 1;
						}
						else
						{
							forward = dir_reversed;
						}
						adjusted_input = 0;
					}
					if (prop_brake_active == 0)
					{
						adjusted_input = map(newinput, 0, 1000 - (servo_dead_band << 1), 2047, 47);
					}
				}

				if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band << 1)))
				{
					adjusted_input = 0;
					prop_brake_active = 0;
				}
			}
			else
			{
				if (newinput > (1000 + (servo_dead_band << 1)))
				{
					if (forward == dir_reversed)
					{
						if (commutation_interval > reverse_speed_threshold)
						{
							forward = 1 - dir_reversed;
							zero_crosses = 0;
							old_routine = 1;
							maskPhaseInterrupts();
							brushed_direction_set = 0;
						}
						else
						{
							newinput = 1000;
						}
					}
					adjusted_input = map(newinput, 1000 + (servo_dead_band << 1), 2000, 47, 2047);
				}
				if (newinput < (1000 - (servo_dead_band << 1)))
				{
					if (forward == (1 - dir_reversed))
					{
						if (commutation_interval > reverse_speed_threshold)
						{
							zero_crosses = 0;
							old_routine = 1;
							forward = dir_reversed;
							maskPhaseInterrupts();
							brushed_direction_set = 0;
						}
						else
						{
							newinput = 1000;
						}
					}
					adjusted_input = map(newinput, 0, 1000 - (servo_dead_band << 1), 2047, 47);
				}

				if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band << 1)))
				{
					adjusted_input = 0;
					brushed_direction_set = 0;
				}
			}
		}
		else if (dshot && bi_direction)
		{
			if (newinput > 1047)
			{

				if (forward == dir_reversed)
				{
					if (commutation_interval > reverse_speed_threshold )
					{
						forward = 1 - dir_reversed;
						zero_crosses = 0;
						old_routine = 1;
						maskPhaseInterrupts();
						brushed_direction_set = 0;
					}
					else
					{
						newinput = 0;
					}
				}
				adjusted_input = ((newinput - 1048) * 2 + 47) - reversing_dead_band;
			}
			if (newinput <= 1047 && newinput > 47)
			{
				//	startcount++;

				if (forward == (1 - dir_reversed))
				{
					if (commutation_interval > reverse_speed_threshold )
					{
						zero_crosses = 0;
						old_routine = 1;
						forward = dir_reversed;
						maskPhaseInterrupts();
						brushed_direction_set = 0;
					}
					else
					{
						newinput = 0;
					}
				}
				adjusted_input = ((newinput - 48) * 2 + 47) - reversing_dead_band;
			}
			if (newinput < 48)
			{
				adjusted_input = 0;
				brushed_direction_set = 0;
			}
		}
		else
		{
			adjusted_input = newinput;
		}
		
		
		if ((zero_crosses > 1000) || (adjusted_input == 0))
		{
			bemf_timeout_happened = 0;
#ifdef USE_RGB_LED
			if (adjusted_input == 0 && armed)
			{
				GPIOB->BSRR = LL_GPIO_PIN_8; // off red
				GPIOB->BRR = LL_GPIO_PIN_5;	 // on green
				GPIOB->BSRR = LL_GPIO_PIN_3; // off blue
			}
#endif
		}
		if (zero_crosses > 100 && adjusted_input < 200)
		{
			bemf_timeout_happened = 0;
		}

		if (crawler_mode)
		{
			if (adjusted_input < 400)
			{
				bemf_timeout_happened = 0;
			}
		}
		else
		{
			if (adjusted_input < 150)
			{ // startup duty cycle should be low enough to not burn motor
				bemf_timeout = 100;
			}
			else
			{
				bemf_timeout = 10;
			}
		}
		if (bemf_timeout_happened > bemf_timeout * (1 + (crawler_mode * 100)) && stuck_rotor_protection)
		{
			allOff();
			maskPhaseInterrupts();
			input = 0;
			bemf_timeout_happened = 102;
#ifdef USE_RGB_LED
			GPIOB->BRR = LL_GPIO_PIN_8;	 // on red
			GPIOB->BSRR = LL_GPIO_PIN_5; //
			GPIOB->BSRR = LL_GPIO_PIN_3;
#endif
		}
		else
		{
#ifdef FIXED_DUTY_MODE
			input = FIXED_DUTY_MODE_POWER * 20;
#else
			input = adjusted_input;	
#endif
		}
		
		

		e_rpm = running * (100000 / e_com_time) * 6; // in tens of rpm
		k_erpm = e_rpm / 10;						 // ecom time is time for one electrical revolution in microseconds

		if (low_rpm_throttle_limit)
		{ // some hardware doesn't need this, its on by default to keep hardware / motors protected but can slow down the response in the very low end a little.
			duty_cycle_maximum = map(k_erpm, low_rpm_level, high_rpm_level, throttle_max_at_low_rpm, throttle_max_at_high_rpm); // for more performance lower the high_rpm_level, set to a consvervative number in source.
		}

		if (degrees_celsius > TEMPERATURE_LIMIT)
		{
			duty_cycle_maximum = map(degrees_celsius, TEMPERATURE_LIMIT, TEMPERATURE_LIMIT + 20, throttle_max_at_high_rpm / 2, 1);
		}

		if (zero_crosses < 100 || commutation_interval > 500)
		{
			filter_level = 12;
		}
		else
		{
			filter_level = map(average_interval, 100, 500, 4, 12);
		}
		if (commutation_interval < 100)
		{
			filter_level = 2;
		}

		if (motor_kv < 900)
		{
			filter_level = filter_level * 2;
		}

		/**************** old routine*********************/
		if (old_routine && running)
		{
			maskPhaseInterrupts();
			if (!zcfound)
			{
				getBemfState();
				if (rising)
				{
					if (bemfcounter > min_bemf_counts_up)
					{
						zcfound = 1;
						//zcfoundroutine();
						zcfoundroutine_nonblock();
					}
				}
				else
				{
					if (bemfcounter > min_bemf_counts_down)
					{
						zcfound = 1;
						//zcfoundroutine();
						zcfoundroutine_nonblock();
					}
				}
			}
		}
		if (INTERVAL_TIMER->cval > 45000 && running == 1)
		{
			bemf_timeout_happened++;
			maskPhaseInterrupts();
			old_routine = 1;
			if (input < 48)
			{
				running = 0;
			}
			zero_crosses = 0;
			//zcfoundroutine();
			zcfoundroutine_nonblock();

		}
	

	}
}
