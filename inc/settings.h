
void loadEEpromSettings()
{
	read_flash_bin(eepromBuffer, EEPROM_START_ADD, 176);

	if (eepromBuffer[17] == 0x01)
	{
		dir_reversed = 1;
	}
	else
	{
		dir_reversed = 0;
	}
	if (eepromBuffer[18] == 0x01)
	{
		bi_direction = 1;
	}
	else
	{
		bi_direction = 0;
	}
	if (eepromBuffer[19] == 0x01)
	{
		//use_sin_start = 1;
		//	 min_startup_duty = sin_mode_min_s_d;
	}
	if (eepromBuffer[20] == 0x01)
	{
		comp_pwm = 1;
	}
	else
	{
		comp_pwm = 0;
	}
	if (eepromBuffer[21] == 0x01)
	{
		VARIABLE_PWM = 1;
	}
	else
	{
		VARIABLE_PWM = 0;
	}
	if (eepromBuffer[22] == 0x01)
	{
		stuck_rotor_protection = 1;
	}
	else
	{
		stuck_rotor_protection = 0;
	}
	if (eepromBuffer[23] < 4)
	{
		advance_level = eepromBuffer[23];
	}
	else
	{
		advance_level = 2; // * 7.5 increments
	}

	if (eepromBuffer[24] < 49 && eepromBuffer[24] > 7)
	{
		if (eepromBuffer[24] < 49 && eepromBuffer[24] > 23)
		{
			TIMER1_MAX_ARR = map(eepromBuffer[24], 24, 144, TIM1_AUTORELOAD, TIM1_AUTORELOAD / 6);
		}
		if (eepromBuffer[24] < 24 && eepromBuffer[24] > 11)
		{
			TIMER1_MAX_ARR = map(eepromBuffer[24], 12, 24, TIM1_AUTORELOAD * 2, TIM1_AUTORELOAD);
		}
		if (eepromBuffer[24] < 12 && eepromBuffer[24] > 7)
		{
			TIMER1_MAX_ARR = map(eepromBuffer[24], 7, 16, TIM1_AUTORELOAD * 3, TIM1_AUTORELOAD / 2 * 3);
		}
		TMR1->pr = TIMER1_MAX_ARR;
		throttle_max_at_high_rpm = TIMER1_MAX_ARR;
		duty_cycle_maximum = TIMER1_MAX_ARR;
	}
	else
	{
		tim1_arr = TIM1_AUTORELOAD;
		TMR1->pr = tim1_arr;
	}

	if (eepromBuffer[25] < 151 && eepromBuffer[25] > 49)
	{
		min_startup_duty = (eepromBuffer[25] / 2 + DEAD_TIME) * TIMER1_MAX_ARR / 2000;
		minimum_duty_cycle = (eepromBuffer[25] / 4 + DEAD_TIME / 4) * TIMER1_MAX_ARR / 2000;
		stall_protect_minimum_duty = minimum_duty_cycle + 10;
	}
	else
	{
		min_startup_duty = 150;
		minimum_duty_cycle = (min_startup_duty / 2) + 10;
	}
	motor_kv = (eepromBuffer[26] * 40) + 20;
	motor_poles = eepromBuffer[27];
	if (eepromBuffer[28] == 0x01)
	{
		brake_on_stop = 1;
	}
	else
	{
		brake_on_stop = 0;
	}
	if (eepromBuffer[29] == 0x01)//NOT SUPPORTED
	{
		//stall_protection = 1;
	}
	else
	{
		//stall_protection = 0;
	}
	setVolume(5);
	if (eepromBuffer[1] > 0)
	{ // these commands weren't introduced until eeprom version 1.

		if (eepromBuffer[30] > 11)
		{
			setVolume(5);
		}
		else
		{
			setVolume(eepromBuffer[30]);
		}
		if (eepromBuffer[31] == 0x01)
		{
			TLM_ON_INTERVAL = 1;
		}
		else
		{
			TLM_ON_INTERVAL = 0;
		}
		servo_low_threshold = (eepromBuffer[32] * 2) + 750; // anything below this point considered 0
		servo_high_threshold = (eepromBuffer[33] * 2) + 1750;
		; // anything above this point considered 2000 (max)
		servo_neutral = (eepromBuffer[34]) + 1374;
		servo_dead_band = eepromBuffer[35];

		if (eepromBuffer[36] == 0x01)
		{
			LOW_VOLTAGE_CUTOFF = 1;
		}
		else
		{
			LOW_VOLTAGE_CUTOFF = 0;
		}

		low_cell_volt_cutoff = eepromBuffer[37] + 250; // 2.5 to 3.5 volts per cell range
		if (eepromBuffer[38] == 0x01)
		{
			RC_CAR_REVERSE = 1;
		}
		else
		{
			RC_CAR_REVERSE = 0;
		}
		if (eepromBuffer[39] == 0x01)
		{
#ifdef HAS_HALL_SENSORS
			USE_HALL_SENSOR = 1;
#else
			USE_HALL_SENSOR = 0;
#endif
		}
		else
		{
			USE_HALL_SENSOR = 0;
		}
		if (eepromBuffer[40] > 4 && eepromBuffer[40] < 26)
		{ // sine mode changeover 5-25 percent throttle
			//sine_mode_changeover_thottle_level = eepromBuffer[40];
		}
		if (eepromBuffer[41] > 0 && eepromBuffer[41] < 11)
		{ // drag brake 1-10
			drag_brake_strength = eepromBuffer[41];
		}

		if (eepromBuffer[42] > 0 && eepromBuffer[42] < 10)
		{ // motor brake 1-9
			driving_brake_strength = eepromBuffer[42];
			dead_time_override = DEAD_TIME + (150 - (driving_brake_strength * 10));
			if (dead_time_override > 200)
			{
				dead_time_override = 200;
			}
			min_startup_duty = eepromBuffer[25] + dead_time_override;
			minimum_duty_cycle = eepromBuffer[25] / 2 + dead_time_override;
			throttle_max_at_low_rpm = throttle_max_at_low_rpm + dead_time_override;
			startup_max_duty_cycle = startup_max_duty_cycle + dead_time_override;
			//	 TIMER_CCHP(TIMER0)|= TIMER_CCHP_DTCFG & dead_time_override;
		}

		if (eepromBuffer[43] >= 70 && eepromBuffer[43] <= 140)
		{
			TEMPERATURE_LIMIT = eepromBuffer[43];
		}

		if (eepromBuffer[44] > 0 && eepromBuffer[44] < 100)
		{
			CURRENT_LIMIT = eepromBuffer[44] * 2;
			use_current_limit = 1;
		}
		if (eepromBuffer[45] > 0 && eepromBuffer[45] < 11)
		{
			//sine_mode_power = eepromBuffer[45];
		}

		if (motor_kv < 300)
		{
			low_rpm_throttle_limit = 0;
		}
		low_rpm_level = motor_kv / 100 / (32 / motor_poles);
		high_rpm_level = motor_kv / 17 / (32 / motor_poles);
	}
	reverse_speed_threshold = map(motor_kv, 300, 3000, 2500, 1250);
	if (!comp_pwm)
	{
		bi_direction = 0;
	}
}

void saveEEpromSettings()
{

	eepromBuffer[1] = eeprom_layout_version;
	if (dir_reversed == 1)
	{
		eepromBuffer[17] = 0x01;
	}
	else
	{
		eepromBuffer[17] = 0x00;
	}
	if (bi_direction == 1)
	{
		eepromBuffer[18] = 0x01;
	}
	else
	{
		eepromBuffer[18] = 0x00;
	}
	if (0/*use_sin_start == 1*/)
	{
		eepromBuffer[19] = 0x01;
	}
	else
	{
		eepromBuffer[19] = 0x00;
	}

	if (comp_pwm == 1)
	{
		eepromBuffer[20] = 0x01;
	}
	else
	{
		eepromBuffer[20] = 0x00;
	}
	if (VARIABLE_PWM == 1)
	{
		eepromBuffer[21] = 0x01;
	}
	else
	{
		eepromBuffer[21] = 0x00;
	}
	if (stuck_rotor_protection == 1)
	{
		eepromBuffer[22] = 0x01;
	}
	else
	{
		eepromBuffer[22] = 0x00;
	}
	eepromBuffer[23] = advance_level;
	save_flash_nolib(eepromBuffer, 176, EEPROM_START_ADD);
}
