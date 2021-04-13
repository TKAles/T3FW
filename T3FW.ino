/*
 Name:		T3FW.ino
 Created:	4/8/2021 6:38:17 PM
 Author:	ThomasAles

 Tip/Tilt/Translate (T3) and global rotation motor firmware.
 Version 0.1.0

*/


// Motor Pin Location Arrays
// Format: {CS, DIR, STEP, LIMIT SWITCH}

#include <ArduinoJson.hpp>
#include <ArduinoJson.h>
#include <TMC2130Stepper_UTILITY.h>
#include <TMC2130Stepper_REGDEFS.h>
#include <TMC2130Stepper.h>

DynamicJsonDocument INPUT_JSON(2048);
const int FW_VER[] = { 0, 1, 0 };
// Motor Pin Arrays
// Format: CS, DIR, STEP, LIMIT SWITCH
const int M0_PIN[] = {6, 9, 33, 31};
const int M1_PIN[] = { 28, 30, 32, 26 };
const int M2_PIN[] = { 14, 15, 29, 24 };
const int M3_PIN[] = { 27, 17, 25, 16 };

const int MOSI_PIN = 11;
const int MISO_PIN = 12;
const int SCK_PIN = 13;

// Global Rotation Motor Encoder Pins
// Format: {Z, A, B, SERIAL RX, SERIAL TX}
const int ENC_PIN[] = { 21, 22, 23, 1, 0 };

// Enable pin. Pull low to enable/lock motors.
const int MOTOR_ENABLE = 10;

// Stepper objects.
TMC2130Stepper M0 = TMC2130Stepper(M0_PIN[0]);
TMC2130Stepper M1 = TMC2130Stepper(M1_PIN[0]);
TMC2130Stepper M2 = TMC2130Stepper(M2_PIN[0]);
TMC2130Stepper M3 = TMC2130Stepper(M3_PIN[0]);
// Stepper count array
int32_t COUNTS[] = { 0, 0, 0, 0 };

void setup() {
	// Initalize serial link. Print header + FW version
	Serial.begin(115200);
	Serial.println("T3 Controller. FW: " + String(FW_VER[0]) 
					+ "." + String(FW_VER[1]) + "." 
					+ String(FW_VER[2]));
	// Pin States
	pinMode(MOSI_PIN, OUTPUT); pinMode(MISO_PIN, INPUT);
	pinMode(SCK_PIN, OUTPUT);
	// Setup Motor Output Enable
	pinMode(MOTOR_ENABLE, OUTPUT);
	digitalWrite(MOTOR_ENABLE, HIGH);
	// Setup Motor Pins
	for (int i = 0; i < 3; i++)
	{
		pinMode(M0_PIN[i], OUTPUT); pinMode(M1_PIN[i], OUTPUT);
		pinMode(M2_PIN[i], OUTPUT); pinMode(M3_PIN[i], OUTPUT);
		digitalWriteFast(M0_PIN[i], LOW); digitalWriteFast(M1_PIN[i], LOW);
		digitalWriteFast(M2_PIN[i], LOW); digitalWriteFast(M3_PIN[i], LOW);
	}
	initalize_motors();
	Serial.println("Ready.");
}

// the loop function runs over and over again until power down or reset
void loop() {
	String recv_str;
	bool parse_error;
	// Listen to the serial link for incoming messages.
	while (Serial.available())
	{
		recv_str = Serial.readStringUntil('\n');
	}
	// If the recieved string isn't null, attempt to decode into a JSON
	// object of the format: keyword: "string", "value": int, "extra": int

	if(recv_str != "")
	{
		deserializeJson(INPUT_JSON, recv_str);
		// If keyword is null, message probably isn't valid. Throw a
		// parse_error flag and listen again.
		if (INPUT_JSON["keyword"].as<String>() == "null")
		{
			parse_error = true;
		}
		else {
			parse_error = false;
		}
	}
	if (parse_error)
	{
		Serial.println("BADCMD");
		parse_error = false;
	}
	else if((recv_str != "") && (parse_error == false)) {
		DynamicJsonDocument _response(256);
		String key = INPUT_JSON["keyword"].as<String>().toLowerCase();
		/*
			enable command.
			value: 0|1, disable/enable the motor enable pin by pulling it low
			extra: not used.
		*/
		if (key == "enable")
		{
			_response["keyword"].set("enable");
			if (INPUT_JSON["value"].as<int>() == 1)
			{
				digitalWrite(MOTOR_ENABLE, LOW);
				_response["value"].set(1);
				_response["extra"].set(0);
			}
			else {
				digitalWrite(MOTOR_ENABLE, HIGH);
				_response["value"].set(0);
				_response["extra"].set(0);
			}
			serializeJson(_response, Serial);
			Serial.print("\n");
		}
		/*
			microstep command.
			value: 0-4. Axes 0-3. 4 - Assign to all T3 motors (1-3).
			extra: 1/microstep
		*/
		else if (key == "microstep")
		{
			_response["keyword"].set("microstep");
			int channel = INPUT_JSON["value"].as<int>();
			int stepsize = INPUT_JSON["extra"].as<uint16_t>();
			// Validate the channel is correct.
			if ((channel <= -1) || (channel >= 5))
			{
				Serial.println("BADCMD");
				return;
			}
			// Assign step size to motor based on value key
			if (channel == 0)
			{
				M0.mres(stepsize);
				delay(5);
				_response["value"].set(0);
				_response["extra"].set(M0.mres());
			}
			else if (channel == 1)
			{
				M1.mres(stepsize);
				_response["value"].set(1);
				_response["extra"].set(M1.mres());
			}
			else if (channel == 2)
			{
				M2.mres(stepsize);
				_response["value"].set(2);
				_response["extra"].set(M2.mres());
			}
			else if (channel == 3)
			{
				M3.mres(stepsize);
				_response["value"].set(3);
				_response["extra"].set(M3.mres());
			}
			else if (channel == 4)
			{
				M1.mres(stepsize);
				M2.mres(stepsize);
				M3.mres(stepsize);
				_response["value"].set(4);
				_response["extra"].set(M3.mres());
			}
			serializeJson(_response, Serial);
		}
		/*
			current command.
			value: 0-4. Axes 0-3. 4 - Assign to all axes
			extra: step current in milliamps
		*/
		else if (key == "current")
		{
			// Pick off the axis and current requested
			int axis = INPUT_JSON["value"].as<uint8_t>();
			int current_req = INPUT_JSON["extra"].as<uint16_t>();
			_response["keyword"].set("current");
			switch (axis) {
			// Set current, build JSON response and read back current
			// from that axis. 
			case 0:
				M0.setCurrent(current_req, 0.11F, 1.0);
				_response["value"].set(0);
				_response["extra"].set(M0.getCurrent());
				serializeJson(_response, Serial);
				Serial.print('\n');
				break;
			case 1:
				M1.setCurrent(current_req, 0.11F, 1.0);
				_response["value"].set(1);
				_response["extra"].set(M1.getCurrent());
				serializeJson(_response, Serial);
				Serial.print('\n');
				break;
			case 2:
				M0.setCurrent(current_req, 0.11F, 1.0);
				_response["value"].set(2);
				_response["extra"].set(M2.getCurrent());
				serializeJson(_response, Serial);
				Serial.print('\n');
				break;
			case 3:
				M0.setCurrent(current_req, 0.11F, 1.0);
				_response["value"].set(3);
				_response["extra"].set(M3.getCurrent());
				serializeJson(_response, Serial);
				Serial.print('\n');
				break;
			// Special case to assign for all motors
			case 4:
				M0.setCurrent(current_req, 0.11F, 1.0);
				M1.setCurrent(current_req, 0.11F, 1.0);
				M2.setCurrent(current_req, 0.11F, 1.0);
				M3.setCurrent(current_req, 0.11F, 1.0);
				_response["value"].set(4);
				_response["extra"].set(M0.getCurrent());
				serializeJson(_response, Serial);
				Serial.print('\n');
				break;
			}
		}
		/*
			move command.
			value: 0-4. Axes 0-3. 4 - Move all T3 motors in lockstep.
			extra: steps to move. Sign determines direction.
				   CW is positive. CCW is negative.
		*/
		else if (key == "move")
		{
			// Pull out the axes requested and the steps required for movement.
			int move_axis = INPUT_JSON["value"].as<int>();
			int steps_req = INPUT_JSON["extra"].as<int>();
			_response["keyword"].set("move");
			switch (move_axis)
			{
			case 0:
				move_motor(M0_PIN[1], M0_PIN[2], steps_req);
				COUNTS[0] += steps_req;
				_response["value"].set(0);
				_response["extra"].set(steps_req);
				serializeJson(_response, Serial);
				Serial.print('\n');
				break;
			case 1:
				move_motor(M1_PIN[1], M1_PIN[2], steps_req);
				COUNTS[1] += steps_req;
				_response["value"].set(1);
				_response["extra"].set(steps_req);
				serializeJson(_response, Serial);
				Serial.print('\n');
				break;
			case 2:
				move_motor(M2_PIN[1], M2_PIN[2], steps_req);
				COUNTS[2] += steps_req;
				_response["value"].set(2);
				_response["extra"].set(steps_req);
				serializeJson(_response, Serial);
				Serial.print('\n');
				break;
			case 3:
				move_motor(M3_PIN[1], M3_PIN[2], steps_req);
				COUNTS[3] += steps_req;
				_response["value"].set(3);
				_response["extra"].set(steps_req);
				serializeJson(_response, Serial);
				Serial.print('\n');
				break;

			// Lockstep T3 Motor Case
			case 4:
				int delay_micro = 10000;
				int _s;
				if (steps_req < 0)
				{
					_s = steps_req * -1;
					digitalWriteFast(M1_PIN[1], LOW);
					digitalWriteFast(M2_PIN[1], LOW);
					digitalWriteFast(M3_PIN[1], LOW);
				}
				else {
					_s = steps_req;
					digitalWriteFast(M1_PIN[1], HIGH);
					digitalWriteFast(M2_PIN[1], HIGH);
					digitalWriteFast(M3_PIN[1], HIGH);
				}
				for (int i = 0; i < _s; i++)
				{
					digitalWriteFast(M1_PIN[2], LOW);
					digitalWriteFast(M2_PIN[2], LOW);
					digitalWriteFast(M3_PIN[2], LOW);
					delayMicroseconds(delay_micro);
					digitalWriteFast(M1_PIN[2], HIGH);
					digitalWriteFast(M2_PIN[2], HIGH);
					digitalWriteFast(M3_PIN[2], HIGH);
					delayMicroseconds(delay_micro);
				}
				_response["value"] = 4;
				_response["extra"] = steps_req;
				serializeJson(_response, Serial);
				Serial.print('\n');
				COUNTS[1] += steps_req;
				COUNTS[2] += steps_req;
				COUNTS[3] += steps_req;
			}
		}
		/*
			zero command.
			zeros out the COUNTS global array
			value: 0-4. Axis to zero. 4 is a shortcut for all axes.
			extra: 0
		*/
		else if (key == "zero")
		{
			if (INPUT_JSON["value"].as<int>() == 4)
			{
				for (int i = 0; i < 4; i++)
				{
					COUNTS[i] = 0;
				}
			}
			else {
				COUNTS[INPUT_JSON["value"].as<int>()] = 0;
			}
			_response["keyword"] = "zero";
			_response["value"] = INPUT_JSON["value"].as<int>();
			_response["extra"] = 0;
			serializeJson(_response, Serial);
			Serial.print('\n');
			return;
			}
		else if (key == "counts")
		{
		_response["keyword"] = "counts";
		_response["value"] = 0;
		for (int i = 0; i < 4; i++)
		{
			_response["extra"].add(COUNTS[i]);
		}
		serializeJson(_response, Serial);
		Serial.print('\n');
		}
		else
		{
		Serial.println("BADCMD");
		}
	}


}
/*
	initalize_motors() - Call begin() and disable stealthChop on all axes.
*/
void initalize_motors()
{
	// Startup the TMC2130Stepper objects
	M0.begin();
	M1.begin();
	M2.begin();
	M3.begin();
	// Disable stealthChop
	M0.stealthChop(true);
	M1.stealthChop(true);
	M2.stealthChop(true);
	M3.stealthChop(true);
	// Set current to 600mA as default, and disable microstepping.
	M0.setCurrent(100, 0.11F, 1.0);
	M1.setCurrent(100, 0.11F, 1.0);
	M2.setCurrent(100, 0.11F, 1.0);
	M3.setCurrent(100, 0.11F, 1.0);
	M0.mres(8);
	M1.mres(8);
	M2.mres(8);
	M3.mres(8);
	
	return;
}

/*
	move_motor(int step_pin, int dir_pin, int steps)
	Moves a motor a given number of steps.
*/
void move_motor(int dir_pin, int step_pin, int steps)
{
	bool fwd_dir = true;
	int delay_in_microseconds = 10000;	// Delay between step toggle in microseconds
	digitalWrite(dir_pin, HIGH);
	// If less than zero, flip the steps positive and toggle forward flag to false
	if (steps < 0)
	{
		steps = steps * -1;
		digitalWrite(dir_pin, LOW);
	}
	// Step requested number of steps
	// TODO: Limit switch awareness.
	for (int i = 0; i < steps; i++)
	{
		digitalWrite(step_pin, LOW);
		delayMicroseconds(delay_in_microseconds);
		digitalWrite(step_pin, HIGH);
		delayMicroseconds(delay_in_microseconds);
	}
	return;
}