#include <arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//#define DEBUG_MODE

enum pins_out
{
    m1d = 2,
    m1p,
    m2d,
    m2p,
    m3p,
    m3d,
    m4d,
    m4p,
    ind1 = 12,
    ind2,
    motor_en = 10,
    led = 11
};

enum pins_in
{
    cs1 = A0,
    cs2 = A1,
    cs3 = A2,
    cs4 = A3,
    prs = A6,
    vin = A7
};

typedef struct  //this thing is 34B
{
	uint8_t header;
	float vin;
	float current1;
	float current2;
	float current3;
	float current4;
	float roll;
	float pitch;
	float pressure;
	uint8_t temp;

} telem_packet;

#define BNO055_SAMPLERATE_DELAY_MS (50)
#define vperdiv 0.0049


Adafruit_BNO055 bno = Adafruit_BNO055();
telem_packet telemetry;

uint8_t headlight_state = 0;
uint8_t motor_sleep_state = 0;

void setup(void)
{
	Serial.begin(9600);

	/* Initialise the sensor */
	if(!bno.begin())
	{
		while(1);
	}

	delay(BNO055_SAMPLERATE_DELAY_MS);

	for(int i = 2; i < 13; i++)
	{
		pinMode(i, OUTPUT);
	}

	pinMode(cs1, INPUT);
	pinMode(cs2, INPUT);
	pinMode(cs3, INPUT);
	pinMode(cs4, INPUT);
	pinMode(prs, INPUT);
	pinMode(vin, INPUT);
	bno.setExtCrystalUse(true);
}
void loop(void)
{
	checkInput();
	getSensorData();
	
#ifdef DEBUG_MODE
	printTelemetry();
#endif
	
	writePacket();
	digitalWrite(led, headlight_state);
	digitalWrite(motor_en, motor_sleep_state);
	
	updateVoltageIndicator();
	
	delay(BNO055_SAMPLERATE_DELAY_MS);
}

void updateVoltageIndicator()
{
	if(telemetry.vin<=10 && telemetry.vin>9.5)
	{
		blinkLedAtSpeed(2000);
	}
	else if (telemetry.vin<=9.5 && telemetry.vin>9)
	{
		blinkLedAtSpeed(250);
	}
	else if (telemetry.vin<=9)
	{
		blinkLedAtSpeed(100);
	}
	else
	{
		blinkLedAtSpeed(0);
	}
}

int led_state = 0;
unsigned long last = 0;
void blinkLedAtSpeed(int interval)
{
	if(interval ==0)
	{
		digitalWrite(ind2, LOW);
		return;
	}
	if(millis() - last >=interval)
	{
		led_state = !led_state;
		last = millis();
	}
	
	digitalWrite(ind2,led_state);
}


void checkInput()
{
	if(Serial.available() > 10)
	{
		digitalWrite(ind1, HIGH);
		int currentMotor = 0;
		for(int i = 0; i < 12 ; i++)
		{
			int in = Serial.read();

			if(in == 127)
			{
				currentMotor = 1;
				if(Serial.available() > 4)
				{
					for(int j = 0; j < 4; j++)
					{
						setMotor(currentMotor, Serial.read());
						currentMotor++;
					}
					process_flags(Serial.read());
				}
			}
		}
		while(Serial.available())
		{
			Serial.read();
		}
		digitalWrite(ind1, LOW);
	}
}

void setMotor(int m, int p)
{

	p -= 50;

	switch (m)
	{
		case 1:
		{
			analogWrite(m1p, abs(p * 255 / 50));
			if(p < 0)
			{
				digitalWrite(m1d, HIGH);
			}
			else
			{
				digitalWrite(m1d, LOW);
			}
			break;
		}
		case 2:
		{
			analogWrite(m2p, abs(p * 255 / 50));
			if(p < 0)
			{
				digitalWrite(m2d, HIGH);
			}
			else
			{
				digitalWrite(m2d, LOW);
			}
			break;
		}
		case 3:
		{
			analogWrite(m3p, abs(p * 255 / 50));
			if(p < 0)
			{
				digitalWrite(m3d, LOW);
			}
			else
			{
				digitalWrite(m3d, HIGH);
			}
			break;
		}
		case 4:
		{
			analogWrite(m4p, abs(p * 255 / 50));
			if(p < 0)
			{
				digitalWrite(m4d, LOW);
			}
			else
			{
				digitalWrite(m4d, HIGH);
			}
			break;
		}
	}
}


void getSensorData()//0 = temp, 1=prs, 2=y, 3=z
{
	imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

	telemetry.header = 255;

	telemetry.temp = bno.getTemp();
	telemetry.pressure = getPressure();
	telemetry.roll = euler.y();
	telemetry.pitch = euler.z();
	telemetry.vin = readVoltage();

	telemetry.current1 = get_current(cs1);
	telemetry.current2 = get_current(cs2);
	telemetry.current3 = get_current(cs3);
	telemetry.current4 = get_current(cs4);

}

float get_current(int pin)
{
	if(motor_sleep_state==1)
	{
		return 0;
	}
	int val = analogRead(pin);
	float voltage = val * vperdiv;
	float current = (voltage - 0.05) / 0.02;
	return current;
}

float readVoltage()
{
	int val = analogRead(A7);
	float volt_pin = val * vperdiv;
	float volt_supply = volt_pin * 3;
	return volt_supply;
}

void process_flags(uint8_t input)
{
	//byte one is motor status
	//byte two is headlight status

	headlight_state = (input >> 0) & 0x01;
	motor_sleep_state = (input >> 1) & 0x01;
}

void writePacket()
{
	/*
	uint8_t* buffer;
	buffer = (uint8_t*)malloc(sizeof(telem_packet));
	memcpy(buffer, &telemetry, sizeof(telem_packet));
	*/

	for(int i = 1; i < sizeof(telem_packet); i++)
	{
		if(((int*)(&telemetry))[i] == 255)
		{
			((int*)(&telemetry))[i] = 254;
		}
	}
#ifdef DEBUG_MODE
	printArray(((uint8_t*)(&telemetry)),sizeof(telem_packet));
#else
	Serial.write(((uint8_t*)(&telemetry)), sizeof(telem_packet));
#endif
}

void printTelemetry()
{
	Serial.println("telemetry data:");
	Serial.print("\t vin:");
	Serial.println(telemetry.vin);
	Serial.print("\t c1:");
	Serial.println(telemetry.current1);
	Serial.print("\t c2:");
	Serial.println(telemetry.current2);
	Serial.print("\t c3:");
	Serial.println(telemetry.current3);
	Serial.print("\t c4:");
	Serial.println(telemetry.current4);
	Serial.print("\t roll:");
	Serial.println(telemetry.roll);
	Serial.print("\t pitch:");
	Serial.println(telemetry.pitch);
	Serial.print("\t pressure:");
	Serial.println(telemetry.pressure);
	Serial.print("\t temperature:");
	Serial.println(telemetry.temp);
}

void printArray(uint8_t* b, int len)
{
	for(int i = 0; i < len; i++)
	{
		Serial.print(i);
		Serial.print("\t");
		Serial.println(b[i]);
	}
}

float getPressure()
{
	int val = analogRead(prs);
	float volt = val * vperdiv;
	return volt;
}

void testPacket()
{
	Serial.write(255);
	for(int i = 0; i < 5; i++)
	{
		Serial.write(i);
	}
}