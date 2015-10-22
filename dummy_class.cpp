//dummy change to check the git - del this line if you read this.!.
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <typeinfo>
#include <time.h>


using namespace std;

//#define SLAVE_ADDRESS 0x14

class MotorIO {
	// private by default unless specified.
	int file;
	const char* i2cDevName;
	unsigned char bufferTx[10];
	unsigned char bufferRx[10];
	__u8 instr, cmd;

	public:
		uint8_t motors_array[12];
		// default constructor
		MotorIO();
		// members
		void start();
		void stop();
		void ping();
		__u8* ping(__u8 arr[]);
		void set_gain(int, char,  float);
		float get_gain(int, char);
		void set_position(int, float);
		void set_speed(int, float);
		void set_load(int, float);
		float get_position(int);
		float get_speed(int);
		float get_load(int);
		void readEEPROM(int, __u8);
		void writeEEPROM(int, __u8, float);
};

MotorIO::MotorIO() {
	//cout << "Class constructed" << endl;
	i2cDevName = "/dev/i2c-2";
}

void MotorIO::start() {

	file = open(i2cDevName, O_RDWR);
	if (file<0) {
		fprintf(stderr, "Bad device name %s\r\n", i2cDevName);
		exit(1);
	}else {
		printf("*** Device '%s' opened succesfully. \r\n", i2cDevName);
	}

}

void MotorIO::stop() {

	close(file);
	printf("*** Device '%s' closed succesfully.\r\n", i2cDevName);

}

void MotorIO::ping() {
	__u8 idx = 0;
	printf("Scanning for motors in '%s'...\r\n", i2cDevName);
	for (int i=0x00;i<0x20;i++) {

		if (ioctl(file, I2C_SLAVE, i)<0)
			printf("Nop");

		bufferTx[0] = 0xf0;
		if (write(file, bufferTx, 1) !=1) {
			//printf("Couldn't ask motor with id '%x'. \r\n",i);
			//exit(1);
		}else {
			if (read(file, bufferRx, 1)!=1) {
				printf("No response from '%x' \r\n", i);
				exit(1);
			}else{
				if (bufferRx[0] == i){
					printf("~~~ Found motor at address '%x' ~~~\r\n",i);
					motors_array[idx] = i;
					idx++;
				}
			}
		}
	}
	printf ("Found total %x motors.\r\n",idx);
}

__u8* MotorIO::ping(__u8 arr[]) {

	__u8 idx = 0;
	for (int i=0x00;i<0x20;i++) {
                if (ioctl(file, I2C_SLAVE, i)<0)
                        printf("Nop");

                bufferTx[0] = 0xff;
                if (write(file, bufferTx, 1) !=1) {
                        printf("Couldn't ask motor with id '%x'. \r\n",i);
                        exit(1);
                }
                // read response
                if (read(file, bufferRx, 1)!=1) {
                        printf("No response from '%x' \r\n", i);
                        exit(1);
                }else{
                        if (bufferRx[0] == i){
                                //printf("~~~~ Found motor at address '%x'.~~~\r\n",i);
                                arr[idx] = i;
                                idx++;
                        }
                }
        }

}


/* -------------------------------------------- SET/GET GAINS ------------------------------------------------- */

void MotorIO::set_gain(int servo_id, char gainID, float gain) {

	// Handshake with the servo
	if (ioctl(file, I2C_SLAVE, servo_id)<0) {
		fprintf(stderr, "Failed to acquire bus acces to '%x' and.or talk to motor_id\r\n", servo_id);
		exit(1);
	} else {
		//printf("*** Acquired bus access to a slave device adr: '%x'.\r\n", servo_id);
	}

	float *p = (float *)&bufferTx[2];
	instr = 0x03;
	cmd = 0;

	if (gainID == 'P')
		cmd = 0x01;
	else if (gainID == 'I')
		cmd = 0x02;
	else if (gainID == 'D')
		cmd = 0x03;
	else {
		fprintf(stderr, "Error, choose 'P', 'I' or 'D' gain update\r\n");
		exit(1);
	}

	// -WORD- construction
	bufferTx[0] = instr;
	bufferTx[1] = cmd;
	*p = gain;
	// -WORD- send
        if (write(file, bufferTx, 6)!= 6) {
                fprintf(stderr, "Failed to demand a buffer from Arduino. \r\n");
                exit(1);
        }
}


float MotorIO::get_gain(int servo_id, char gainID){

	//Handshake with the servo
	if (ioctl(file, I2C_SLAVE, servo_id)<0) {
		fprintf(stderr, "Failed to acquire bus acces to '%x' and/or talk to motor_id\r\n", servo_id);
		exit(1);
	} else {
		//printf("*** Acquired access to motor with address: '%x'.\r\n", servo_id);
	}

	instr = 0x03;

	if (gainID == 'P')
		cmd = 0x04;
	else if (gainID == 'I')
		cmd = 0x05;
	else if (gainID == 'D')
		cmd = 0x06;
	else {
		fprintf(stderr, "Error, choose 'P', 'I' or 'D' gain update\r\n");
		exit(1);
	}

	// -WORD- construction
	bufferTx[0] = instr;
	bufferTx[1] = cmd;
	// Request
	if (write(file, bufferTx, 2)!=2) {
		fprintf(stderr, "Failed to demand a buffer from Arduino.\r\n");
		exit(1);
	}

	// Read the response
        float *ret = (float *)&bufferRx[2];
        if (read(file, bufferRx, 6) != 6) {
                fprintf(stderr, "Failed to read from the i2c bus.\r\n");
                exit(1);
        } else {
		return *ret;
        }

}

/* ------------------------------------------- ~ SET/GET GAINS ~ ------------------------------------------------ */
/* -------------------------------------------------------------------------------------------------------------- */




/* --------------------------------------------- SET/GET GOALS -------------------------------------------------- */
void MotorIO::set_position(int servo_id, float pos) {

	// Handshake with the servo
        if (ioctl(file, I2C_SLAVE, servo_id)<0) {
                fprintf(stderr, "Failed to acquire bus acces to '%x' and/or talk to motor \r\n", servo_id);
		exit(1);
        } else {
                //printf("*** Acquired bus access to a slave device adr: '%x'.\r\n", servo_id);
        }

	instr = 0x04;
	cmd = 0x01;
	// -WORD- construction
	bufferTx[0] = instr;
	bufferTx[1] = cmd;
	float *p = (float *)&bufferTx[2];
	*p = pos;
	// -WORD- send
	if (write(file, bufferTx, 6) != 6) {
		printf("Failed to access the motor");
                exit(1);
        }

}


void MotorIO::set_speed(int servo_id, float spd) {

        // Handshake with the servo
        if (ioctl(file, I2C_SLAVE, servo_id)<0) {
                fprintf(stderr, "Failed to acquire bus acces to '%x' and/or talk to motor \r\n", servo_id);
                exit(1);
        } else {
                printf("*** Acquired bus access to a slave device adr: '%x'.\r\n", servo_id);
        }

        instr = 0x04;
        cmd = 0x02;
        bufferTx[0] = instr;
        bufferTx[1] = cmd;
        float *p = (float *)&bufferTx[2];
        *p = spd;
        if (write(file, bufferTx, 6) != 6) {
                fprintf(stderr, "Failed to access the motor with id '%x'.\r\n", servo_id);
                exit(1);
        }

}


void MotorIO::set_load(int servo_id, float load) {

        // Handshake with the servo
        if (ioctl(file, I2C_SLAVE, servo_id)<0) {
                fprintf(stderr, "Failed to acquire bus acces to '%x' and/or talk to motor \r\n", servo_id);
                exit(1);
        } else {
                printf("*** Acquired bus access to a slave device adr: '%x'.\r\n", servo_id);
        }

        instr = 0x04;
        cmd = 0x03;
        bufferTx[0] = instr;
        bufferTx[1] = cmd;
        float *p = (float *)&bufferTx[2];
        *p = load;
        if (write(file, bufferTx, 6) != 6) {
                fprintf(stderr, "Failed to access the motor with id '%x'.\r\n", servo_id);
                exit(1);
        }

}


float MotorIO::get_position(int servo_id) {

	// Handshake with the servo
        if (ioctl(file, I2C_SLAVE, servo_id)<0) {
                fprintf(stderr, "Failed to acquire bus acces to '%x' and/or talk to motor \r\n", servo_id);
                exit(1);
        } else {
                //printf("*** Acquired bus access to a slave device adr: '%x'.\r\n", servo_id);
        }

	instr = 0x04;
	cmd = 0x05;

        // -WORD- construction
        bufferTx[0] = instr;
        bufferTx[1] = 0x05;
        // Request
        if (write(file, bufferTx, 2)!=2) {
                fprintf(stderr, "Failed to demand a buffer from Arduino.\r\n");
                exit(1);
        }

        // Read the response
        float *ret = (float *)&bufferRx[2];
        if (read(file, bufferRx, 6) != 6) {
                fprintf(stderr, "Failed to read from the i2c bus.\r\n");
                exit(1);
        } else {
                return *ret;
        }

}


float MotorIO::get_speed(int servo_id) {

        // Handshake with the servo
        if (ioctl(file, I2C_SLAVE, servo_id)<0) {
                fprintf(stderr, "Failed to acquire bus acces to '%x' and/or talk to motor \r\n", servo_id);
                exit(1);
        } else {
                printf("*** Acquired bus access to a slave device adr: '%x'.\r\n", servo_id);
        }

        instr = 0x04;
        cmd = 0x06;

        // -WORD- construction
        bufferTx[0] = instr;
        bufferTx[1] = cmd;
        // Request
        if (write(file, bufferTx, 2)!=2) {
                fprintf(stderr, "Failed to demand a buffer from Arduino.\r\n");
                exit(1);
        }

        // Read the response
        float *ret = (float *)&bufferRx[2];
        if (read(file, bufferRx, 6) != 6) {
                fprintf(stderr, "Failed to read from the i2c bus.\r\n");
                exit(1);
        } else {
                return *ret;
        }

}


float MotorIO::get_load(int servo_id) {

        // Handshake with the servo
        if (ioctl(file, I2C_SLAVE, servo_id)<0) {
                fprintf(stderr, "Failed to acquire bus acces to '%x' and/or talk to motor \r\n", servo_id);
                exit(1);
        } else {
                printf("*** Acquired bus access to a slave device adr: '%x'.\r\n", servo_id);
        }

        instr = 0x04;
        cmd = 0x07;

        // -WORD- construction
        bufferTx[0] = instr;
        bufferTx[1] = cmd;
        // Request
        if (write(file, bufferTx, 2)!=2) {
                fprintf(stderr, "Failed to demand a buffer from Arduino.\r\n");
                exit(1);
        }

        // Read the response
        float *ret = (float *)&bufferRx[2];
        if (read(file, bufferRx, 6) != 6) {
                fprintf(stderr, "Failed to read from the i2c bus.\r\n");
                exit(1);
        } else {
                return *ret;
        }

}

/* ------------------------------------------- ~ SET/GET GOALS ~ ------------------------------------------------ */
/* -------------------------------------------------------------------------------------------------------------- */




/* -------------------------------------------  EEPROM commands ------------------------------------------------- */

void MotorIO::writeEEPROM(int servo_id, __u8 address, float val) {
        if (ioctl(file, I2C_SLAVE, servo_id)<0) {
                fprintf(stderr, "Failed to acquire bus access to '%x' and/or talk to motor with id: \r\r", servo_id);
                exit(1);
        } else
                printf("*** Acquired bus acces to the motor with id '%x':\r\n", servo_id);

        //char bufferTx[6];
        bufferTx[0] = 0x02;     // 0x02: Instruction => write EEPROM
        bufferTx[1] = address;
        __u8 *p  = (__u8 *)&val;
        for (int i=0; i<4; i++) {
                bufferTx[2+i] = p[i];
        }
        //Tell Arduino what you want to read
        if (write(file, bufferTx, 6)!=6) {
                fprintf(stderr, "Failed to demand a buffer from Arduino.\r\n");
                exit(1);
        }
	usleep(20000);	// Give time to write to EEPROM !!

}

void MotorIO::readEEPROM(int servo_id, __u8 address) {
	if (ioctl(file, I2C_SLAVE, servo_id)<0) {
		fprintf(stderr, "Fialed to acquire bus access to '%x' and/or talk to motor with id:\r\r", servo_id);
		exit(1);
	} else
		printf("*** Acquired bus acces to the motor with id '%x':\r\n", servo_id);

	bufferTx[0] = 0x01;	// 0x01: Instruction => read EEPROM
	bufferTx[1] = address;
	//Tell Arduino what you want to read
	if (write(file, bufferTx, 2)!=2) {
		fprintf(stderr, "Failed to demand a buffer from Arduino.\r\n");
		exit(1);
	}else {
		cout << "Ep ep" << endl;
	}
	//Read bytes from Arduino
	float *ret = (float *)&bufferRx[2];
	if (read(file, bufferRx, 6) !=6 ) {
		fprintf(stderr, "Failed to read from the i2c bus.\r\n");
		exit(1);
	} else {
		cout << "Op Op" << endl;
		//printf("*** Arduino sent back %.2f \r\n", *ret);
		float f = *ret;
		cout << f << endl;
	}
}

/* ------------------------------------------ ~ EEPROM commands ~ ----------------------------------------------- */
/* -------------------------------------------------------------------------------------------------------------- */


int main() {
	MotorIO com;	//constructor
	com.start();
	//com.ping();
	//__u8 motors_array[12];
	//__u8 *p = com.ping(motors_array);
	//printf("Motors array contain '%x' and '%x' \r\n",motors_array[0], motors_array[1]);
	com.set_gain(0x14, 'I', 0.123);
	float p = com.get_gain(0x14,'I');
	cout << "Motor 'i_gain' is: " << p << endl;
	com.set_position(0x14, 20.44);
	float position = com.get_position(0x14);
	cout << "Motor 'position' is: " << position << endl;

	//com.writeEEPROM(0x14,0,2.245);
	//com.readEEPROM(0x14,0);
	com.stop();
}
