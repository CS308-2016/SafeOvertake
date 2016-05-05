#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.c"//included to support lcd prints

#define FCPU 14745600ul //defined here to make sure that program works properly

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

unsigned char safe=0;
unsigned char state=0;
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned int time_required=1000;
unsigned char flag1 = 0;
unsigned char flag2 = 0;
//variables used importantly in the program
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char Front_Sharp_Sensor=0;
unsigned char Front_IR_Sensor=0;
unsigned char Left_Sharp_Sensor=0;
unsigned char Right_Sharp_Sensor=0;
unsigned char Left_IR_Sensor=0;
unsigned char Right_IR_Sensor=0;
unsigned char Full_Left_Sharp_Sensor=0;
unsigned char Full_Right_Sharp_Sensor =0;
unsigned int count = 0;

unsigned char data = 0x38;
unsigned char tx_data;
unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning
//unsigned char output_enable=PORTC & 0x80;
//unsigned char start_conv = PORTC & 0x40;

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}
int turn=-1;

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00;
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to initialize Buzzer
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
	buzzer_pin_config();
}

// Timer 5 initialised in PWM mode for velocity control
// Prescale:64
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:674.988Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionalit to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/

	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}
//function to switch on the buzzer
void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}
//function to switch off the buzzer
void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}


void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{

	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}
//function to set the direction of motor in forward and start the motor
void forward (void)
{
  motion_set (0x06);//0110
}
//function to stop the motor
void stop (void)
{
  motion_set (0x00);
}
//function to move the  bot in left direction
void move_left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x04);
}
//function to move the  bot in right direction
void move_right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x02);
}
void left(void)
{
    PORTA = 0x05;
}
void right(void)
{
    PORTA = 0x0A;
}
void soft_left(void)
{
    PORTA = 0x02;
}
void soft_right(void)
{
    PORTA = 0x04;
}


void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt
}


//ISR for right position encoder
ISR(INT5_vect)
{
 ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}

//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0;
 ShaftCountLeft = 0;

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 stop(); //Stop robot
}

void left_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}

void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}

void soft_left_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}


//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 //UBRR0L = 0x47; //11059200 Hz
//
 UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98; //for reciever
 //UCSR0B = 0x18; //for transmiter

}

void zigbee_init(void){
    DDRE  = DDRE | 0x03;  //Set output of the PORTE pin 0 and 1 as output
    PORTE = PORTE | 0x03; //Enable internal pull-up for PORTE 4 pin

}


//SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
ISR(USART0_RX_vect)
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable
	UDR0 = data; 				//echo data back to PC
	//safe = 0;
	if(data == 0x38) safe = 0;
	if(data == 0x32) safe = 1;
}

// ISR for Data Register empty interrupt
ISR(USART0_UDRE_vect)
{
	UDR0 = tx_data; 				//data that need to be transmitted is transferred to UDRn
    //lcd_string("not done");
    //buzzer_on();
}

// ISR for Transmit complete interrupt.
ISR(USART0_TX_vect)
{
    //lcd_string(" done");
    //buzzer_on();
    _delay_ms(1000);
    buzzer_off();


}
//intialize all the ports of lcd, zigbee et cetera
void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	uart0_init();
	timer5_init();
	sei();   //Enables the global interrupts
}

void my_right_overtake(void){
    int k=0;
    velocity(255,255);
    while(k<670){
        soft_left();
        _delay_ms(1);
        forward();
        _delay_ms(1);
        k++;
    }
    stop();
}

void my_left_overtake(void){
    int k=0;
    velocity(255,255);
    while(k<630){
        soft_right();
        _delay_ms(1);
        forward();
        _delay_ms(1);
        k++;
    }
    stop();
}


void switch_right(){
    my_right_overtake();
    my_left_overtake();
}

void switch_left(){
    my_left_overtake();
    my_right_overtake();
}

void clear_led(void){
    lcd_cursor(2,1);
    lcd_string("                ");
    lcd_cursor(1,1);
}

//Main Function
int main()
{
	init_devices();
	motion_pin_config();
	left_encoder_pin_config();
    right_encoder_pin_config();
    left_position_encoder_interrupt_init();
    right_position_encoder_interrupt_init();
	lcd_set_4bit();
	lcd_init();
	//   velocity(108,125);
	velocity(125,125);
	forward();
/*	velocity(250,255);
	forward();
	switch_right();
	forward();
	_delay_ms(1000);
	switch_left();
	forward();
	_delay_ms(500);
	*/

while(1){
    while(1){
        Front_Sharp_Sensor = ADC_Conversion(11);
        Full_Left_Sharp_Sensor = ADC_Conversion(9);
        print_sensor(1,8,11);
        print_sensor(1,12,9);
        if(Front_Sharp_Sensor>90){
            stop();
            buzzer_on();
           // clear_led();
           // lcd_cursor(2,1);
           // lcd_string("Obstacle Ahead");
        }
        else if(Front_Sharp_Sensor>30){
            buzzer_off();
         //   velocity(108,125);
            velocity(125,125);
            forward();
            if(safe == 1){
                _delay_ms(1000);
                if(safe == 1){
                     switch_right();
                    break;
                }
            }
            /*else{
                clear_led();
                lcd_cursor(2,1);
                lcd_string("Waiting");
            }*/
        }
        else{
            buzzer_off();
            //velocity(250,255);
            velocity(255,255);
            forward();
        }
    }

    while(1){
        Front_Sharp_Sensor = ADC_Conversion(11);
        Full_Left_Sharp_Sensor = ADC_Conversion(9);
        print_sensor(1,8,11);
        print_sensor(1,12,9);


        if(Front_Sharp_Sensor>90){
            stop();
            buzzer_on();
           // clear_led();
           // lcd_cursor(2,1);
           // lcd_string("Obstacle Ahead");
        }
        else{
            //velocity(250,255);
            velocity(255,255);
            forward();
            _delay_ms(100);
        }
        if(Full_Left_Sharp_Sensor<30 && state==0){
                    state = 0;
                }
        else if(Full_Left_Sharp_Sensor<30 && state==1){
            _delay_ms(1000);
            switch_left();
            break;
        }
        else if(Full_Left_Sharp_Sensor>30 && state==0){
            state = 1;
        }

    }
    }
    /*while(1){
        Front_Sharp_Sensor = ADC_Conversion(11);
        print_sensor(1,8,11);
        if(Front_Sharp_Sensor>90){
            stop();
            buzzer_on();
            clear_led();
            lcd_cursor(2,1);
            lcd_string("Obstacle Ahead");
            //_delay_ms(200);
            continue;
        }
        else{
            buzzer_off();
            clear_led();
            lcd_cursor(2,1);
            lcd_string("No Obstacle");
        }

// If no object in front, go forward and come in next loop
    if(Front_Sharp_Sensor < 30) {
            forward();
            velocity(244,255);
          //  _delay_ms(50);
          /*  if(lane == 1){

                    }////////////

            continue;
        }
        else{
            forward();
            velocity(119,125);
            //_delay_ms(50);
            if(safe == 1){
                switch_right();
            }
            else{
                clear_led();
                lcd_cursor(2,1);
                lcd_string("Waiting");
            }
        }
    }

//    smart_overtake();
//    cross();

	/*while(1)

	{

		Front_Sharp_Sensor = ADC_Conversion(11);
		Right_Sharp_Sensor = ADC_Conversion(12);
		Left_Sharp_Sensor = ADC_Conversion(10);
        Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
        Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
        Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

        Full_Left_Sharp_Sensor = ADC_Conversion(9);
        Full_Right_Sharp_Sensor= ADC_Conversion(13);
       // print_sensor(1,1,9);
        //print_sensor(1,5,13);

		//print_sensor(2,4,11);	//Prints Value of Front Sharp Sensor
		//print_sensor(2,1,3); // Left_sharp_Sensor
		//print_sensor(2,5,2); // Center_Sharp_Sensor
		//print_sensor(2,9,1); // Right_Sharp_Sensor

		//buzzer_off();


        if(Front_Sharp_Sensor < 30) {
            buzzer_off();
            forward();
        }
        else if(Front_Sharp_Sensor >= 30 && Right_Sharp_Sensor < 30){
            total_right_overtake();
        }
        else if(Front_Sharp_Sensor >= 30 && Right_Sharp_Sensor >= 30 && Left_Sharp_Sensor < 30){
            total_left_overtake();
        }
        else if(Front_Sharp_Sensor >= 30 && Right_Sharp_Sensor >= 30 && Left_Sharp_Sensor >= 30){
            stop();
            buzzer_on();
        }*/

//	}
}

