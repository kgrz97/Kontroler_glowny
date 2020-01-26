#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <HD44780/HD44780.c>
//#include <util/crc16.h>
#include <avr/eeprom.h>

//#include "crc.h"
#include "base64.h"

#define UART_BAUD 38400
#define __UBRR ((F_CPU+UART_BAUD*8UL) / (16UL*UART_BAUD)-1)

#define NELEMS(x)  (sizeof(x) / sizeof(x[0]))


void USART_Init(uint16_t ubrr);
void USART_Off();
void Usart_Transmit(unsigned char data);
void Send_clause();

void USART2_Init(unsigned int ubrr);
void Usart2_Transmit(unsigned char data);
unsigned char USART2_Receive();
void Send_clause2(unsigned char * napis);

void Find_Start(int addr);
void Set_Zero();
unsigned char USART_Receive();
void Timer_Init();
void Timer1_Off();
void pid(int enc);

void Ask_Encoder(int c);

void Timer3_Init();


//uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData);

void int_to_char (int l, char* tab);

int char_to_int(char* c_array);

void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);

void Write_Position_EEPROM(int Ring, int Pos);
void Read_Position_EEPROM(int Ring);


void Menu();

int Ret_flag = 0;
volatile unsigned char Usart2_receive, Addr;


double _dt = 0.5;
double _max = 100;
double _min = -100;
double _Kp = 4;
double _Kd = 0.2;
volatile double _pre_error=0, _pre_error2=0, _pre_error3=0;
volatile double _integral=0;

volatile int stan1, pozycja1=0, stan2, pozycja2=0, stan3, pozycja3=0;
volatile int cnt, cnt3;
volatile int pre_stan=0, pre_stan2=0, pre_stan3=0;

volatile uint8_t i = 0, write_flag = 0, ins = 0, move_flag = 0, pid_enc = 1;
volatile unsigned char set[6];

uint8_t qtFlag = 0;
//volatile int start_poz1 = 43, start_poz2 = 455, start_poz3 = 2;
volatile int chapter = -1;
volatile uint8_t Ring1_flag=0, Ring2_flag=0, Ring3_flag=0;

volatile char Address = '0';
volatile char Prompt_PC[10];
volatile uint8_t Prompt_counter = 0;
volatile uint8_t Rings = 0;

//EEMEM char Stan_zero;


//------------------------------------------------
char dane[2];
int counter = 0;


int main(void)
{
	MCUCR = (1<<JTD);
	MCUCR = (1<<JTD);

	XMCRA &= ~(1<<SRE);

// enkoder + przycisk
	DDRE &= ~(1<<PE6); 	// Pin B0 jako wyjscie przycisku
	PORTE |= (1<<PE6);

	DDRE &= ~(1<<PE5);
	PORTE |= (1<<PE5);
	DDRE &= ~(1<<PE4);
	PORTE |= (1<<PE4);

	//int enkoder = 0;

// DRV2
	DDRA |= (1<<PA0)|(1<<PA1)|(1<<PA2)|(1<<PA4)|(1<<PA5)|(1<<PA6);
	//			en									step	dir
	PORTA &= ~(1<<PA0);
	PORTA &= ~(1<<PA5);
	PORTA &= ~(1<<PA6);
	PORTA &= ~(1<<PA2);

	PORTA |= (1<<PA1)|(1<<PA4);

//DRV1
	DDRC |= (1<<PC1)|(1<<PC2)|(1<<PC5)|(1<<PC6)|(1<<PC7);
	//			dir		step	ms1			ms2		en
	PORTC &= ~(1<<PC7);
	PORTC &= ~(1<<PC5);
	PORTC &= ~(1<<PC2);


	PORTC |= (1<<PC1)|(1<<PC6);



//DRV3
	DDRF |= (1<<PF0)|(1<<PF1)|(1<<PF2)|(1<<PF4)|(1<<PF5)|(1<<PF6);
	PORTF &= ~(1<<PF0);
	PORTF &= ~(1<<PF5);
	PORTF &= ~(1<<PF6);
	PORTF &= ~(1<<PF2);

	PORTF |= (1<<PF1)|(1<<PF4);


	USART_Init(__UBRR);


	USART2_Init(__UBRR);


	LCD_Initalize();

	/*uint8_t crc = 0;

	char tab[2] = {'1', '2', '3'};

	for(int i=0; i<=2; i++)
	{
		crc = _crc8_ccitt_update(crc, tab[i]);
	}


	Usart2_Transmit(crc);

	Send_clause2("\r\n");
*/

	DDRE |= (1<<PE2)|(1<<PE3);

	PORTE &= ~(1<<PE2);   // PE2=0 && PE3=0  -> Receiver
	PORTE &= ~(1<<PE3);	 // PE2=1 && PE3=1	-> Transmitter


	/*while(1)
	{
		char rec = USART_Receive();
		Usart2_Transmit(rec);
	} */




				/*char num[3];


				sprintf(num,"%d", 313);

				int l = NELEMS(num);

				num[l] = '\0';

				const unsigned int* test_a[] = {num[0], num[1], num[2], num[3]};

				unsigned int size_a = NELEMS(test_a);

				unsigned int out_size_a = b64e_size(size_a) + 1;
				unsigned char *out_a = malloc(out_size_a);

				out_size_a = b64_encode((const unsigned int*)test_a,size_a,out_a);

				Send_clause2(out_a);
				Send_clause2("  ");



_delay_ms(500);

						int len_a = out_size_a;

						int out_size_ad = b64d_size(len_a);
						unsigned int *out_ad = malloc(out_size_ad+100);
						out_size_ad = b64_decode((const unsigned int *)out_a, len_a, out_ad); // ???


						for(int i=0; i<out_size_ad; i++)
						{
							Usart2_Transmit(out_ad[i]);
						}
						Usart2_Transmit(' ');
						char s[10];
						sprintf(s,"%d", out_size_ad);
						Send_clause2(s);
						Send_clause2("\r\n");


	_delay_ms(200);  */


	DDRB |= (1<<PB6)|(1<<PB7); // debug LED
	PORTB |= (1<<PB6)|(1<<PB7);
	_delay_ms(1000);
	PORTB &= ~(1<<PB6);
	PORTB &= ~(1<<PB7);

	_delay_ms(1500);

	char c_pozycja[3];

	Find_Start(1);
	_delay_ms(250);
	Find_Start(2);
	_delay_ms(250);
	Find_Start(3);
	_delay_ms(250);


	Set_Zero();


	Menu();

	chapter = 0;
	stan1 = 0;

	pre_stan = 0;
	pre_stan2=0;

	sei();

	PORTC &= ~(1<<PC7);
	PORTA &= ~(1<<PA0);
	PORTF &= ~(1<<PF0);

	PORTB &= ~(1<<PB6);
	PORTB &= ~(1<<PB7);

	uint8_t Ring = 1;
	int Enk_pos = 0;

	while(1)
	{
		if(chapter == 0)
		{
			//unsigned char odpc = USART2_Receive();

			if(move_flag == 1)
			{
				chapter = 1;
				cnt = 500;
				Timer_Init();
				LCD_Clear();
			}



			if(!(PINE & (1<<PE6)))
			{
				_delay_ms(150);
				Usart2_receive = ' ';

				if(Ring == 1)
					pozycja1 = Enk_pos;

				if(Ring == 2)
					pozycja2 = Enk_pos;

				if(Ring == 3)
					pozycja3 = Enk_pos;

				Enk_pos = 0;

				Ring++;


				//if(pozycja1>=0)
				//	PORTC |= (1<<PC1);
				//else
				//	PORTC &= ~(1<<PC1);

				if(Ring >= 4)
				{
					LCD_GoTo(1,1);
					LCD_WriteText("Start");
					_delay_ms(1500);
					LCD_Clear();

					Ring = 1;

					chapter = 1;

					cnt = 500;

					Timer_Init();
				}
			}


			if((PINE & (1<<PE4)))
			{
				if(!(PINE & (1<<PE5)))
				{
					if(Enk_pos < 500)
						Enk_pos+=5;
				}

			} else
			{
				if(PINE & (1<<PE5))
				{
					if(Enk_pos > 0)
						Enk_pos-=5;
				}
			}


			 itoa(Enk_pos, c_pozycja, 10);
			 LCD_Clear();
			 LCD_GoTo(0,0);
			 if(Ring == 1)
				 LCD_WriteText("Pos 1 = ");

			 if(Ring == 2)
			 	 LCD_WriteText("Pos 2 = ");

			 if(Ring == 3)
			 	 LCD_WriteText("Pos 3 = ");

			 LCD_GoTo(0,9);
			 LCD_WriteText(c_pozycja);

			 _delay_ms(70);
		}

		 if(chapter == 1)
		 {
			// LCD_Clear();
			 LCD_GoTo(0,0);
			 LCD_WriteText("Wait...");
			 LCD_GoTo(5,1);
			 LCD_WriteText("<Click>Stop");


			if(!(PINE & (1<<PE6)) || Usart2_receive == 'B' || move_flag == 0)
			{
					chapter = 0;

					move_flag = 0;

					Timer1_Off();

					_delay_ms(10);

					LCD_Clear();
					LCD_GoTo(0,0);
					LCD_WriteText("Done");
					_delay_ms(3000);

					LCD_Clear();
			}


		 }

	} // end while

} // end main

ISR(USART1_RX_vect)
{
	Usart2_receive = UDR1;
	// Code to be executed when the USART receives a byte here

	if(UDR1 == 'P')
	{
		write_flag = 1;

	}

	if(write_flag == 1)
	{
		Prompt_PC[Prompt_counter] = Usart2_receive;
		Prompt_counter++;

		if(ins != 0 && Usart2_receive != 'K')
		{
			set[i] = Usart2_receive;
			i++;
		}

		if(Usart2_receive == 'S')
			ins = 1;

		if(Usart2_receive == 'G')
			ins = 2;

		if(Usart2_receive == 'U')
			ins = 3;

		if(Usart2_receive == 'A')
			ins = 4;






		if(Usart2_receive == 'K')
		{
			if(ins == 1)
			{
				LCD_Clear();
				LCD_GoTo(0,0);
				LCD_WriteText("New settings");
				_delay_ms(3500);
				LCD_Clear();


				if(set[1] == '0')
				{
					PORTC |= (1<<PC5);
					PORTC &= ~(1<<PC6);
				}

				if(set[1] == '1')
				{
					PORTC &= ~(1<<PC5); // ms1
					PORTC |= (1<<PC6);
				}

				if(set[1] == '2')
				{
					PORTC &= ~(1<<PC5);
					PORTC &= ~(1<<PC6);
				}
				if(set[1] == '3')
				{
					PORTC |= (1<<PC5);
					PORTC |= (1<<PC6);
				}
			}

			if(ins == 2)
			{
				qtFlag = 0;

				set[i] = '\0';

				LCD_Clear();
				LCD_GoTo(0,0);

				if(Prompt_PC[0] == '1')
				{
					LCD_WriteText("Pos 1 = ");
					pozycja1 = atoi(set);
					Rings++;
				}
				if(Prompt_PC[0] == '2')
				{
					LCD_WriteText("Pos 2 = ");
					pozycja2 = atoi(set);
					Rings++;
				}


				if(Prompt_PC[0] == '3')
				{
					LCD_WriteText("Pos 3 = ");
					pozycja3 = atoi(set);
					Rings++;
				}

				LCD_GoTo(0,5);
				LCD_WriteText(set);

				_delay_ms(500);

				if(Rings >= 3)
				{
					move_flag = 1;
					Rings = 0;
				}

			}

			if(ins == 3)
				move_flag = 1;

			if(ins == 4)
			{
				Send_clause2("PLK");
			}


			ins = 0;
			i=0;
			write_flag = 0;
			Prompt_counter = 0;
		}
	}








		/*if(set[1] == '1')
		{
			PORTC &= ~(1<<PC5); // ms1
			PORTC |= (1<<PC6);
		}
		if(set[1] == '2')
		{
			PORTC &= ~(1<<PC5);
			PORTC &= ~(1<<PC6);
		}  */




}

void USART_Init( unsigned int ubrr)
{
		UBRR0H = (unsigned char)(ubrr>>8);
		UBRR0L = (unsigned char)ubrr;
		UCSR0B = (1<<RXEN0)|(1<<TXEN0);
		//UCSR0C = (1<<UMSEL00);
}

void USART2_Init(unsigned int ubrr)
{
	UBRR1H = (unsigned char)(ubrr>>8);
	UBRR1L = (unsigned char)ubrr;
	UCSR1B = (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1);
	//UCSR1A = (1<<RXCIE1);
}

void Usart2_Transmit(unsigned char data)
{
	while ( !( UCSR1A & (1<<UDRE1)) );
		UDR1 = data;
}
unsigned char USART2_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR1A & (1<<RXC1)) )
	;
	/* Get and return received data from buffer */
	return UDR1;
}

void Send_clause2(unsigned char * napis)
{
	while(*napis)
		Usart2_Transmit(*napis++);
}


void USART_Off()
{
	UCSR0B &= ~(1<<TXEN0);
}


void Usart_Transmit(unsigned char data)
{
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 = data;
}

void Send_clause(char * napis)
{
	while(*napis)
		Usart_Transmit(*napis++);
}

unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	/* Get and return received data from buffer */
	return UDR0;
}


void Find_Start(int addr)
{
	LCD_Clear();
	LCD_GoTo(0,0);
	LCD_WriteText("Wait");

	//for(int i=1; i<=3; i++)
	//{
		//PORTE |= (1<<PE2);   // PE2=0 && PE3=0  -> Receiver
		//PORTE |= (1<<PE3);
		//_delay_ms(5);

		//if(i==1)
		//{
			//Send_clause("R1S");
			//_delay_ms(10);
		//}

		/*if(i==2)
		{
			Send_clause("R2S");
			_delay_ms(10);
		}

		if(i==3)
		{
			Send_clause("R3S");
			_delay_ms(10);
		}*/

		//////////////////	Zero searching; Ask->Response

	_delay_ms(1000);
		while(1)
		{

			if(addr == 1)
			{
				PORTE |= (1<<PE2);   // PE2=0 && PE3=0  -> Receiver
				PORTE |= (1<<PE3);
				_delay_ms(15);
				Send_clause("R1S");
				_delay_ms(5);


				PORTE &= ~(1<<PE2);   // PE2=0 && PE3=0  -> Receiver
				PORTE &= ~(1<<PE3);
				_delay_ms(5);

				PORTB |= (1<<PB7);

				char handler[4];
				int it = 0;

				while(1)
				{
					unsigned char r = USART_Receive();

					if(r == 'K')
						break;

					handler[it] = r;
					it++;
				}

				if(handler[1] == '1' && handler[2] == 'N')
				{
					_delay_ms(2);
					PORTC |= (1<<PC2);
					_delay_ms(2);
					PORTC &= ~(1<<PC2);
					_delay_ms(5);
				}
				else
				{
					//PORTC |= (1<<PC7);
					break;
				}


				/*if(handler[1] == '1' && handler[2] == 'P')
				{
					PORTC |= (1<<PC7);
					break;
				}*/

				PORTB &= ~(1<<PB7);

			}


			if(addr == 2)
						{
							PORTE |= (1<<PE2);   // PE2=0 && PE3=0  -> Receiver
							PORTE |= (1<<PE3);
							_delay_ms(15);
							Send_clause("R2S");
							_delay_ms(5);


							PORTE &= ~(1<<PE2);   // PE2=0 && PE3=0  -> Receiver
							PORTE &= ~(1<<PE3);
							_delay_ms(5);

							PORTB |= (1<<PB6);

							char handler[4];
							int it = 0;

							while(1)
							{
								unsigned char r = USART_Receive();

								if(r == 'K')
									break;

								handler[it] = r;
								it++;
							}

							if(handler[1] == '2' && handler[2] == 'N')
							{
								_delay_ms(2);
								PORTA |= (1<<PA5);
								_delay_ms(2);
								PORTA &= ~(1<<PA5);
								_delay_ms(5);
							}


							if(handler[1] == '2' && handler[2] == 'P')
							{
								PORTA |= (1<<PA0);
								break;
							}

							PORTB &= ~(1<<PB6);

						}

			if(addr == 3)
			{
					PORTE |= (1<<PE2);   // PE2=0 && PE3=0  -> Receiver
					PORTE |= (1<<PE3);
					_delay_ms(15);
					Send_clause("R3S");
					_delay_ms(5);


					PORTE &= ~(1<<PE2);   // PE2=0 && PE3=0  -> Receiver
					PORTE &= ~(1<<PE3);
					_delay_ms(5);

					PORTB |= (1<<PB6);

					char handler[4];
					int it = 0;

					while(1)
					{
						unsigned char r = USART_Receive();

						if(r == 'K')
							break;

					handler[it] = r;
					it++;
					}

					if(handler[1] == '3' && handler[2] == 'N')
					{
						_delay_ms(2);
						PORTF |= (1<<PF5);
						_delay_ms(2);
						PORTF &= ~(1<<PF5);
						_delay_ms(5);
					}


					if(handler[1] == '3' && handler[2] == 'P')
					{
						PORTF |= (1<<PF0);
							break;
					}

					PORTB &= ~(1<<PB6);

				}


		/*	if(enc == 1 && end1 == 0)
				Send_clause("R1S");
			if(enc == 2 && end2 == 0)
				Send_clause("R2S");
			_delay_ms(5);

			enc++;

			PORTE &= ~(1<<PE2);   // PE2=0 && PE3=0  -> Receiver
			PORTE &= ~(1<<PE3);
			_delay_ms(5);

			PORTB |= (1<<PB7);

			char handler[4];
			int it = 0;

			while(1)
			{
				unsigned char r = USART_Receive();

				if(r == 'K')
					break;

				handler[it] = r;
				it++;
			}
			//unsigned char r = USART_Receive();

			//Usart2_Transmit(r);
			//unsigned char rec1 = USART_Receive();

			if(end1 == 0)
			{
			if(handler[1] == '1' && handler[2] == 'N')
			{
				_delay_ms(2);
				PORTC |= (1<<PC2);
				_delay_ms(2);
				PORTC &= ~(1<<PC2);
				_delay_ms(5);
			}


			if(handler[1] == '1' && handler[2] == 'P')
			{
				end1=1;
				PORTC |= (1<<PC7);
			}
			}

			if(end2 == 0)
			{
			if(handler[1] == '2' && handler[2] == 'N')
						{
							_delay_ms(2);
							PORTA |= (1<<PA5); // zmienic port
							_delay_ms(2);
							PORTA &= ~(1<<PA5);
							_delay_ms(5);
						}


						if(handler[1] == '2' && handler[2] == 'P')
						{
							//break;
							end2=1;
							PORTA |= (1<<PA0);
						}
			}

			_delay_ms(5); */

			PORTB &= ~(1<<PB7);


			//if(end1 + end2 >= 2)
			//	break;

		}

		/////////////////////


		PORTC |= (1<<PC1);

		/*PORTE &= ~(1<<PE2);   // PE2=0 && PE3=0  -> Receiver
		PORTE &= ~(1<<PE3);
		_delay_ms(5);

		while(1)
		{
			unsigned char odb = USART_Receive();
			//Usart2_Transmit(odb);


			if(odb == 'P')
				break;

			_delay_ms(10);
			PORTC |= (1<<PC2);
			_delay_ms(10);
			PORTC &= ~(1<<PC2);
		} */


		//PORTE |= (1<<PE2);
		//PORTE |= (1<<PE3);



	LCD_Clear();
	LCD_GoTo(0,0);
	LCD_WriteText("Init OK");

	_delay_ms(2000);
	LCD_Clear();

	sei();

	PORTB &= ~(1<<PB7);
}

/* Inicjalizacja timera
 * - wys³anie ¿¹dania do slave'a
 * - czekanie na odpowiedz
 * - interpretacja odpowiedzi*/

void Timer3_Init()
{
	TCNT3 = 0;
	TCCR3A = (1<<WGM31);
	TCCR3B = (1<<CS30);//| (1<<CS12);
	TIFR3 = (1<<TOV3);
	TIMSK3 = (1<<TOIE3);//(1<<OCIE1A);
	cnt3 = 250;

}

ISR (TIMER3_OVF_vect)
{
} // end Timer3_Init

void Timer_Init()
{
		TCNT1 = 0;   //

		TCCR1A = (1<<WGM01);
		TCCR1B = (1<<CS10);//| (1<<CS12);
		TIFR1 = (1<<TOV1);
		TIMSK1 = (1<<TOIE1);//(1<<OCIE1A);
		//sei();        // Enable global interrupts by setting global interrupt enable bit in SREG
		cnt = 500;

}

void Timer1_Off()
{
	TCCR1B = 0x00;
}

ISR (TIMER1_OVF_vect)    // Timer1 ISR
{
	cnt--;

	if(cnt == 150)
	{
		Ask_Encoder(1);
		cnt=149;
	}

	if(cnt == 100)
	{
		Ask_Encoder(2);
		cnt=99;
	}

	if(cnt == 50)
	{
		Ask_Encoder(3);
		cnt=49;
	}


	if(cnt <= 0)
	{
		pid(pid_enc);

		pid_enc++;

		if(pid_enc >= 4)
			pid_enc = 1;

		if(chapter == -1 && Ring1_flag == 1 && Ring2_flag == 1 && Ring3_flag == 1)
		{
			chapter = 0;
			Timer1_Off();
		}
	}

}

void pid(int enc) // implement this as timer interrupt service routine {
{
	int error_pd = 0;

	if(enc == 1)
		error_pd = pozycja1 - pre_stan;

	if(enc == 2)
		error_pd = pozycja2 - pre_stan2;

	if(enc == 3)
		error_pd = pozycja3 - pre_stan3;

	 double Pout = _Kp * error_pd;
	 double derivative = 0;

	 if(enc == 1)
		 derivative = (error_pd - _pre_error) / _dt;

	 if(enc == 2)
		 derivative = (error_pd - _pre_error2) / _dt;

	 if(enc == 3)
		 derivative = (error_pd - _pre_error3) / _dt;

	 double Dout = _Kd * derivative;
	 double output = Pout + Dout;

	 double output_round = round(output);

if(abs(error_pd) > 250)
{
		if(error_pd >= 250 && enc == 1)
		{
			PORTC &= ~(1<< PC1);
		}

		if(error_pd < 250 && enc == 1)
		{
			PORTC |= (1<< PC1);
		}

		if(error_pd >= 250 && enc == 2)
		{
			PORTA &= ~(1<< PA6);
		}

		if(error_pd < 250 && enc == 2)
		{
			PORTA |= (1<< PA6);
		}

		if(error_pd >= 250 && enc == 3)
		{
			PORTF &= ~(1<<PF6);
		}
		if(error_pd < 250 && enc == 3)
		{
			PORTF |= (1<<PF6);
		}
}
else
{
	if(output_round >= 0 && enc == 1)
		PORTC |= (1<< PC1); // ttuaj jest zamiana
	else
		PORTC &= ~(1<< PC1);

	if(output_round >= 0 && enc == 2)
		PORTA |= (1<< PA6);
	else
		PORTA &= ~(1<< PA6);

	if(output_round >= 0 && enc == 3)
		PORTF |= (1<<PF6);
	else
		PORTF &= ~(1<<PF6);
}




	if(output_round != 0)
	{
		if(enc == 1)
		{
			PORTC &= ~(1<<PC7);

			PORTC |= (1<<PC2);
			_delay_ms(1);
			PORTC &= ~(1<<PC2);
		}

		if(enc == 2)
		{
			PORTA &= ~(1<<PA0);

			PORTA |= (1<<PA5);
			_delay_ms(1);
			PORTA &= ~(1<<PA5);
		}

		if(enc == 3)
		{
			PORTF &= ~(1<<PF0);

			PORTF |= (1<<PF5);
			_delay_ms(1);
			PORTF &= ~(1<<PF5);
		}
	}

	/*if(output_round == 0)
	{
		if(enc == 1)
			PORTC |= (1<<PC7);

		if(enc == 2)
			PORTA |= (1<<PA0);
	}*/

	if(enc == 1)
		_pre_error = error_pd;

	if(enc == 2)
		_pre_error2 = error_pd;

	if(enc == 3)
		_pre_error3 = error_pd;


	if(enc == 1)
	{
		TCNT1 = 0;

		double count = (1/output_round)*10000;
		cnt = abs((int) count);

		if(cnt < 200)
			cnt = 200;

		if(cnt > 750)
			cnt = 750;
	}


	if(chapter == -1)
	{
		if(_pre_error == 0)
			Ring1_flag = 1;
		else
			Ring1_flag = 0;

		if(_pre_error2 == 0)
			Ring2_flag = 1;
		else
			Ring2_flag = 0;

		if(_pre_error3 == 0)
			Ring3_flag = 1;
		else
			Ring3_flag = 0;

	}

/*if(enc == 1)
{
	char c[10];
	itoa(pre_stan, c, 10);
	Send_clause2(c);
    Send_clause2(" ");
	itoa(error_pd, c, 10);
	Send_clause2(c);
	Send_clause2(" ");
	itoa(output, c, 10);
	Send_clause2(c);
	Send_clause2(" ");
	itoa(cnt, c, 10);
	Send_clause2(c);
	Send_clause2("\r\n");
}*/


}

/*uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData)
     {
         uint8_t   i;
         uint8_t   data;

         data = inCrc ^ inData;

         for ( i = 0; i < 8; i++ )
         {
            if (( data & 0x80 ) != 0 )
           {
                data <<= 1;
                data ^= 0x07;
           }
           else
           {
                data <<= 1;
           }
        }
        return data;
    } */

void int_to_char (int l, char* tab)
{
	int i =0;
	char c_array[3];

	if(l==0)
	{
		tab[i++] = '0';
	}else
	{
		while (l != 0)
		    {
		        int rem = l % 10;
		        c_array[i++] = (rem > 9)? (rem-10) + 'a' : rem + '0';
		        l = l/10;
		    }

		int p =0;
		for(int j = i-1; j>=0; j--)
		{
			tab[p]=c_array[j];
			p++;
		}

	}
	tab[i]='\0';
}

int char_to_int(char* c_array)
{
	    int res = 0; // Initialize result

	    // Iterate through all characters of input string and
	    // update result
	    for (int i = 0; c_array[i] != '\0'; ++i)
	        res = res * 10 + c_array[i] - '0';

	    // return result.
	    return res;
}

void Ask_Encoder(int c)
{
					PORTE |= (1<<PE2);
					PORTE |= (1<<PE3);
					_delay_ms(5);


					//Usart_Transmit('R');
					//PORTB |= (1<<PB7);
					if(c==1)
					{
						Send_clause("R1");
					}
					if(c==2)
					{
						Send_clause("R2");
					}
					if(c==3)
					{
						Send_clause("R3");
					}
					_delay_ms(2);
					PORTE &= ~(1<<PE2);
					PORTE &= ~(1<<PE3);

					PORTB |= (1<<PB6);

					int j=0;
					char ciag[10] = "";



					while(1)
					{
							unsigned char odb = USART_Receive();


							if(odb == 'K') // \0
							{
								break;
							}
							else
							{
								//Usart2_Transmit(odb);
								ciag[j]=odb;
								j++;
							}
					}
					//PORTB &= ~(1<<PB7);
					//Send_clause2("\r\n");
					/*
					PORTB |= (1<<PB6);

					//Send_clause2("  ");

					ciag[j] = '\0';

					//Send_clause2(ciag[0]); */




					char tab[5] = {ciag[1], ciag[2], ciag[3], ciag[4], ciag[5]};

					tab[j] = '\0';
					int out_size_ad = b64d_size(j);
					unsigned int out_ad[6];//*out_ad = malloc(out_size_ad+100);
					out_size_ad = b64_decode((const unsigned char*)tab, j, out_ad);

					char num[4] = {out_ad[0], out_ad[1], out_ad[2], '\0'};

					//pre_stan = 0;
					//pre_stan2 = 0;

					if(c==1)
					{
						pre_stan = char_to_int(num);
					}


					if(c==2)
						pre_stan2 = char_to_int(num);

					if(c==3)
						pre_stan3 = char_to_int(num);



					if(pre_stan == pozycja1 && pre_stan2 == pozycja2 && pre_stan3 == pozycja3 && qtFlag == 0)
					{
							Send_clause2("IAD\0");
							move_flag = 0;
							qtFlag = 1;
					}

					PORTB &= ~(1<<PB6);
}

void EEPROM_write(unsigned int ucAddress, unsigned char ucData)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set Programming mode */
	EECR = (0<<EEPM1)|(0<<EEPM0);
	/* Set up address and data registers */
	EEAR = ucAddress;
	EEDR = ucData;
	/* Write logical one to EEMPE */
	EECR |= (1<<EEMPE);
	/* Start eeprom write by setting EEPE */
	EECR |= (1<<EEPE);
}

unsigned char EEPROM_read(unsigned int ucAddress)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set up address register */
	EEAR = ucAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from data register */
	return EEDR;
}

void Write_Position_EEPROM(int Ring, int Pos)
{
	uint8_t l1=0, l2=0;

	if(Pos > 250)
	{
		l1=250;
		l2=Pos-250;
	}
	else
	{
		l1=Pos;
		l2=0;
	}

	if(Ring == 1)
	{
		EEPROM_write(100, l1);
		EEPROM_write(101, l2);
	}

	if(Ring == 2)
	{
		EEPROM_write(200, l1);
		EEPROM_write(201, l2);
	}

	if(Ring == 3)
	{
		EEPROM_write(300, l1);
		EEPROM_write(301, l2);
	}
}

void Read_Position_EEPROM(int Ring)
{
	uint8_t l1=0,l2=0;

	if(Ring == 1)
	{
		l1 = EEPROM_read(100);
		l2 = EEPROM_read(101);
		pozycja1 = l1+l2;
	}

	if(Ring == 2)
	{
		l1 = EEPROM_read(200);
		l2 = EEPROM_read(201);
		pozycja2 = l1+l2;
	}

	if(Ring == 3)
	{
		l1 = EEPROM_read(300);
		l2 = EEPROM_read(301);
		pozycja3 = l1+l2;
	}
}

void Menu()
{
	int wsk = 0, Step_size = 0;

	int8_t C = 4, U = 0;
	char Ca[3], Ua[3];

	while(1)
	{
		switch(wsk)
		{
		case 0: { LCD_GoTo(0,0);
					LCD_WriteText(">Start process");
					LCD_GoTo(0,1);
					LCD_WriteText("Speed"); break;}
		case 1: { LCD_GoTo(0,0);
					LCD_WriteText(">Speed");
					LCD_GoTo(0,1);
					LCD_WriteText("Step size"); break;}
		case 2: { LCD_GoTo(0,0);
					LCD_WriteText("Speed");
					LCD_GoTo(0,1);
					LCD_WriteText(">Step size"); break;}
		case 5: { LCD_GoTo(0,0);
					LCD_WriteText("Speed");
					LCD_GoTo(1,1);
					int_to_char(C, Ca);
					LCD_WriteText(Ca);
					LCD_GoTo(3,1);
					LCD_WriteText(",");
					LCD_GoTo(4,1);
					int_to_char(U, Ua);
					LCD_WriteText(Ua);
					break;}
		case 6: {LCD_GoTo(0,0);
					LCD_WriteText("Step size");
					LCD_GoTo(0,1);
					if(Step_size == 0)
						LCD_WriteText("1/2");
					if(Step_size == 1)
						LCD_WriteText("1/4");
					if(Step_size == 2)
						LCD_WriteText("1/8");
					if(Step_size == 3)
						LCD_WriteText("1/16");
					break;}
		}

		_delay_ms(75);

		if((PINE & (1<<PE4)))
		{
			LCD_Clear();
			if(!(PINE & (1<<PE5)))
			{
				if(wsk > 3 && Step_size < 3)
					Step_size++;


				if(wsk < 2)
						wsk++;

				if(wsk == 5)
				{
					U++;

					if(U == 10)
					{
						C++;
						U=0;
					}
				}
			}
		} else
		{
			LCD_Clear();
			if(PINE & (1<<PE5))
			{
				if(wsk>3 && Step_size > 0)
					Step_size--;

				if(wsk > 0 && wsk <= 3)
					wsk--;

				if(wsk == 5)
				{
					U--;

					if(U == -1)
					{
						C--;
						U=9;
					}
				}
			}
		}


		if(!(PINE & (1<<PE6)))
		{
			if(wsk == 0)
				break;

			if(wsk==1)
			{
				wsk=5;
				continue;
			}

			if(wsk==2)
			{
				wsk=6;
				continue;
			}

			if(wsk==5)
			{
				wsk=1;
				continue;
			}

			if(wsk==6)
			{
				wsk=2;
				continue;
			}
		}

	}


	switch(Step_size)
	{
	case 0: {	PORTC |= (1<<PC5);
				PORTC &= ~(1<<PC6); break;}

	case 1: {	PORTC &= ~(1<<PC5); // ms1
				PORTC |= (1<<PC6); break;}

	case 2: {	PORTC &= ~(1<<PC5);
				PORTC &= ~(1<<PC6); break;}

	case 3: {	PORTC |= (1<<PC5);
				PORTC |= (1<<PC6); break;}
	}


	_delay_ms(500);
}


void Set_Zero()
{
	// ustawienia
			uint8_t wsk = 0;

			while(1)
			{
				if(wsk == 0)
				{
					LCD_GoTo(0,0);
					LCD_WriteText(">Set start");
					LCD_GoTo(0,1);
					LCD_WriteText("Use existing");
				}

				if(wsk == 1)
				{
					LCD_GoTo(0,0);
					LCD_WriteText("Set start");
					LCD_GoTo(0,1);
					LCD_WriteText(">Use existing");
				}


				if((PINE & (1<<PE4)))
				{
					if(!(PINE & (1<<PE5)))
					{
						if(wsk == 0)
							wsk = 1;
					}
				} else
				{
					if(PINE & (1<<PE5))
					{
						if(wsk == 1)
							 wsk = 0;
					}
				}

				if(!(PINE & (1<<PE6)))
					break;

				_delay_ms(75);
				LCD_Clear();
			}

			_delay_ms(500);

			if(wsk == 0)
			{
				LCD_Clear();
				LCD_GoTo(0,0);
				LCD_WriteText("Set start");
				LCD_GoTo(5,1);
				LCD_WriteText("<Click> OK");


				while(1)
				{
					if(!(PINE & (1<<PE6)))
						break;
				}

				Ask_Encoder(1);
				_delay_ms(100);
				Write_Position_EEPROM(1, pre_stan);

				Ask_Encoder(2);
				_delay_ms(100);
				Write_Position_EEPROM(2, pre_stan2);

				Ask_Encoder(3);
				_delay_ms(100);
				Write_Position_EEPROM(3, pre_stan3);

				char a[10];
				itoa(pre_stan3, a, 10);



				LCD_Clear();
				LCD_GoTo(0,0);
				LCD_WriteText(a);

				_delay_ms(2500);

				PORTE |= (1<<PE2);
				PORTE |= (1<<PE3);

				_delay_ms(100);
				//Send_clause("RA0K");
				Send_clause("RZAK");
				//Send_clause2("R10K");

				_delay_ms(1200);

				pozycja1 = 0;
				pozycja2 = 0;
				pozycja3 = 0;
				/*PORTE &= ~(1<<PE2);   // PE2=0 && PE3=0  -> Receiver
				PORTE &= ~(1<<PE3);
				_delay_ms(5);

				unsigned char p;

				while(1)
				{
					unsigned char r = USART_Receive();
					Usart2_Transmit(r);
					Ret_flag = 0;

					if(r == 'S' && p=='1')
						break;

					p = r;
				} */

			} // end if wsk == 0
			else if(wsk == 1)// wsk == 1
			{
				Read_Position_EEPROM(1);
				Read_Position_EEPROM(2);
				Read_Position_EEPROM(3);

				_delay_ms(1000);
			//	Write_Position_EEPROM(1, pozycja1);
			//	Write_Position_EEPROM(2, pozycja2);
			//	Write_Position_EEPROM(3, pozycja3);

				cnt = 500;

				LCD_Clear();
				LCD_GoTo(0,0);
				LCD_WriteText("Zero positions");
				LCD_GoTo(0,1);
				LCD_WriteText("Wait...");

				Timer_Init();

				while(chapter == -1)
				{}
				LCD_Clear();
				LCD_GoTo(0,0);
				LCD_WriteText("OK");
				_delay_ms(1500);

				PORTE |= (1<<PE2);
				PORTE |= (1<<PE3);

				_delay_ms(100);

				Send_clause("RZAK");

				_delay_ms(1200);

				pozycja1 = 0;
				pozycja2 = 0;
				pozycja3 = 0;


				//////////// zapis do eeprom
				///////////
			}
}
