/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#include "derivative.h" /* include peripheral declarations */

#define nt15_msec	0x02E8
#define nt40_usec	0x1AD0

//LCD Control
#define nIns	0
#define nData	1

#define PortLCD    	GPIOD_PDOR
//Enable connected to portb_01
#define Enable_1	GPIOB_PDOR |= 0x02
#define Enable_0	GPIOB_PDOR &= 0xFD
#define RS_1   		GPIOB_PDOR |= 0x01
#define RS_0   		GPIOB_PDOR &= 0xFE

//#define iData 		GPIOE_PDIR |= 0x01 ///  E0 datos
//#define iClock 		GPIOE_PDIR |= 0x02

int int_Temp;
unsigned char string_out[32];
unsigned char i=0;


void cfgPorts(void);
void initLCD(void);
void delay(long time);
void sendCode(int Code, int Data);

void valor (void);
void keyboard (void);
void character (void);
void caps_on (void);
void mayusculas (void);
void comparar (void);
void posicion (void);
void envpag (void);
void UARTinit(void);
void send_string_bypointer(unsigned char *pointer_string,unsigned char limit);
void UART0_tx(int tx0);
void UART0_rx(int rx0);


const unsigned char InitializeLCD[5] = {0x38, 0x38, 0x38, 0x0C, 0x01};

int bandera =0;
int ivDato[9];
int dato,codigo;
int dshift = 0 , dcaps = 0, dshift2 = 0;
int segundafuncion = 0;
int x=0;
int y =0;
int datos [256];
int banderapag=0;
unsigned char L;

int main(void)

{
	cfgPorts();
	UARTinit();
	initLCD();
	
	
		
	for(;;) 
	{	   
		keyboard();

	}
	
	return 0;
}


	//((GPIOC_PDIR==0x01== clock)&&(GPIOE_PDIR==0x02== datos))

void keyboard (void)
{
	while ((GPIOE_PDIR == 0x02 ))
	{
		
	}
	
	while ((GPIOC_PDIR == 0x01))
	{
		
	}
	while ((GPIOC_PDIR == 0x01))
	{			
	}
	ivDato[0] = GPIOE_PDIR/2;
	while ((GPIOC_PDIR == 0x00))
	{			
	}
	
	while ((GPIOC_PDIR == 0x01))
	{			
	}
	ivDato[1] = GPIOE_PDIR/2;
	while ((GPIOC_PDIR == 0x00))
	{			
	}
	while ((GPIOC_PDIR == 0x01))
	{		
	}
	ivDato[2] = GPIOE_PDIR/2;
	while ((GPIOC_PDIR == 0x00))
	{			
	}
	while ((GPIOC_PDIR == 0x01))
	{			
	}
	ivDato[3] = GPIOE_PDIR/2;
	while ((GPIOC_PDIR == 0x00))
	{			
	}
	
	
	while ((GPIOC_PDIR == 0x01))
	{		
	}
	ivDato[4] = GPIOE_PDIR/2;
	while ((GPIOC_PDIR == 0x00))
		{			
		}
	
	while ((GPIOC_PDIR == 0x01))
	{			
	}
	ivDato[5] = GPIOE_PDIR/2;
	while ((GPIOC_PDIR == 0x00))
		{			
		}
	
	while ((GPIOC_PDIR == 0x01))
	{		
	}
	ivDato[6] = GPIOE_PDIR/2 ;
	while ((GPIOC_PDIR == 0x00))
		{			
		}
	
	while ((GPIOC_PDIR == 0x01))
	{			
	}
	ivDato[7] = GPIOE_PDIR/2 ;
	while ((GPIOC_PDIR == 0x00))
		{			
		}
	while ((GPIOC_PDIR == 0x01))
	{			
	}
	ivDato[8] = GPIOE_PDIR/2 ;
	while ((GPIOC_PDIR == 0x00))
		{			
		}
	
	while ((GPIOC_PDIR == 0x01))
	{			
	}
	while ((GPIOC_PDIR == 0x00))
		{			
		}
	valor();
}
void valor (void)

{
	int x =1;
	int i ;
	dato = 0;
	 for(i=1;i<9;i++)
	    {                                       
	       dato = dato + (ivDato[i] )* x   ;
	       x = x*2;
	    }
	 comparar();
	 	
}

void cfgPorts(void)
{
	//Turn on clock for portd
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;	
	//Turn on clock for porte
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;	
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	/* Set pins of PORTE as GPIO */
	PORTC_PCR0=(0|PORT_PCR_MUX(1));
	PORTE_PCR1=(0|PORT_PCR_MUX(1));
	PORTB_PCR0=(0|PORT_PCR_MUX(1));
	PORTB_PCR1=(0|PORT_PCR_MUX(1));
	
	/* Set pins of PORTD as GPIO */
	PORTD_PCR0= PORT_PCR_MUX(1);
	PORTD_PCR1= PORT_PCR_MUX(1);
	PORTD_PCR2=(0|PORT_PCR_MUX(1));
	PORTD_PCR3=(0|PORT_PCR_MUX(1));
	PORTD_PCR4=(0|PORT_PCR_MUX(1));
	PORTD_PCR5=(0|PORT_PCR_MUX(1));
	PORTD_PCR6=(0|PORT_PCR_MUX(1));
	PORTD_PCR7=(0|PORT_PCR_MUX(1));
		
	//Initialize PortD 
	GPIOD_PDOR = 0x00;
	GPIOB_PDOR = 0x00;
	//Initialize PortD 
	
	//Configure PortD as outputs
	GPIOD_PDDR = 0xFF;
	GPIOB_PDDR = 0xFF;
	//Configure PortD as inputs
	GPIOE_PDDR = 0x00;
	GPIOC_PDDR = 0x00;
}
void initLCD(void)
{
	int i;
	delay(nt15_msec);
	for(i=0;i<5;i++)
	{										
		sendCode(nIns, InitializeLCD[i]);	/* send initialization instructions */			
	}
	
}
void sendCode(int Code, int Data)
{
	int i=0;
	RS_0;
	Enable_0;
	//Assign the value we want to send to the LCD
	PortLCD = Data;	
	
	//We make the algorithm to establish if its an instruction we start with 0 on RS value, otherwise if its a data command we start with RS as 1;
	if (Code == nIns)
	{
		Enable_1;
		delay(nt40_usec);
		Enable_0;
		RS_0;
	}		
	else if(Code == nData)
	{
		
		string_out[i]= Data;
		i++;
		RS_1;
		Enable_1;
		delay(nt40_usec);
		Enable_0;
		RS_0;
		
	}
}
void delay(long time)
{
	while (time > 0)
	{
		time--;
	}
}
void caps_on (void)
{
	//if((dato == 18)|(dcaps == 89)|(dshift2 == 88))
	{
		segundafuncion = 1;
	}
}
void comparar (void)
{
	if (dato == 240)
	{
		bandera =1;
		keyboard ();	
	}
	if (bandera == 1)
	{
		bandera = 0;
		keyboard ();
	}
	if((dato == 0x05)|(dato == 0x04)|(dato == 0x06)|(dato == 0x0C)|(dato == 0x03)|(dato == 0x0B)|(dato == 0x83)|(dato == 0x0A))
	{
		envpag();
	}
	
	character ();
}
void envpag (void)
{
	int i=0;
	x=0;
	sendCode(nIns, 0x01);
	banderapag=1;
	
		while(i<32)
		{
			dato = datos[i];
			delay(nt15_msec);
			character();
			i++;
		}
		banderapag = 0;
	
}
void posicion(void)
{
	if (banderapag==0)
	{
		if (x == 32)
		{
			x=0;
			sendCode(nIns, 0x01);
			//sendCode(nIns, 0x80);
		}
	}
	if (x == 0)
	{
		sendCode(nIns, 0x80);
		
	}
	if(x == 16)
	{
		sendCode(nIns, 0xC0);
		
	}
	
	
		
}
void character (void)
{
	//void caps_on();
	
	posicion ();
	x++;
	
	switch (dato)
	{
	segundafuncion =0;
		case(28):
			if (segundafuncion == 1)
			{
				
				sendCode(nData, 'A');
			}
			else 
			{
				
				sendCode(nData, 'a');
			}
			break;
		case(50):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'B');
			}
			else 
			{
				sendCode(nData, 'b');
			}
			break;
		case(33):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'C');
			}
			else 
			{
				sendCode(nData, 'c');
			}
			break;
		case(35):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'D');
			}
			else 
			{
				sendCode(nData, 'd');
			}
			break;
		case(36):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'E');
			}
			else 
			{
				sendCode(nData, 'e');
			}
			break;
		case(43):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'F');
			}
			else 
			{
				sendCode(nData, 'f');
			}
			break;
		case(52):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'G');
			}
			else 
			{
				sendCode(nData, 'g');
			}
			break;
		case(51):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'H');
			}
			else 
			{
				sendCode(nData, 'h');
			}
			break;
		case(67):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'I');
			}
			else 
			{
				sendCode(nData, 'i');
			}
			break;
		case(59):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'J');
			}
			else 
			{
				sendCode(nData, 'j');
			}
			break;
		case(66):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'K');
			}
			else 
			{
				sendCode(nData, 'k');
			}
			break;
		case(75):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'L');
			}
			else 
			{
				sendCode(nData, 'l');
			}
			break;
		case(58):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'M');
			}
			else 
			{
				sendCode(nData, 'm');
			}
			break;
		case(49):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'N');
			}
			else 
			{
				sendCode(nData, 'n');
			}
			break;
		case(68):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'O');
			}
			else 
			{
				sendCode(nData, 'o');
			}
			break;
		case(77):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'P');
			}
			else 
			{
				sendCode(nData, 'p');
			}
			break;
		case(21):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'Q');
			}
			else 
			{
				sendCode(nData, 'q');
			}
			break;
		case(45):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'R');
			}
			else 
			{
				sendCode(nData, 'r');
			}
			break;
		case(27):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'S');
			}
			else 
			{
				sendCode(nData, 's');
			}
			break;
		case(44):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'T');
			}
			else 
			{
				sendCode(nData, 't');
			}
			break;
		case(60):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'U');
			}
			else 
			{
				sendCode(nData, 'u');
			}
			break;
		case(42):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'V');
			}
			else 
			{
				sendCode(nData, 'v');
			}
			break;
		case(29):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'W');
			}
			else 
			{
				sendCode(nData, 'w');
			}
			break;
		case(34):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'X');
			}
			else 
			{
				sendCode(nData, 'x');
			}
			break;
		case(53):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'Y');
			}
			else 
			{
				sendCode(nData, 'y');
			}
			break;
		case(26):
			if (segundafuncion == 1)
			{
				sendCode(nData, 'Z');
			}
			else 
			{
				sendCode(nData, 'z');
			}
			break;
		case(0x5A):
		
		send_string_bypointer(&string_out[0],32);
		UART0_tx (0x0A);
		UART0_tx (0x0D);
		
		break;
		default:
		break;
	}
	
	if (banderapag==0)
		{
			datos[y]=dato;
			y++;
			keyboard();
		}
		
}
void UARTinit()
{
 SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;                                                   
	
 /* PORTA_PCR1: ISF=0,MUX=2 */
 PORTA_PCR1 |= (PORT_PCR_MUX(2));
 
 /* PORTA_PCR2: ISF=0,MUX=2 */
 PORTA_PCR2 |= (PORT_PCR_MUX(2));

 /* Disable TX & RX while we configure settings */
 UART0_C2 &= ~(UART0_C2_TE_MASK); //disable transmitter
 UART0_C2 &= ~(UART0_C2_RE_MASK); //disable receiver
 
 /* UART0_C1: LOOPS=0,DOZEEN=0,RSRC=0,M=0,WAKE=0,ILT=0,PE=0,PT=0 */
 UART0_C1 = 0x00U; /* Set the C1 register */
 /* UART0_C3: R8T9=0,R9T8=0,TXDIR=0,TXINV=0,ORIE=0,NEIE=0,FEIE=0,PEIE=0 */
 UART0_C3 = 0x00U; /* Set the C3 register */
 /* UART0_S2: LBKDIF=0,RXEDGIF=0,MSBF=0,RXINV=0,RWUID=0,BRK13=0,LBKDE=0,RAF=0 */
 UART0_S2 = 0x00U; /* Set the S2 register */
 
 SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1); //set clock source to be from PLL/FLL
 SIM_SOPT2 |= SIM_SOPT2_CLKOUTSEL(0b100);
 unsigned SBR = 546;//137; //Set the baud rate register, SBR = 137
 UART0_BDH |= (~UART0_BDH_SBR_MASK) | SBR >> 8;
 UART0_BDL |= (~UART0_BDL_SBR_MASK) | SBR;
 
 char OSR = 3; //set the oversampling ratio to option #3 = 4x
 UART0_C4 &= (~UART0_C4_OSR_MASK) | OSR;
 
 /*
 * Target Baud rate = 38400 9600
 *
 * Baud rate = baud clock / ((OSR+1) * SBR)
 * baud clock = FLL/PLL = 20.97152MHz  32kHZ
 * OSR = 3
 * SBR = 137 //546
 * Resulting Baud rate = 20.97152MHz / ((3 + 1) * 546) = 9600
 */
 
 UART0_C5 |= UART0_C5_BOTHEDGE_MASK; //enable sampling on both edges of the clock
 UART0_C2 |= UART0_C2_TE_MASK; //enable transmitter
 UART0_C2 |= UART0_C2_RE_MASK; //enable receiver
 
	
}


void send_string_bypointer(unsigned char *pointer_string,unsigned char limit)
{
	unsigned char i;
	unsigned char byte_out;
	
	for(i=0;i<limit;i++)
	{
		byte_out = *pointer_string;
		UART0_tx(byte_out);
		pointer_string++;
	}
}




void UART0_tx(int tx0)
{
	
	
	while ((UART0_S1 & UART_S1_TDRE_MASK) == 0);
	
	tx0 = UART0_D;		

}


void UART0_rx(int rx0)
{
	
	
	while ((UART0_S1 & UART_S1_RDRF_MASK) == UART_S1_RDRF_MASK);
	
	rx0 = UART0_D;	
	

}

