 /*
 Proyecto Microcontroladores y Electronica de Potencia
 Julio J. da Costa Neto
 */

#define	F_CPU 16000000
#define brate0 9600

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include <util/twi.h>

//Variables posicion, posicion anterior, periodos y activado
volatile int8_t Act=-1;
volatile uint8_t Accion=0, Etapa=0, EtapaSeg=0;
volatile int16_t Pos=0, Pos_ant=0;
volatile uint16_t T=0, TP=0, TE=0;

//Define estados e registro
enum tEstado{D, A, h, E, p, P} estado;

//Define rango de las variables
#define T_MIN 1
#define P_MIN 0
#define E_MIN 1
#define B_MIN 0
#define T_MAX 9999
#define P_MAX 9999
#define E_MAX 255
#define B_MAX 5

#define LC256_DIR 0xA0
#define BYTES_ETAPA 10

#define I2C_READ 0x01
#define I2C_WRITE 0x00

#define F_SCL 100000 // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

 //Define variables para buffer
uint8_t indcom, dato[BYTES_ETAPA];
char comando[30];

 void i2c_init(void)
 {
     TWBR = (uint8_t)TWBR_val;
 }

 uint8_t i2c_start(uint8_t address)
 {
     // reset TWI control register
     TWCR = 0;
     // transmit START condition
     TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

     // wait for end of transmission
     while( !(TWCR & (1<<TWINT)) );

     // check if the start condition was successfully transmitted
     if((TWSR & 0xF8) != TW_START){ return 1; }

     // load slave address into data register
     TWDR = address;
     // start transmission of address
     TWCR = (1<<TWINT) | (1<<TWEN);
     // wait for end of transmission
     while( !(TWCR & (1<<TWINT)) );

     // check if the device has acknowledged the READ / WRITE mode
     uint8_t twst = TW_STATUS & 0xF8;

     if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
     return 0;
 }

 void i2c_stop(void)
 {
     // transmit STOP condition
     TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
     _delay_ms(5);
 }

 uint8_t i2c_write(uint8_t data)
 {
     // load data into data register
     TWDR = data;
     // start transmission of data
     TWCR = (1<<TWINT) | (1<<TWEN);
     // wait for end of transmission
     while( !(TWCR & (1<<TWINT)) );

     if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }

     return 0;
 }

 uint8_t i2c_read_ack(void)
 {

     // start TWI module and acknowledge data after reception
     TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
     // wait for end of transmission
     while( !(TWCR & (1<<TWINT)) );
     // return received data from TWDR
     return TWDR;
 }

 uint8_t i2c_read_nack(void)
 {

     // start receiving without acknowledging reception
     TWCR = (1<<TWINT) | (1<<TWEN);
     // wait for end of transmission
     while( !(TWCR & (1<<TWINT)) );
     // return received data from TWDR
     return TWDR;
 }

 uint8_t i2c_writeReg(uint8_t devaddr, uint16_t regaddr, uint8_t* data, uint16_t length)
 {
     if (i2c_start(devaddr | I2C_WRITE)) return 1;

     i2c_write((regaddr >> 8) & 0xff);
     i2c_write(regaddr & 0xff);

     for (uint16_t i = 0; i < length; i++)
     {
         if (i2c_write(data[i])) return 1;
     }

     i2c_stop();

     return 0;
 }

 uint8_t i2c_readReg(uint8_t devaddr, uint16_t regaddr, uint8_t* data, uint16_t length)
 {
     if (i2c_start(devaddr | I2C_WRITE)) return 1;

     i2c_write((regaddr >> 8) & 0xff);
     i2c_write(regaddr & 0xff);

     if (i2c_start(devaddr | I2C_READ)) return 1;

     for (uint16_t i = 0; i < (length-1); i++)
     {
         data[i] = i2c_read_ack();
     }
     data[(length-1)] = i2c_read_nack();

     i2c_stop();

     return 0;
 }

//----------------------------------------------------
//      Definiciones siguientes son para usar getc y putc para E/S de caracteres por UART
int mi_putc(char, FILE *stream), mi_getc(FILE *stream);
#define getc() mi_getc(&uart_io)		// redefine la primitiva de entrada como  funcion recibir por UART
#define putc(x) mi_putc(x,&uart_io)		// redefine la primitiva de salida como funcion transmitir por UART
//      Declara un tipo stream de E/S
FILE uart_io = FDEV_SETUP_STREAM(mi_putc, mi_getc, _FDEV_SETUP_RW);
//		Inicializa UART
void mi_UART_Init(unsigned int ubrr)
{
    UBRR0 = F_CPU/16/ubrr-1;				// Configura baudrate.
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);			// Habilita bits TXEN0 y RXEN0
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);		// USBS0=1 2 bits stop, UCSZxx=3 8 bits
}
//		Pone caracter en UART. *stream es un parametro usado solo para igualar los parametros en stdio
int mi_putc(char c, FILE *stream)
{
	while(!(UCSR0A & (1<<UDRE0)));  // Espera mientras el bit UDRE0=0 (buffer de transmision ocupado)
    UDR0 = c;						// Cuando se desocupa, UDR0 puede recibir el nuevo dato c a trasmitir
	return 0;
}
//	    Recibe caracter de la UART. *stream es un parametro usado solo para igualar los parametros en stdio
int mi_getc(FILE *stream)
{
	while (!(UCSR0A & (1<<RXC0)));  // Espera mientras el bit RXC0=0 (recepcion incompleta)
	return UDR0;					// Cuando se completa, se lee UDR0
}
//----------------------------------------------------
//		Funcion delay
 void mi_delay(int ms)			// Delay con argumento variable
 {
     int t;
     for(t=0;t<ms;t++)
         _delay_ms(1);
 }

 uint8_t borrarTodo() {
     printf(":BT...");
     for (int i=0; i<500; i+=64) {
         if (i2c_start(LC256_DIR) || i2c_write((i >> 8) & 0xff) || i2c_write(i & 0xff)) return 1;
         for (int j; j<64; j++)
            if (i2c_write(0)) return 1;
         i2c_stop();
     }
     printf("ok\r\n");
     return 0;
 }
 uint8_t borrarEtapa(uint8_t num) {
     printf(":B%d...", num);
     if (i2c_start(LC256_DIR) || i2c_write((((num-1)*BYTES_ETAPA) >> 8) & 0xff) ||
             i2c_write(((num-1)*BYTES_ETAPA) & 0xff)) return 1;
     for (int j = 0; j < BYTES_ETAPA; j++) {
         if (i2c_write(0)) return 1;
     }
     i2c_stop();
     printf("ok\r\n");
     return 0;
 }
 uint8_t grabar() {
     if (Etapa==0 || TP==0 || TE==0 || EtapaSeg==0) return 1;

     dato[0] = Etapa;
     dato[1] = (Pos >> 8) & 0xff;
     dato[2] = Pos & 0xff;
     dato[3] = (TP >> 8) & 0xff;
     dato[4] = TP & 0xff;
     dato[5] = Accion;
     dato[6] = (TE >> 8) & 0xff;
     dato[7] = TE & 0xff;
     dato[8] = EtapaSeg;
     dato[9] = 0;

     if (i2c_start(LC256_DIR) || i2c_write((((Etapa-1)*BYTES_ETAPA) >> 8) & 0xff) ||
             i2c_write(((Etapa-1)*BYTES_ETAPA) & 0xff)) return 1;
     for (int j=0; j<BYTES_ETAPA; j++)
         if (i2c_write(dato[j])) return 1;
     i2c_stop();
     _delay_ms(10);
     return 0;
 }
//		Analiza los elementos del buffer de recepcion
void InterpretaComando()
{
    int aux=0;
    switch(comando[0]) {                    // Analiza primer Byte del buffer.
        case 'T':                            // T modifica tiempos etapa
            if (Etapa!=0) {
                switch (comando[1]) {
                    case 'P':
                        aux = atoi(&comando[2]);              // Convierte cadena decimal en entero
                        if (aux >= T_MIN && aux <= T_MAX)
                            TP = aux;                         // evalua rango, asigna a TP
                        printf(":TP%d\r\n", TP);       // imprime tiempo informado
                        break;
                    case 'E':
                        aux = atoi(&comando[2]);
                        if (aux >= T_MIN && aux <= T_MAX)
                            TE = aux;                         // evalua rango, asigna a TE
                        printf(":TE%d\r\n", TE);       // imprime tiempo informado
                        break;
                    default:
                        break;
                }
            } else
                printf("Error, etapa=0.\r\n");
            break;
        case 'P':                                           // P modifica posicion etapa
            if (Etapa!=0) {
                aux = atoi(&comando[1]);
                if (aux >= P_MIN && aux <= P_MAX)
                    Pos = aux;                                  // evalua rango, asigna a P
                printf(":P%d\r\n", Pos);
            } else
                printf("Error, etapa=0.\r\n");
            break;
        case 'S':                                           // S modifica etapa seguiente
            if (Etapa!=0) {
                aux = atoi(&comando[1]);
                if (aux >= E_MIN && aux <= E_MAX)
                    EtapaSeg = aux;                                  // evalua rango, asigna a etapaSeg
                printf(":S%d\r\n", EtapaSeg);
            } else
                printf("Error, etapa=0.\r\n");
            break;
        case 'R':                                           // R modifica status sistema
            if (atoi(&comando[1]) == 1 && estado == D) {    // activa sistema
                Act = 1;
                printf(":R%d\r\n", Act);
                estado = A;
            } else if (atoi(&comando[1]) == -1) {            // desactiva sistema
                Etapa = 0;
                Act = -1;
                printf(":R%d\r\n", Act);
                Pos = Pos_ant;
                estado = D;
            } else
                printf(":R%d\r\n", Act);
            break;
        case 'E':
            if (estado != D)
                printf(":E%d\r\n", Etapa);        // imprime etapa actual, si en ejecucion
            break;
        case 'B':                           // B borrar etapas
            if (estado == D) {
                switch (comando[1]) {
                    case 'T':                   // borrar todas etapas
                        if (borrarTodo()) printf("Erro BT\r\n");
                        break;
                    default:                    // borrar etapa X
                        aux = atoi(&comando[1]);              // Convierte cadena decimal en entero
                        if (aux >= E_MIN && aux <= E_MAX)
                            if (borrarEtapa(aux)) printf("Erro B%d\r\n", aux); // evalua rango, borra etapa informada
                        break;
                }
            } else
                printf("Error, en ejecucion.\r\n");
            break;
        case 'N':                           // N nueva etapa
            if (estado == D) {
                aux = atoi(&comando[1]);              // Convierte cadena decimal en entero
                if (aux >= E_MIN && aux <= E_MAX)
                    Etapa = aux;                        // evalua rango, asigna a Etapa
                printf(":N%d\r\n", Etapa);
                Pos = 0;                                // borra variables temporales
                TP = 0;
                Accion = 0;
                TE = 0;
                EtapaSeg = 0;
            } else
                printf("Error, en ejecucion.\r\n");
            break;
        case 'A':                           // A accionamiento puerta X(solo en PORTB)
            if (Etapa!=0) {
                aux = atoi(&comando[1]);              // Convierte cadena decimal en entero
                if (aux >= B_MIN && aux <= B_MAX) {
                    Accion = aux;                         // evalua rango, asigna a Accion
                }
                printf(":AB%d\r\n", Accion);
            } else
                printf("Error, etapa=0.\r\n");
            break;
        case 'G':                           // G graba variables en la memoria
            if (Etapa!=0)
                if (grabar()) printf("Erro grabando\r\n");
                else printf(":G\r\n");
            else
                printf("Error, etapa=0.\r\n");
            break;
        default:
		    break;
	}
}
//      Programa main
int main()
{
	int dif;

    i2c_init();
    mi_UART_Init(brate0);           // Inicializa UART
	stdout = stdin = &uart_io;

    DDRD &= ~(1<<2);
	DDRD |= (1<<5)|(1<<6)|(1<<7);
    DDRB  = 63;                     // Puertas 0 a 5 salida

    // PORTD2 -> INT0 FINAL DE CARRERA
    // PORTD5 -> PULSO
    // PORTD6 -> ENABLE
    // PORTD7 -> DIRECCION
    // PORTBX -> ACCIONAMIENTOS

	EIMSK=(1<<INT0);	    // INT0 interrupcion
	EICRA=(3<<ISC00);	    // Flanco de subida
    UCSR0B |= (1<<RXCIE0);	// Interrupcion Rx UART0

    indcom=0;
    estado=D;               // Estado inicial
    Pos=-1;                // Eje no referenciado


    sei();					// Interrupcion global

    while(1) {
        switch (estado) {
            case D:
                PORTD |= (1<<6);                     // desabilita driver
                PORTD &= ~(1<<7);
                PORTD &=~(1<<5);
                break;
            case A:
                PORTD &= ~(1<<6);                    // habilita driver
                estado=h;
                break;
            case h:
                printf(":h\r\n");

                T=50;
                PORTD &= ~(1<<7);                   // cambio direccion motor
                while (Pos!=0) {                    // pulso mientras no sea activada la int0 final de carrera
                    if (estado!=h) break;
                    PORTD |=(1<<5);
                    mi_delay(T);
                    PORTD &=~(1<<5);
                    mi_delay(T);
                }
                if (estado!=h) break;

                _delay_ms(100);                     // delay fijo
                T=100;                              // tiempo de avance lento fijo
                PORTD |= (1<<7);                    // cambio direccion motor
                while (PIND&(1<<PD2)) {             // pulso mientras salga del fin de carrera (tension baja)
                    if (estado!=h) break;
                    PORTD |=(1<<5);
                    mi_delay(T);
                    PORTD &=~(1<<5);
                    mi_delay(T);
                }
                if (estado!=h) break;

                printf(":H\r\n");
                Etapa = 1;
                estado=E;
                break;
            case E:
                //busca info da etapa
                i2c_readReg(LC256_DIR, (Etapa-1)*BYTES_ETAPA, dato, BYTES_ETAPA);

                if (Etapa!=dato[0]) {
                    printf("Erro etapa: %d %d\r\n", Etapa, dato[0]);
                    estado = D;
                } else {

                    Etapa = dato[0];
                    Pos = ((uint16_t)dato[1] << 8) | dato[2];
                    TP = ((uint16_t)dato[3] << 8) | dato[4];
                    Accion = dato[5];
                    TE = ((uint16_t)dato[6] << 8) | dato[7];
                    EtapaSeg = dato[8];

                    estado = P; //inicia movimiento
                }
                break;
            case P:
                if (Pos!=Pos_ant)                   // si posicion nueva, cambia estado para 'realizando posicionamento'
                    estado=p;
                else {
                    PORTB |=(1<<Accion);            // inicia accionamiento auxiliar
                    mi_delay(TE);                   // delay tiempo espera
                    PORTB &= ~(1<<Accion);          // termina accionamiento auxiliar
                    Etapa = EtapaSeg;
                    estado = E;                       // si ya esta en la posicion, cambia busca proxima etapa
                }
                break;
            case p:
                T=TP;

                if (Pos > Pos_ant) {
                    dif = 1;
                    PORTD |= (1<<7);                // cambio direccion motor
                } else if (Pos < Pos_ant) {
                    dif = -1;
                    PORTD &= ~(1<<7);               // cambio direccion motor
                } else
                    dif = 0;

                while (Pos_ant != Pos) {                   // pulso si la posicion actual es distinta de la deseada
                    PORTD |=(1<<5);
                    mi_delay(T);
                    PORTD &=~(1<<5);
                    mi_delay(T);

                    if (estado!=p) break;
                    Pos_ant += dif;
                }
                if (estado!=p) break;
                estado=P;
                break;
            default:
                break;
        }
    }
 }
//----------------------------------------------------
//      Rutina de Servicio de Interrupcion de UART
ISR(USART_RX_vect)
{
    char dato;
    dato=getc();
    switch (dato) {
        case ':':                   // Delimitadores de inicio
        case '/':
            comando[0] = 0;
            indcom = 0;             // Inicializa indice de buffer de recepcion
            break;
        case ';':                   // Delimitadores de final
        case '\r':
            comando[indcom] = 0;    // coloca \0 luego del ultimo caracter recibido antes de \r
            InterpretaComando();    // Llama a funcion interprete de comandos
            break;
        default:
            comando[indcom++] = dato; // Guarda en elemento del buffer, si la direccion es correcta
            break;
    }
}
//      Rutina de Servicio de Interrupcion de INT0(fin de carrera)
ISR(INT0_vect)
{
    Pos_ant = 0;
    Pos = 0;
}
//----------------------------------------------------
