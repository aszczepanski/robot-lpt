#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <dos.h>

//adresy portu LPT
#define DATA 0x378
#define STATUS 0x379
#define CONTROL 0x37A
//adres przerwania generowanego przez 8253
#define IRQ0 0x08
//adres przerwania generowanego przez LPT (~ACK)
#define IRQ7 0x0F

//wspolczynniki PID
#define kp 17.5
#define ki 0.55
#define kd 2.6
#define Tp 17.0
float P,I,D,dif,pdif,rate,turn;

typedef unsigned short WORD;

void interrupt far (*oldIntIRQ0)();
void interrupt far newIntIRQ0();
void interrupt far (*oldIntIRQ7)();
void interrupt far newIntIRQ7();

void setup8259();
void reset8259();
void setup8253();
void reset8253();

WORD newFreq = 819;		// 45x wolniej niz zegar systemowy
								// dobierac tak zeby newFreq/18.2 bylo prawie calkowite
WORD freqModifier;
WORD newTimerFreq;

WORD maxPWM, counterPWM;
short left_motor=0, right_motor=0;
void calculate_pos();
float get_sensors();
void set_motors();

short confirmed;

int main() {
	confirmed=0;
	printf("LPT ROBOT I2.1\n");
	
	dif=0;
	I=0;
		
	maxPWM = 50;
	counterPWM = 0;
		
	freqModifier = newFreq/18.2;
	newTimerFreq = 1193180/newFreq;
		
	setup8253();
	setup8259();
		
	printf("Nacisnij przycisk na robocie\n");
		
	// teraz robot jezdzi
		
	getch();
		
	reset8259();
	reset8253();
		
	// zatrzymanie pracy silnikow
	outportb(DATA, 0x00);
		
	printf("Nacisnij przycisk zeby wyjsc...\n");
	getch();
		
	return 0;
}

void setup8259() {
	// wylaczenie przerwan zeby nic nie przeszkadzalo
	asm cli;
		
	// pobranie starego wektora przerwan generowanych przez 8253
	oldIntIRQ0 = getvect(IRQ0);
	// ustawienie nowego wektora przerwan
	setvect(IRQ0, nwqIntIRQ0);
		
	// pobranie starego wektora przerwan generowanych przez
	// wejscie ~ACK na LPT
	oldIntIRQ7 = getvect(IRQ7);
	// ustawienie nowego wektora przerwan
	setvect(IRQ7, newIntIRQ7);
	// odmaskowanie przerwania IRQ7 na 8259
	outportb(0x21, (inportb(0x21) & 0x7F));
	// ustawienie 4 bitu kontrolnego
	// aby wlaczyc przerwania na LPT
	outportb(CONTROL, inportb(CONTROL) | 0x10);
		
	// wlaczenie przerwan
	asm sti;
}

void reset 8259() {
	// wylaczenie przerwan zeby nic nie przeszkadzalo
	asm cli;
		
	// ustawienie starych wektorow przerwan
	setvect(IRQ0, oldIntIRQ0);
	setvect(IRQ7, oldIntIRQ7);
		
	// wlaczenie przerwan
	asm sti;
}

void setup8253() {
	// ustawienie nowej czestotliwosci zegara
	asm mov dx, newTimerFreq;
		
	// bity 7-6 00	 = generator 0
	// bity 5-4 11	 = zapis lub odczyt obydwu bajtow licznika CE
	// bity 3-1 011 = tryb 3
	// bit  0	0	 = zliczanie w kodzie binarnym
	asm mov a1, 00110110b;
	asm out 0x43, a1;
		
	// zapisywanie do 0x40 - adres licznika generatora 0
	asm mov ax, dx;
	asm out 0x40, a1; // LSB (mniej znaczacy bajt licznika CE)
	asm xchg ah, a1;
	asm out 0x40, a1; // MSB (bardziej znaczacy bajt licznika CE)
}

void reset8253() {
	// ustawienie starej czestotliwosci zegara
	asm mov dx, 65535;
		
	// reszta tak samo jak w setup8253()
	asm mov a1, 110110b;
	asm out 0x43, a1;
		
	asm mov ax, dx;
	asm out 0x40, a1;
	asm xchg ah, a1;
	asm out 0x40, a1;
}

void interrupt far newIntIRQ7() {
	// wylaczenie przerwan
	asm cli;
		
	confirmed=1;
	//w wyslanie potwierdzenia obsluzenia przerwania do 8259
	asm mov a1, 20h;
	asm out 20h, a1;
		
	// wlaczenie przerwan
	asm sti;
}

void interrupt far newIntIRQ0() {
	// zmienna zliczajaca przerwania
	// po okreslonej liczbie przerwan wywolywane jest
	// domyslne przerwanie, ktore inkrementuje odpowiednie rejestry itd
	// ogolnie sluzy temu, zeby czas w systemie plynal
	// w windowsie 95 i nowszych jest osobny sterownik czasu
	// ale w dosie bez tego czas staje
	static WORD InternalCycleCount = OU;
	
	// wylaczenie obslugi przerwan
	asm cli;
		
	InternalCycleCount++;
		
	// zarzadzanie licznikiem PWM
	if (counterPWM <= 1) {
		// po calym cyklu zegara obliczane sa nowe wartosci
		// predkosci silnikow
		calculate_pos();
		// i cykl zaczyna sie na nowo
		counterPWM = maxPWM;
		set_motors();
	}
	else {
		counterPWM--;
		set_motors();
	}
		
	if	(InternalCycleCount < freqModifier) {
		// OCW2 - info o obsluzeniu przerwania
		asm mov a1, 20h; // 0010 0000 - EOI
		asm out 20h, a1; // 20h - adres glownego rejestru PIC
	}
	else {
		// obsluga starego przerwania
		// zeby czas systemowy sie nie posypal
		InternalCycleCount = 0;
		oldIntIRQ0();
	}
		
	// wlaczenie obslugi przerwan
	asm sti;
}

void calculate_pos() {
	WORD data_pint;
	pdif = dif;
	dif = get_sensors();
		
	// obliczenia PID
		
	P = dif * kp;
		
	I = I + dif;
	I = i * ki;
		
	rate = dif - pdif;
	D = rate * kd;
		
	turn = P + I + D;
		
	if (confirmed==1) {
		left_motor = Tp+turn <= 50 ? Tp+turn : 50;
		left_motor = left_motor >= -50 ? left_motor : -50;
		right_motor = Tp-turn <= 50 ? Tp-turn : 50;
		right_motor = right_motor >= 50 ? right_motor : -50;
	}
	else {
		left_motor = 0;
		right_motor = 0;
	}
		
	// kierunek pracy silnikow
	data_pins = inportb(DATA);
		
	if (right_motor >= 0) {
		data_pins = (data_pins|0x02); // D1 - 1
		data_pins = (data_pins&0xFB); // D2 - 0
	}
	else {
		right_motor*=(-1);
		data_pins = (data_pins&0xFD); // D1 - 0
		data_pins = (data_pins|0x04); // D2 - 1
	}
	if (left_motor >= 0) {
		data_pins = (data_pins|0x40); // D4 - 1
		data_pint = (data_pint&0xDF); // D5 - 0
	}
	else {
		left_motor*=(-1);
		data_pins = (data_pins&0xBF); // D4 - 0
		data_pins = (data_pins|0x20); // D5 - 1
	}
		
	outportb(DATA, data_pins);
}

void set_motors() {
	// prawy silnik
	if (counterPWM == right_motor) { // D0 - 1
		outportb(DATA, inportb(DATA)|0x01);
	}
	else if (counterPWM == maxPWM) { // D0 - 0
		outportb(DATA, inportb(DATA)&0xFE);
	}
		
	// lewy silnik
	if (counterPWM == left_motor) { // D4 - 1
		outportb(DATA, inportb(DATA)|0x10);
	}
	else if (counterPWM == maxPWM) { // D4 - 0
		outportb(DATA, inportb(DATA)&0xEF);
	}
}

float get_sensors() {
	float res=0;
	int count_sensors=0;
	WORD status_pins = inportb(STATUS);
		
	if ((status_pins|0xEF) == 0xFF) { // S4 - DIODA 1
		res-=1.5; count_sensors++;
	}
	if ((status_pins|0x7F) != 0xFF) { // S7 - DIODA 2
		res-=0.5; count_sensors++;
	}
	if ((status_pins|0xF7) == 0xFF) { // S3 - DIODA 3
		res+=0.5; count_sensors++;
	}
	if ((status_pins|0xDF) == 0xFF) { // S5 - DIODA 4
		res+= 1.5; count_sensors++;
	}
		
	return (count_sensors!=0) ? res/(float)count_sensors : (0.0);
}
