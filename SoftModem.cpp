#include "SoftModem.h"

#define TX_PIN      (3)
#define RX_PIN1     (6)  // AIN0
#define RX_PIN2     (7)  // AIN1

SoftModem *SoftModem::activeObject = 0;

SoftModem::SoftModem() {
}

SoftModem::~SoftModem() {
	end();
}

#if F_CPU == 16000000
#if SOFT_MODEM_BAUD_RATE <= 126
  #define TIMER_CLOCK_SELECT       (7)
  #define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(1024))
#elif SOFT_MODEM_BAUD_RATE <= 315
  #define TIMER_CLOCK_SELECT       (6)
  #define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(256))
#elif SOFT_MODEM_BAUD_RATE <= 630
  #define TIMER_CLOCK_SELECT       (5)
  #define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(128))
#elif SOFT_MODEM_BAUD_RATE <= 1225
  #define TIMER_CLOCK_SELECT       (4)								// set prescaler to 64
  #define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(64))  // 4µs @ 16MHz
#else
  #define TIMER_CLOCK_SELECT       (3)
  #define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(32))
#endif
#else
#if SOFT_MODEM_BAUD_RATE <= 126
  #define TIMER_CLOCK_SELECT       (6)
  #define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(256))
#elif SOFT_MODEM_BAUD_RATE <= 315
  #define TIMER_CLOCK_SELECT       (5)
  #define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(128))
#elif SOFT_MODEM_BAUD_RATE <= 630
  #define TIMER_CLOCK_SELECT       (4)
  #define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(64))
#else
  #define TIMER_CLOCK_SELECT       (3)
  #define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(32))
#endif
#endif

#define BIT_PERIOD             (1000000/SOFT_MODEM_BAUD_RATE)
#define HIGH_FREQ_MICROS       (1000000/SOFT_MODEM_HIGH_FREQ)
#define LOW_FREQ_MICROS        (1000000/SOFT_MODEM_LOW_FREQ)

#define HIGH_FREQ_CNT          (BIT_PERIOD/HIGH_FREQ_MICROS)
#define LOW_FREQ_CNT           (BIT_PERIOD/LOW_FREQ_MICROS)

#define MAX_CARRIR_BITS        (40000/BIT_PERIOD) // max preamble length = 4% of the baudrate

#define TCNT_BIT_PERIOD        (BIT_PERIOD/MICROS_PER_TIMER_COUNT)
#define TCNT_HIGH_FREQ         (HIGH_FREQ_MICROS/MICROS_PER_TIMER_COUNT)
#define TCNT_LOW_FREQ          (LOW_FREQ_MICROS/MICROS_PER_TIMER_COUNT)

#define TCNT_HIGH_TH_L         (TCNT_HIGH_FREQ * 0.90)
#define TCNT_HIGH_TH_H         (TCNT_HIGH_FREQ * 1.15)
#define TCNT_LOW_TH_L          (TCNT_LOW_FREQ * 0.85)
#define TCNT_LOW_TH_H          (TCNT_LOW_FREQ * 1.10)

#if SOFT_MODEM_DEBUG_ENABLE
static volatile uint8_t *_portLEDReg;
static uint8_t _portLEDMask;
#endif

enum { START_BIT = 0, DATA_BIT = 8, STOP_BIT = 9, INACTIVE = 0xff };

void SoftModem::begin(void)
{
	pinMode(RX_PIN1, INPUT);
	digitalWrite(RX_PIN1, LOW);

	pinMode(RX_PIN2, INPUT);
	digitalWrite(RX_PIN2, LOW);

	pinMode(TX_PIN, OUTPUT);
	digitalWrite(TX_PIN, LOW);

	_txPortReg = portOutputRegister(digitalPinToPort(TX_PIN));
	_txPortMask = digitalPinToBitMask(TX_PIN);

#if SOFT_MODEM_DEBUG_ENABLE
	_portLEDReg = portOutputRegister(digitalPinToPort(13));
	_portLEDMask = digitalPinToBitMask(13);
	pinMode(13, OUTPUT);
#endif

	_recvStat = INACTIVE;
	_recvBufferHead = _recvBufferTail = 0;

	SoftModem::activeObject = this;

	_lastTCNT = TCNT2;
	_lastDiff = _lowCount = _highCount = 0;

	TCCR2A = 0;
	TCCR2B = TIMER_CLOCK_SELECT;	   // Timer 2 in normal mode, prescaler depending on TIMER_CLOCK_SELECT
	ACSR   = _BV(ACIE) | _BV(ACIS1);   // set Analog Comparator Interrupt Enable  | trigger on falling edge
	DIDR1  = _BV(AIN1D) | _BV(AIN0D);  // Digital Input Disable, to reduce power consumption
}

void SoftModem::end(void)
{
	ACSR   &= ~(_BV(ACIE));
	TIMSK2 &= ~(_BV(OCIE2A));
	DIDR1  &= ~(_BV(AIN1D) | _BV(AIN0D));
	SoftModem::activeObject = 0;
}

void SoftModem::demodulate(void)
{
	uint8_t t = TCNT2;  		// get time in Timer 2
	uint8_t diff;
	
	if(TIFR2 & _BV(TOV2)){   	// Timer 2 overflow
		TIFR2 |= _BV(TOV2);
		diff = (255 - _lastTCNT) + t + 1;
	}
	else{
		diff = t - _lastTCNT;
	}
	
	if(diff < (uint8_t)(TCNT_HIGH_TH_L))				// Noise?
		return;
	
	_lastTCNT = t;
	
	if(diff > (uint8_t)(TCNT_LOW_TH_H))
		return;
	
//	_lastDiff = (diff >> 1) + (diff >> 2) + (_lastDiff >> 2);
	_lastDiff = diff;
	
	if(_lastDiff >= (uint8_t)(TCNT_LOW_TH_L)){   // recv low frequency signal
		_lowCount += _lastDiff;
		if((_recvStat == INACTIVE) && (_lowCount >= (uint8_t)(TCNT_BIT_PERIOD * 0.5))){ // maybe Start-Bit , now at half of start bit
			_recvStat  = START_BIT;
			_highCount = 0;
			_recvBits  = 0;
			OCR2A      = t + (uint8_t)(TCNT_BIT_PERIOD) - _lowCount; // 1 bit period after detected, set next timer2 interrupt at half TCNT_BIT_PERIOD later
			TIFR2     |= _BV(OCF2A);
			TIMSK2    |= _BV(OCIE2A);
		}
	}
	else if(_lastDiff <= (uint8_t)(TCNT_HIGH_TH_H)){  // recv high frequency signal
		_highCount += _lastDiff;
		if((_recvStat == INACTIVE) && (_highCount >= (uint8_t)(TCNT_BIT_PERIOD))){
			_lowCount = _highCount = 0;
		}
	}
}

ISR(ANALOG_COMP_vect)
{
	SoftModem::activeObject->demodulate();
}

void SoftModem::recv(void)
{
	uint8_t high;
	if(_highCount > _lowCount){
 		if(_highCount >= (uint8_t)TCNT_BIT_PERIOD)
 			_highCount -= (uint8_t)TCNT_BIT_PERIOD;
		else
			_highCount = 0;
		high = 0x80;
	}
	else{
 		if(_lowCount >= (uint8_t)TCNT_BIT_PERIOD)
 			_lowCount -= (uint8_t)TCNT_BIT_PERIOD;
 		else
			_lowCount = 0;
		high = 0x00;
	}
	
	if(_recvStat == START_BIT){	// Start bit
		if(!high){
			_recvStat++;
		}else{
			goto end_recv;
		}
	}
	else if(_recvStat <= DATA_BIT) { // Data bits
		_recvBits >>= 1;
		_recvBits |= high;
		_recvStat++;
	}
	else if(_recvStat == STOP_BIT){	// Stop bit
		uint8_t new_tail = (_recvBufferTail + 1) & (SOFT_MODEM_RX_BUF_SIZE - 1);
		if(new_tail != _recvBufferHead){
			_recvBuffer[_recvBufferTail] = _recvBits;
			_recvBufferTail = new_tail;
		}
		goto end_recv;
	}
	else{
	end_recv:
		_recvStat = INACTIVE;
		TIMSK2 &= ~_BV(OCIE2A);
	}
}

ISR(TIMER2_COMPA_vect)
{
	OCR2A += (uint8_t)TCNT_BIT_PERIOD;
	SoftModem::activeObject->recv();
#if SOFT_MODEM_DEBUG_ENABLE
	*_portLEDReg ^= _portLEDMask;
#endif  
}

int SoftModem::available()
{
	return (_recvBufferTail + SOFT_MODEM_RX_BUF_SIZE - _recvBufferHead) & (SOFT_MODEM_RX_BUF_SIZE - 1);
}

int SoftModem::read()
{
	if(_recvBufferHead == _recvBufferTail)
		return -1;
	int d = _recvBuffer[_recvBufferHead];
	_recvBufferHead = (_recvBufferHead + 1) & (SOFT_MODEM_RX_BUF_SIZE - 1);
	return d;
}

int SoftModem::peek()
{
	if(_recvBufferHead == _recvBufferTail)
		return -1;
	return _recvBuffer[_recvBufferHead];
}

void SoftModem::flush()
{
	_recvBufferHead = _recvBufferTail = 0;
}

void SoftModem::modulate(uint8_t b)
{
	uint8_t cnt,tcnt,tcnt2;
	if(b){
		cnt = (uint8_t)(HIGH_FREQ_CNT);
		tcnt2 = (uint8_t)(TCNT_HIGH_FREQ / 2);
		tcnt = (uint8_t)(TCNT_HIGH_FREQ) - tcnt2;
	}else{
		cnt = (uint8_t)(LOW_FREQ_CNT);
		tcnt2 = (uint8_t)(TCNT_LOW_FREQ / 2);
		tcnt = (uint8_t)(TCNT_LOW_FREQ) - tcnt2;
	}
	do {
		cnt--;
		{
			OCR2B += tcnt;
			TIFR2 |= _BV(OCF2B);	// clears the Output Compare Flag 2B
			while(!(TIFR2 & _BV(OCF2B)));	// wait until compare match occurs
		}
		*_txPortReg ^= _txPortMask;		// toggle output
		{
			OCR2B += tcnt2;
			TIFR2 |= _BV(OCF2B);
			while(!(TIFR2 & _BV(OCF2B)));
		}
		*_txPortReg ^= _txPortMask;
	} while (cnt);
}

//  Brief carrier tone before each transmission
//  1 start bit (LOW)
//  8 data bits, LSB first
//  1 stop bit (HIGH)
//  ...
//  1 push bit (HIGH)

size_t SoftModem::write(const uint8_t *buffer, size_t size)
{
    uint8_t cnt = ((micros() - _lastWriteTime) / BIT_PERIOD) + 1;
    if(cnt > MAX_CARRIR_BITS)
        cnt = MAX_CARRIR_BITS;
    for(uint8_t i = 0; i<cnt; i++)
        modulate(HIGH);
    size_t n = size;
    while (size--) {
        uint8_t data = *buffer++;
        modulate(LOW);							 // Start Bit
        for(uint8_t mask = 1; mask; mask <<= 1){ // Data Bits
            if(data & mask){
                modulate(HIGH);
            }
            else{
                modulate(LOW);
            }
        }
        modulate(HIGH);				// Stop Bit
    }
	modulate(HIGH);				// Push Bit
    _lastWriteTime = micros();
    return n;
}

size_t SoftModem::write(uint8_t data)
{
    return write(&data, 1);
}
