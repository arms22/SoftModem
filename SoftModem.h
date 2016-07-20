#ifndef SoftModem_h
#define SoftModem_h

#include <Arduino.h>

//#define SOFT_MODEM_BAUD_RATE   (126)
//#define SOFT_MODEM_LOW_FREQ    (882)
//#define SOFT_MODEM_HIGH_FREQ   (1764)
//#define SOFT_MODEM_RX_BUF_SIZE (4)

//#define SOFT_MODEM_BAUD_RATE   (315)
//#define SOFT_MODEM_LOW_FREQ    (1575)
//#define SOFT_MODEM_HIGH_FREQ   (3150)
//#define SOFT_MODEM_RX_BUF_SIZE (8)

//#define SOFT_MODEM_BAUD_RATE   (630)
//#define SOFT_MODEM_LOW_FREQ    (3150)
//#define SOFT_MODEM_HIGH_FREQ   (6300)
//#define SOFT_MODEM_RX_BUF_SIZE (16)

#define SOFT_MODEM_BAUD_RATE   (1225)
#define SOFT_MODEM_LOW_FREQ    (4900)
#define SOFT_MODEM_HIGH_FREQ   (7350)
#define SOFT_MODEM_RX_BUF_SIZE (32)

//#define SOFT_MODEM_BAUD_RATE   (2450)
//#define SOFT_MODEM_LOW_FREQ    (4900)
//#define SOFT_MODEM_HIGH_FREQ   (7350)
//#define SOFT_MODEM_RX_BUF_SIZE (32)

#define SOFT_MODEM_DEBUG_ENABLE  (0)

class SoftModem : public Stream
{
private:
	volatile uint8_t *_txPortReg;
	uint8_t _txPortMask;
	uint8_t _lastTCNT;
	uint8_t _lastDiff;
	uint8_t _recvStat;
	uint8_t _recvBits;
	uint8_t _recvBufferHead;
	uint8_t _recvBufferTail;
	uint8_t _recvBuffer[SOFT_MODEM_RX_BUF_SIZE];
	uint8_t _lowCount;
	uint8_t _highCount;
    unsigned long _lastWriteTime;
	void modulate(uint8_t b);
public:
	SoftModem();
	~SoftModem();
	void begin(void);
	void end(void);
	virtual int available();
	virtual int read();
	virtual void flush();
	virtual int peek();
    virtual size_t write(const uint8_t *buffer, size_t size);
	virtual size_t write(uint8_t data);
	void demodulate(void);
	void recv(void);
	static SoftModem *activeObject;
};

#endif
