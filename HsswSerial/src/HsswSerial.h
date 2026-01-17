//
//	HsswSerial (High Speed SoftWare Serial) library
//		(c) 2019	1YEN Toru
//
//	occupied resources:
//		D8(PB0): for RXD input
//		D9(PB1): for TXD output
//		TC1(16 bit timer): PWM output for OC1A(D9) and OC1B(D10) do not work.
//		TIMER1_CAPT_vect: for receive
//
//
//	2020/06/20	ver.1.14
//		write(): interrupt disable / recovery
//
//	2019/06/29	ver.1.12
//		You can select RTS and CTS pin from any digital pin.
//		begin (long baud, int opt, int pin_rts, int pin_cts);
//
//	2019/04/20	ver.1.10
//		change RXD: PCINT0 to TC1 input capture interrupt
//
//	2019/04/13	ver.1.00
//


#ifndef		HsswSerial_h
#define		HsswSerial_h


#include	<inttypes.h>
#include	<Stream.h>


#define		Hssw_VERS	114
#ifndef		Hssw_RX_BUF_MAX
#define		Hssw_RX_BUF_MAX		64		// size of rx buffer (<256)
#endif	//	Hssw_RX_BUF_MAX


// pin assign
const	int		Hssw_pin_RXD=8;			// RXD
const	int		Hssw_pin_TXD=9;			// TXD
const	int		Hssw_pin_RTS=10;		// RTS (optional, default pin)
const	int		Hssw_pin_CTS=11;		// CTS (optional, default pin)
const	int		Hssw_bit_RXD=0x01;		// RXD: PB0
const	int		Hssw_bit_TXD=0x02;		// TXD: PB1
volatile	uint8_t	*const	Hssw_reg_RXD=(uint8_t *)0x0023;		// RXD: pinb
volatile	uint8_t	*const	Hssw_reg_TXD=(uint8_t *)0x0025;		// TXD: portb

// memory mapped i/o registers
volatile	uint8_t		*const	Hssw_tifr1=(uint8_t *)0x0036;
volatile	uint8_t		*const	Hssw_timsk1=(uint8_t *)0x006f;
volatile	uint8_t		*const	Hssw_tccr1a=(uint8_t *)0x0080;
volatile	uint8_t		*const	Hssw_tccr1b=(uint8_t *)0x0081;
volatile	uint8_t		*const	Hssw_tccr1c=(uint8_t *)0x0082;
volatile	uint16_t	*const	Hssw_tcnt1=(uint16_t *)0x0084;
volatile	uint16_t	*const	Hssw_icr1=(uint16_t *)0x0086;


enum	HsswParam
{
	// HsswSerial.begin() opt
	// 0b_0000_0ncr_00pp_sbb0:
	//		bb: bits, 11=8 bits / 10=7 bits / 01=6 bits / 00=5 bits
	//		s: stop, 0=1 bit / 1=2 bits
	//		pp: parity, 00=N / 10=E / 11=O / 01=reserved
	//		r: RTS, enable bit
	//		c: CTS, enable bit
	//		n: RTS & CTS, negative logic
	Hssw_SERIAL_MSK=0x003e,				// frame format mask
	Hssw_RTS_ENBL=0x0100,				// RTS flow control enable
	Hssw_CTS_ENBL=0x0200,				// CTS flow control enable
	Hssw_LGC_NEGA=0x0400,				// RTS & CTS negative logic
	// error code
	HsswErrNo=0,						// no error
	HsswErrBaud=-1,						// bad baud rate
	HsswErrBadParam=-2,					// bad parameter
	HsswErrRecv=-3,						// receive error
	HsswErrOvfl=-4,						// receive buffer overflow
	HsswErrFrame0=-5,					// bad frame (start bit is not 0)
	HsswErrFrame1=-6,					// bad frame (stop bit is not 1)
};


class	HsswSerial : public Stream
{
	public:

	void	begin (
	long	baud,
	int		opt=SERIAL_8N1,
	int		pin_rts=Hssw_pin_RTS,
	int		pin_cts=Hssw_pin_CTS);

	virtual		int		available (void);
	virtual		int		peek (void);
	virtual		int		read (void);
	virtual		int		availableForWrite (void);
	virtual		void	flush (void);
	virtual		size_t	write (uint8_t);

	inline	size_t	write (
	unsigned	long	n)
	{
		return	(write (uint8_t (n)));
	}

	inline	size_t	write (
	long	n)
	{
		return (write (uint8_t (n)));
	}

	inline	size_t	write (
	unsigned	int		n)
	{
		return  (write (uint8_t (n)));
	}

	inline	size_t	write (
	int		n)
	{
		return (write (uint8_t (n)));
	}

	inline	void	Hssw_negate_rts (void)
	{
		// negate RTS signal, if RTS enabled

		if (rts_enb)
		{
			if (!lgc_neg)
				digitalWrite (rts_pin, LOW);
			else
				digitalWrite (rts_pin, HIGH);
		}
	}

	inline	void	Hssw_assert_rts (void)
	{
		// assert RTS signal, if RTS enabled
		if (rts_enb)
		{
			if (lgc_neg)
				digitalWrite (rts_pin, LOW);
			else
				digitalWrite (rts_pin, HIGH);
		}
	}

	void	Hssw_auto_rts (
	int		thsh=Hssw_RX_BUF_MAX/2);

    using	Print::write;

	operator	bool (void)
	{
		return ((errcod==HsswErrNo && prescl>0)? true: false);
	}

	int		errcod;						// error code
	char	rts_enb;					// RTS enable
	char	cts_enb;					// CTS enable
	char	lgc_neg;					// RTS & CTS negative logic
	int		rts_pin;					// RTS pin #
	int		cts_pin;					// CTS pin #
	int		prescl;						// TC1 count clock prescaler
	uint16_t	cnt_frame;				// TC1 count per frame
	uint32_t	baud_rate;				// baud rate
	uint8_t		rx_buf_wr;				// write pointer for rx_buf[]
	uint8_t		rx_buf_rd;				// read pointer for rx_buf[]
	uint8_t		rx_buf[Hssw_RX_BUF_MAX];	// ring buffer for receive data
	uint16_t	tx_cnt[10];				// TXD output interval in TC1 counter
	uint16_t	rx_cnt[10];				// RXD sampling time in TC1 counter
};


extern	HsswSerial	SerialHssw;


#endif	//	HsswSerial_h
