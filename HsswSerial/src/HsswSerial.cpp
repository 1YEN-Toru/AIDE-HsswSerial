//
//	HsswSerial (High Speed SoftWare Serial) library
//		(c) 2019	1YEN Toru
//


#include	<Arduino.h>
#include	"HsswSerial.h"


#ifdef		__AVR_ATmega328P__
#else	//	__AVR_ATmega328P__
#error		"HsswSerial class library does not work except ATmega328P."
#endif	//	__AVR_ATmega328P__

#if			Hssw_RX_BUF_MAX > 255
#error		"Hssw_RX_BUF_MAX must be less than 256"
#endif	//	Hssw_RX_BUF_MAX > 255


// instance
HsswSerial	SerialHssw;


void	HsswSerial::begin (
long	baud,
int		opt,
int		pin_rts,
int		pin_cts)
{
	// begin HsswSerial
	int		idx;
	uint8_t		clks;

	// baud_rate
	baud_rate=baud;
	// baud_rate -> prescl
	prescl=11UL*F_CPU/65536UL/baud_rate;
	clks=0;
	if (prescl<1)
	{
		clks=1;
		prescl=1;
	}
	else if (prescl<8)
	{
		clks=2;
		prescl=8;
	}
	else if (prescl<64)
	{
		clks=3;
		prescl=64;
	}
	else
	{
		// ERROR: baud rate too low
		prescl=0;						// error status indicator
		errcod=HsswErrBaud;
		return;
	}
	// prescl -> cnt_frame
	cnt_frame=10UL*F_CPU/baud_rate/prescl;
	if (cnt_frame<4*10)
	{
		// ERROR: bad baud rate
		prescl=0;						// error status indicator
		errcod=HsswErrBaud;
		return;
	}

	// rx sampling time in TC1 counter
	for (idx=0; idx<10; idx++)
	{
		if (idx==9)
		{
			// early capture for stop bit
			rx_cnt[idx]=uint32_t (cnt_frame)*(idx*8 + 1)/80;
		}
		else
			rx_cnt[idx]=(uint32_t (cnt_frame)*(idx*2 + 1) + 10)/20;
	}
	// tx output time in TC1 counter
	for (idx=0; idx<10; idx++)
	{
		tx_cnt[idx]=(uint32_t (cnt_frame)*(idx + 1) + 5)/10;
	}

	// option
	if ((opt&Hssw_SERIAL_MSK)!=SERIAL_8N1)
	{
		// ERROR: unsupported frame format
		prescl=0;						// error status indicator
		errcod=HsswErrBadParam;
		return;
	}
	pinMode (Hssw_pin_RXD, INPUT);
	digitalWrite (Hssw_pin_TXD, HIGH);
	pinMode (Hssw_pin_TXD, OUTPUT);
	// flow control
	lgc_neg=(opt&Hssw_LGC_NEGA)? true: false;
	rts_pin=(pin_rts<=0)? Hssw_pin_RTS: pin_rts;
	cts_pin=(pin_cts<=0)? Hssw_pin_CTS: pin_cts;
	rts_enb=false;
	if (opt&Hssw_RTS_ENBL)
	{
		rts_enb=true;
		digitalWrite (rts_pin, (lgc_neg)? LOW: HIGH);
		pinMode (rts_pin, OUTPUT);
	}
	cts_enb=false;
	if (opt&Hssw_CTS_ENBL)
	{
		cts_enb=true;
		pinMode (cts_pin, INPUT);
	}
	if ((rts_enb && cts_enb && rts_pin==cts_pin) ||
		(rts_enb && rts_pin==Hssw_pin_RXD) ||
		(rts_enb && rts_pin==Hssw_pin_TXD) ||
		(cts_enb && cts_pin==Hssw_pin_RXD) ||
		(cts_enb && cts_pin==Hssw_pin_TXD))
	{
		// ERROR: bad pin assignment
		prescl=0;						// error status indicator
		errcod=HsswErrBadParam;
		return;
	}

	// timer (TC1) setting
	(*Hssw_tccr1a)=0x00;				// normal port, normal mode
	(*Hssw_tccr1b)=0x00 | clks;			// no NC, falling, normal mode, clks
	(*Hssw_tifr1)=0x20;					// ICF1 clear
	(*Hssw_timsk1)=0x20;				// ICIE1 set
}

int		HsswSerial::available (void)
{
	// is receive data available
	// return: count of available data
	int		siz;

	siz=Hssw_RX_BUF_MAX + rx_buf_wr - rx_buf_rd;
	if (siz>=Hssw_RX_BUF_MAX)
		siz -= Hssw_RX_BUF_MAX;

	// RTS control
	SerialHssw.Hssw_auto_rts ();

	return (siz);
}

int		HsswSerial::peek (void)
{
	// peek rx_buf[]
	// return: data of read pointer / -1 (no data)
	int		dat;

	if (rx_buf_rd==rx_buf_wr)
	{
		// buffer empty
		dat=-1;
	}
	else
	{
		dat=rx_buf[rx_buf_rd];
	}

	// RTS control
	SerialHssw.Hssw_auto_rts ();

	return (dat);
}

int		HsswSerial::read (void)
{
	// read rx_buf[] and update read pointer
	// return: data of read pointer / -1 (no data)
	int		dat;

	if (rx_buf_rd==rx_buf_wr)
	{
		// buffer empty
		dat=-1;
	}
	else
	{
		dat=rx_buf[rx_buf_rd];

		// update read pointer
		rx_buf_rd++;
		if (rx_buf_rd>=Hssw_RX_BUF_MAX)
			rx_buf_rd -= Hssw_RX_BUF_MAX;
	}

	// RTS control
	SerialHssw.Hssw_auto_rts ();

	return (dat);
}

void	HsswSerial::flush (void)
{
	// flush send data buffer, nothing to do
}

int		HsswSerial::availableForWrite (void)
{
	// can send data
	// returns CTS status: 1=assert / 0=negate
	//		always 1 if CTS disabled
	// return: 1 / 0
	int		rtncod;
	uint8_t		pbd;

	rtncod=1;
	if (cts_enb)
	{
		pbd=digitalRead (cts_pin);
		if (lgc_neg)
			pbd=(pbd==LOW)? HIGH: LOW;
		rtncod=(pbd==LOW)? 0: 1;
	}

	return (rtncod);
}

void	HsswSerial::Hssw_auto_rts (
int		thsh)
{
	// control RTS signal by comparing counts of received data and thsh
	// thsh: RTS assert/negate threshold
	int		siz;

	if (rts_enb)
	{
		siz=Hssw_RX_BUF_MAX + rx_buf_wr - rx_buf_rd;
		if (siz>=Hssw_RX_BUF_MAX)
			siz -= Hssw_RX_BUF_MAX;
		if (siz<thsh)
			Hssw_assert_rts ();
		else
			Hssw_negate_rts ();
	}
}

size_t		HsswSerial::write (
uint8_t		chr)
{
	// send data
	// return: 1 (count of data sent) / 0 (not initialized)
	uint8_t		idx;
	uint8_t		pbd;
	uint8_t		pbd_c;
	uint8_t		pbd_s;
	uint8_t		sreg_bk;
	uint16_t	frm;
	volatile	uint8_t		*const	sreg=(uint8_t *)0x005f;

	// return if not initialized
	if (prescl==0)
		return (0);

	// check & polling CTS
	if (cts_enb)
	{
		do
		{
			pbd=digitalRead (cts_pin);
			if (lgc_neg)
				pbd=(pbd==LOW)? HIGH: LOW;
		} while (pbd==LOW);
	}

	// negate RTS
	Hssw_negate_rts ();

	// preparation
	pbd_c=(*Hssw_reg_TXD);
	pbd_c &= (~Hssw_bit_TXD);
	pbd_s=pbd_c | Hssw_bit_TXD;
	frm=(0x01<<9) | (chr<<1) | 0x00;

	// interrupt disable
	sreg_bk=(*sreg);
	noInterrupts ();

	// send frame (10 bits)
	(*Hssw_tcnt1)=0x0000;
	for (idx=0; idx<10; idx++)
	{
		if (frm&0x01)
			(*Hssw_reg_TXD)=pbd_s;
		else
			(*Hssw_reg_TXD)=pbd_c;
		frm >>= 1;
		// TC1 counter polling
		while ((*Hssw_tcnt1)<tx_cnt[idx])
			;
	}

	// check ICF1 flag
	if ((*Hssw_tifr1)&0x20)
	{
		// ERROR: receive (and lost) data during sending
		errcod=HsswErrRecv;
		(*Hssw_tifr1)=0x20;				// ICF1 clear
	}

	// interrupt recovery
	if (sreg_bk&0x80)
		(*sreg) |= 0x80;
	else
		(*sreg) &= (~0x80);

	// RTS control
	Hssw_auto_rts ();

	return (1);
}


ISR (TIMER1_CAPT_vect)
{
	// ISR on TIMER1 CAPT as RXD
	char	idx;
	uint8_t		pbd;
	uint16_t	frm;
	uint16_t	*rx_cnt_ptr;

	// TC1 counter set
	(*Hssw_tcnt1) -= (*Hssw_icr1);

	// receive
	frm=0;
	for (idx=0, rx_cnt_ptr=SerialHssw.rx_cnt; idx<10; idx++, rx_cnt_ptr++)
	{
		// TC1 counter polling
		while ((*Hssw_tcnt1)<(*rx_cnt_ptr))
			;

		// sampling
		frm >>= 1;
		pbd=(*Hssw_reg_RXD);
		if (pbd&Hssw_bit_RXD)
			frm |= 0x200;

		// distribution
		if (idx==4)
		{
			// RTS control
			SerialHssw.Hssw_auto_rts ();
		}
		if (idx==8)
		{
			SerialHssw.rx_buf[SerialHssw.rx_buf_wr]=(frm>>2)&0xff;
			SerialHssw.rx_buf_wr++;
			if (SerialHssw.rx_buf_wr>=Hssw_RX_BUF_MAX)
				SerialHssw.rx_buf_wr -= Hssw_RX_BUF_MAX;
			if (SerialHssw.rx_buf_wr==SerialHssw.rx_buf_rd)
			{
				// ERROR: rx buffer overflow
				SerialHssw.errcod=HsswErrOvfl;
			}
		}
	}

	// check frame format
	if (frm&0x01 && SerialHssw.errcod==HsswErrNo)
	{
		// ERROR: bad frame (start bit is not 0)
		SerialHssw.errcod=HsswErrFrame0;
	}
	if (!(frm&0x0200))
	{
		// ERROR: bad frame (stop bit is not 1)
		SerialHssw.errcod=HsswErrFrame1;
	}

	// clear ICF1
	(*Hssw_tifr1)=0x20;					// ICF1 clear
}
