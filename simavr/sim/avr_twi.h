/*
	avr_twi.h

	Copyright 2008, 2009 Michel Pollet <buserror@gmail.com>
        Copyright 2020  Luis Claudio Gambôa Lopes <lcgamboa@yahoo.com>
  
 	This file is part of simavr.

	simavr is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	simavr is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with simavr.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __AVR_TWI_H__
#define __AVR_TWI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "sim_avr.h"
#include "avr_bitbang.h"

enum {
	TWI_IRQ_INPUT = 0,
	TWI_IRQ_OUTPUT,
	TWI_IRQ_STATUS,
	TWI_IRQ_COUNT
};

enum {
	TWI_COND_START = (1 << 0),
	TWI_COND_STOP = (1 << 1),
	TWI_COND_ADDR = (1 << 2),
	TWI_COND_ACK = (1 << 3),
	TWI_COND_WRITE = (1 << 4),
	TWI_COND_READ = (1 << 5),
	// internal state, do not use in irq messages
	TWI_COND_SLAVE	= (1 << 6),
	TWI_COND_ACK_ADDR = (1 << 7),
};

typedef struct avr_twi_msg_t {
	uint32_t unused : 8,
		msg : 8,
		addr : 8,
		data : 8;
} avr_twi_msg_t;

typedef struct avr_twi_msg_irq_t {
	union {
		uint32_t v;
		avr_twi_msg_t twi;
	} u;
} avr_twi_msg_irq_t;

// add port number to get the real IRQ
#define AVR_IOCTL_TWI_GETIRQ(_name) AVR_IOCTL_DEF('t','w','i',(_name))

typedef struct avr_twi_t {
	avr_io_t	io;
	char name;

	avr_regbit_t	disabled;	// bit in the PRR

	avr_io_addr_t	r_twbr;			// bit rate register
	avr_io_addr_t	r_twcr;			// control register
	avr_io_addr_t	r_twsr;			// status register
	avr_io_addr_t	r_twar;			// address register (slave)
	avr_io_addr_t	r_twamr;		// address mask register
	avr_io_addr_t	r_twdr;			// data register
        
        avr_io_addr_t	r_ddrx;			// pin direction register
	
	avr_regbit_t twen;		// twi enable bit
	avr_regbit_t twea;		// enable acknowledge bit
	avr_regbit_t twsta;		// start condition
	avr_regbit_t twsto;		// stop condition
	avr_regbit_t twwc;		// write collision
	
	avr_regbit_t twsr;		// status registers, (5 bits)
	avr_regbit_t twps;		// prescaler bits (2 bits)
	
	avr_int_vector_t twi;	// twi interrupt

	uint8_t state;
	uint8_t peer_addr;
	uint8_t next_twstate;

	avr_iopin_t	p_sda;		// data in/out pin
	avr_iopin_t	p_scl;		// clk  in/out pin
	avr_bitbang_t	bit_bang;
        avr_bitbang_t	bit_bang_ack;

	uint8_t		input_data_register;
	//uint8_t		output_data_register;
} avr_twi_t;

void
avr_twi_init(
		avr_t * avr,
		avr_twi_t * port);

/*
 * Create a message value for twi including the 'msg' bitfield,
 * 'addr' and data. This value is what is sent as the IRQ value
 */
uint32_t
avr_twi_irq_msg(
		uint8_t msg,
		uint8_t addr,
		uint8_t data);



#define AVR_TWI_DECLARE(_prr, _prtwi, _pin_port, _p_scl, _p_sda) \
	.twi = { \
		.disabled = AVR_IO_REGBIT(_prr, _prtwi), \
	\
                .r_twcr = TWCR, \
		.r_twsr = TWSR, \
		.r_twbr = TWBR, \
		.r_twdr = TWDR, \
		.r_twar = TWAR, \
		.r_twamr = TWAMR, \
                .r_ddrx = DDRB, \
	\
		.twen = AVR_IO_REGBIT(TWCR, TWEN), \
		.twea = AVR_IO_REGBIT(TWCR, TWEA), \
		.twsta = AVR_IO_REGBIT(TWCR, TWSTA), \
		.twsto = AVR_IO_REGBIT(TWCR, TWSTO), \
		.twwc = AVR_IO_REGBIT(TWCR, TWWC), \
	\
		.bit_bang.p_clk = AVR_IOPIN(_pin_port, _p_scl), \
		.bit_bang.p_in = AVR_IOPIN(_pin_port, _p_sda), \
		.bit_bang.p_out = AVR_IOPIN(_pin_port, _p_sda), \
		.p_scl = AVR_IOPIN(_pin_port, _p_scl), \
		.p_sda = AVR_IOPIN(_pin_port, _p_sda), \
		.twsr = AVR_IO_REGBITS(TWSR, TWS3, 0x1f),\
		.twps = AVR_IO_REGBITS(TWSR, TWPS0, 0x3),\
	\
		.twi = {\
			.enable = AVR_IO_REGBIT(TWCR, TWIE),\
			.raised = AVR_IO_REGBIT(TWCR, TWINT),\
			.raise_sticky = 0,\
			.vector = TWI_vect,\
		},\
	}


#ifdef __cplusplus
};
#endif

#endif /*__AVR_TWI_H__*/
