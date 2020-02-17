/*
    avr_twi.c

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

#include <stdio.h>
#include "avr_twi.h"

#include <stdlib.h> //FIXME remove
/*
 * This block respectfully nicked straight out from the Atmel sample
 * code for AVR315. Typos and all.
 * There is no copyright notice on the original file.
 */
/****************************************************************************
  TWI State codes
 ****************************************************************************/
// General TWI Master status codes
#define TWI_START                  0x08  // START has been transmitted
#define TWI_REP_START              0x10  // Repeated START has been transmitted
#define TWI_ARB_LOST               0x38  // Arbitration lost

// TWI Master Transmitter status codes
#define TWI_MTX_ADR_ACK            0x18  // SLA+W has been transmitted and ACK received
#define TWI_MTX_ADR_NACK           0x20  // SLA+W has been transmitted and NACK received
#define TWI_MTX_DATA_ACK           0x28  // Data byte has been transmitted and ACK received
#define TWI_MTX_DATA_NACK          0x30  // Data byte has been transmitted and NACK received

// TWI Master Receiver status codes
#define TWI_MRX_ADR_ACK            0x40  // SLA+R has been transmitted and ACK received
#define TWI_MRX_ADR_NACK           0x48  // SLA+R has been transmitted and NACK received
#define TWI_MRX_DATA_ACK           0x50  // Data byte has been received and ACK transmitted
#define TWI_MRX_DATA_NACK          0x58  // Data byte has been received and NACK transmitted

// TWI Slave Transmitter status codes
#define TWI_STX_ADR_ACK            0xA8  // Own SLA+R has been received; ACK has been returned
#define TWI_STX_ADR_ACK_M_ARB_LOST 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define TWI_STX_DATA_ACK           0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_NACK          0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define TWI_STX_DATA_ACK_LAST_BYTE 0xC8  // Last data byte in TWDR has been transmitted (TWEA = �0�); ACK has been received

// TWI Slave Receiver status codes
#define TWI_SRX_ADR_ACK            0x60  // Own SLA+W has been received ACK has been returned
#define TWI_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define TWI_SRX_GEN_ACK            0x70  // General call address has been received; ACK has been returned
#define TWI_SRX_GEN_ACK_M_ARB_LOST 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_ACK       0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_NACK      0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define TWI_SRX_GEN_DATA_ACK       0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define TWI_SRX_GEN_DATA_NACK      0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define TWI_SRX_STOP_RESTART       0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave

// TWI Miscellaneous status codes
#define TWI_NO_STATE               0xF8  // No relevant state information available; TWINT = �0�
#define TWI_BUS_ERROR              0x00  // Bus error due to an illegal START or STOP condition

#define AVR_TWI_DEBUG 1

/*
//FIXME
#undef AVR_TRACE
#define AVR_TRACE(avr, ... ) \
    printf( __VA_ARGS__)
*/


static inline void
_avr_twi_status_set(
                    avr_twi_t * p,
                    uint8_t v,
                    int interrupt)
{
 avr_regbit_setto_raw (p->io.avr, p->twsr, v);
#if AVR_TWI_DEBUG
 AVR_TRACE (p->io.avr, "%s %02x\n", __func__, v);
#endif
 avr_raise_irq (p->io.irq + TWI_IRQ_STATUS, v);
 if (interrupt)
  {
    avr_raise_interrupt (p->io.avr, &p->twi);
    if (p->p_sda.port)
    {
     avr_raise_irq (avr_io_getirq (p->bit_bang.avr, AVR_IOCTL_IOPORT_GETIRQ (p->p_sda.port), p->p_sda.pin), 0);
    }
  }
}


static __attribute__ ((unused)) inline uint8_t
_avr_twi_status_get(
                    avr_twi_t * p)
{
 return avr_regbit_get_raw (p->io.avr, p->twsr);
}

static avr_cycle_count_t
avr_stop_clk_timer(struct avr_t * avr, avr_cycle_count_t when, void * param)
{
 avr_twi_t * p = (avr_twi_t *) param;

 p->bit_bang.clk_generate = 1;
 p->bit_bang.buffer_size = 1;
 p->bit_bang.clk_phase = 1;
 p->bit_bang.clk_pol = 0;
 p->bit_bang.data_order = 0;
 p->bit_bang.p_clk.port = 0;
 p->bit_bang.p_out = p->p_sda;

 //p->bit_bang.p_clk = p->p_scl;
 //p->bit_bang.p_out = p->p_sda;

 avr_bitbang_stop (&(p->bit_bang));
 avr_bitbang_reset (avr, &(p->bit_bang));
 p->bit_bang.data = 1;
 avr_bitbang_start (&(p->bit_bang));

 if (p->p_sda.port)
  {
   //avr_raise_irq (avr_io_getirq (avr, AVR_IOCTL_IOPORT_GETIRQ (p->p_sda.port), p->p_sda.pin), 0);
   avr_raise_irq (avr_io_getirq (avr, AVR_IOCTL_IOPORT_GETIRQ (p->p_scl.port), p->p_scl.pin), 1);
  }

 return 0;
}

static avr_cycle_count_t
avr_start_clk_timer(struct avr_t * avr, avr_cycle_count_t when, void * param)
{
 avr_twi_t * p = (avr_twi_t *) param;
 
   p->bit_bang.clk_generate = 1;
   p->bit_bang.buffer_size = 1;
   p->bit_bang.clk_phase = 1;
   p->bit_bang.clk_pol = 0;
   p->bit_bang.data_order = 0;
   p->bit_bang.p_clk = p->p_scl;
   p->bit_bang.p_out = p->p_sda;


   //update pin direction
   avr_set_r (avr, p->r_ddrx, avr_core_watch_read (avr, p->r_ddrx) | 1 << p->p_scl.pin | 1 << p->p_sda.pin);
 
   //pull-up
   avr_set_r (avr, p->r_ddrx+1, avr_core_watch_read (avr, p->r_ddrx+1) & ~(1 << p->p_scl.pin | 1 << p->p_sda.pin));

   avr_bitbang_stop (&(p->bit_bang));
   avr_bitbang_reset (avr, &(p->bit_bang));
   p->bit_bang.data = 0;
   avr_bitbang_start (&(p->bit_bang));

   if (p->p_sda.port)
    {
     avr_raise_irq (avr_io_getirq (p->bit_bang.avr, AVR_IOCTL_IOPORT_GETIRQ (p->p_sda.port), p->p_sda.pin), 1);
     avr_raise_irq (avr_io_getirq (p->bit_bang.avr, AVR_IOCTL_IOPORT_GETIRQ (p->p_scl.port), p->p_scl.pin), 1);
    }
   
   return 0;
}

static uint32_t
avr_twi_transfer_finished(uint32_t data, void *param)
{
 avr_twi_t * p = (avr_twi_t *) param;
 /*    
     printf("=======> state= 0x%02X\nSTART=%i  STOP=%i ADDR=%i  ACK=%i WRITE=%i READ=%i SLAVE=%i ACK_ADDR=%i\n",p->state,
         (p->state & 0x01)  > 0,
         (p->state & 0x02)  > 0,
         (p->state & 0x04)  > 0,
         (p->state & 0x08)  > 0,
         (p->state & 0x10)  > 0,
         (p->state & 0x20)  > 0,
         (p->state & 0x40)  > 0,
         (p->state & 0x80)  > 0   
         );
  */
 if (p->bit_bang.clk_generate)
  {
   avr_bitbang_stop (&(p->bit_bang));

   if (p->bit_bang.buffer_size == 1)
    {
     if (p->state & TWI_COND_STOP) // avr_regbit_get (p->bit_bang.avr, p->twsto))
      {
       // generate a stop condition
       /* clear stop condition regardless of status */
       avr_regbit_clear (p->bit_bang.avr, p->twsto);
       _avr_twi_status_set (p, TWI_NO_STATE, 0);
       p->state = 0;

      }
     else
      {
       // generate a start condition
       if (p->state & TWI_COND_START)
        _avr_twi_status_set(p, TWI_REP_START,1);
       else
        _avr_twi_status_set(p, TWI_START,1);
       p->peer_addr = 0;
       p->state = TWI_COND_START;
      }
    }
   else if (p->bit_bang.buffer_size == 8) //read ack on transmission end
    {
     //printf("data dir=%i value=%02X\n" ,p->peer_addr & 1,p->bit_bang.data);
     
     p->bit_bang_ack.clk_generate = 1;
     p->bit_bang_ack.buffer_size = 1;
     p->bit_bang_ack.clk_phase = 0;
     p->bit_bang_ack.clk_pol = 0;
     p->bit_bang_ack.data_order = 0;
     p->bit_bang_ack.p_clk = p->p_scl;

     if ((p->peer_addr & 1)&&(!(p->state & TWI_COND_ACK_ADDR)))//read operation?
      {
        p->bit_bang_ack.p_out = p->p_sda;
       //update pin direction
       avr_set_r (p->bit_bang_ack.avr, p->r_ddrx, avr_core_watch_read (p->bit_bang_ack.avr, p->r_ddrx) | (1 << p->p_sda.pin));
       p->input_data_register=data;
      }
     else
      {
       p->bit_bang_ack.p_out.port = 0;
       //update pin direction
       avr_set_r (p->bit_bang_ack.avr, p->r_ddrx, avr_core_watch_read (p->bit_bang_ack.avr, p->r_ddrx)& ~(1 << p->p_sda.pin));
      }

     avr_bitbang_stop (&(p->bit_bang_ack));
     avr_bitbang_reset (p->bit_bang_ack.avr, &(p->bit_bang_ack));
     p->bit_bang_ack.data = 0;//ACK
     avr_bitbang_start (&(p->bit_bang_ack));
     //avr_raise_irq (avr_io_getirq (p->bit_bang.avr, AVR_IOCTL_IOPORT_GETIRQ (p->p_scl.port), p->p_scl.pin), 0);
    }
  }
 return data;
}

static uint32_t
avr_twi_ack_finished(uint32_t data, void *param)
{
 avr_twi_t * p = (avr_twi_t *) param;
 /*    
     printf("=======> state= 0x%02X\nSTART=%i  STOP=%i ADDR=%i  ACK=%i WRITE=%i READ=%i SLAVE=%i ACK_ADDR=%i\n",p->state,
         (p->state & 0x01)  > 0,
         (p->state & 0x02)  > 0,
         (p->state & 0x04)  > 0,
         (p->state & 0x08)  > 0,
         (p->state & 0x10)  > 0,
         (p->state & 0x20)  > 0,
         (p->state & 0x40)  > 0,
         (p->state & 0x80)  > 0   
         );
  */
 if (p->bit_bang_ack.clk_generate)
  {
   avr_bitbang_stop (&(p->bit_bang_ack));

   //printf ("######## READ bitbang %02X\n", data);

   if (p->state & TWI_COND_ACK_ADDR)
    {
     p->state &= ~TWI_COND_ACK_ADDR;
    
     if (data & 0x01)
        p->state &= ~TWI_COND_ACK;
     else
        p->state |= TWI_COND_ACK;

     // we send an IRQ and we /expect/ a slave to reply
     // via an IRQ tp set the COND_ACK bit
     // otherwise it's assumed it's been nacked...
     avr_raise_irq (p->io.irq + TWI_IRQ_OUTPUT,
                    avr_twi_irq_msg (TWI_COND_START, p->peer_addr, 0));

     if (p->peer_addr & 1)
      { // read ?
       p->state |= TWI_COND_READ; // always allow read to start with
       _avr_twi_status_set (p,
                            p->state & TWI_COND_ACK ?
                            TWI_MRX_ADR_ACK : TWI_MRX_ADR_NACK,1);
      }
     else
      {
       /*
       if (p->state & TWI_COND_WRITE)
        {
         _avr_twi_status_set (p,
                              p->state & TWI_COND_ACK ?
                              TWI_MTX_DATA_ACK : TWI_MTX_DATA_NACK,1);
        }  
       else
        {*/
         _avr_twi_status_set (p,
                              p->state & TWI_COND_ACK ?
                              TWI_MTX_ADR_ACK : TWI_MTX_ADR_NACK,1);
        //}
      }
    }
   else
    {

     int do_read = p->peer_addr & 1;
     int do_ack = avr_regbit_get (p->bit_bang.avr, p->twea) != 0;
     uint8_t msgv = do_read ? TWI_COND_READ : TWI_COND_WRITE;
     if (do_ack)
      {
       msgv |= TWI_COND_ACK;
      }
     
     if (data & 0x01)
        p->state &= ~TWI_COND_ACK;
     else
        p->state |= TWI_COND_ACK;
     

     // we send an IRQ and we /expect/ a slave to reply
     // via an IRQ to set the COND_ACK bit
     // otherwise it's assumed it's been nacked...
     avr_raise_irq (p->io.irq + TWI_IRQ_OUTPUT,
                    avr_twi_irq_msg (msgv, p->peer_addr, p->bit_bang.avr->data[p->r_twdr]));


     if (do_read)
      { // read ?
       _avr_twi_status_set (p,
                            msgv & TWI_COND_ACK ?
                            TWI_MRX_DATA_ACK : TWI_MRX_DATA_NACK,1);
       avr_core_watch_write (p->bit_bang.avr, p->r_twdr , p->input_data_register);
      }
     else
      {
            
       _avr_twi_status_set (p,
                            p->state & TWI_COND_ACK ?
                            TWI_MTX_DATA_ACK : TWI_MTX_DATA_NACK,1);
      }
    }

   p->state &= ~TWI_COND_WRITE;

   //update pin direction
   avr_set_r (p->bit_bang_ack.avr, p->r_ddrx, avr_core_watch_read (p->bit_bang_ack.avr, p->r_ddrx) | 1 << p->p_scl.pin | 1 << p->p_sda.pin);

   if (p->p_sda.port)
    {
     avr_raise_irq (avr_io_getirq (p->bit_bang_ack.avr, AVR_IOCTL_IOPORT_GETIRQ (p->p_sda.port), p->p_sda.pin), 0);
     avr_raise_irq (avr_io_getirq (p->bit_bang_ack.avr, AVR_IOCTL_IOPORT_GETIRQ (p->p_scl.port), p->p_scl.pin), 0);
    }


  }

 return 0;
}
/*
static void avr_twi_bitbang_switch_mode(avr_twi_t * p, uint8_t master)
{
	
    if (master) {
        p->bit_bang.p_in = p->p_sda;
        p->bit_bang.p_out = p->p_sda;
        avr_irq_t *irq;
        uint32_t irq_val;
        irq = avr_io_getirq( p->bit_bang.avr,
                                        AVR_IOCTL_IOPORT_GETIRQ(p->p_sda.port),
                                        IOPORT_IRQ_DIRECTION_ALL );
        irq_val = irq->value & (1 << p->p_sda.pin);
        avr_raise_irq(irq, irq_val);
        p->bit_bang.clk_generate = 1;
    } else {
        p->bit_bang.p_in = p->p_sda;
        p->bit_bang.p_out = p->p_sda;
        avr_irq_t *irq;
        uint32_t irq_val;
        irq = avr_io_getirq( p->bit_bang.avr,
                                        AVR_IOCTL_IOPORT_GETIRQ(p->bit_bang.p_clk.port),
                                        IOPORT_IRQ_DIRECTION_ALL );
        irq_val = irq->value & (1 << p->bit_bang.p_clk.pin);
        avr_raise_irq(irq, irq_val);
        irq = avr_io_getirq( p->bit_bang.avr,
                                        AVR_IOCTL_IOPORT_GETIRQ(p->p_sda.port),
                                        IOPORT_IRQ_DIRECTION_ALL );
        irq_val = irq->value & (1 << p->p_sda.pin);
        avr_raise_irq(irq, irq_val);
        //irq = avr_io_getirq( p->bit_bang.avr,
                                        //AVR_IOCTL_IOPORT_GETIRQ(p->p_ss.port),
                                        //IOPORT_IRQ_DIRECTION_ALL );
        //irq_val = irq->value & (1 << p->p_ss.pin);
        avr_raise_irq(irq, irq_val);
        p->bit_bang.clk_generate = 0;
    }
}
 */

/*
void adicionar(struct avr_t * avr, avr_io_addr_t addr, uint8_t v, void * param)
{
    avr_twi_t * p = (avr_twi_t *)param;
	
//#################### start?
        // The byte to be sent should NOT be written there,
        // the value written could never be read back.
        //avr_core_watch_write(avr, addr, v);
        p->output_data_register = v;
        //if (avr->gdb) {
        //	avr_gdb_handle_watchpoints(avr, addr, AVR_GDB_WATCH_WRITE);
        //}
        if (avr_regbit_get(avr, p->twen)) {
            avr_bitbang_stop(&(p->bit_bang));
            avr_bitbang_reset(avr, &(p->bit_bang));
            p->bit_bang.data = v;
            //FIXME if (avr_regbit_get(avr, p->mstr)) 
            if(1)
            {
                p->bit_bang.clk_generate = 1;
                avr_bitbang_start(&(p->bit_bang));
            } else {
                p->bit_bang.clk_generate = 0;
            }
        }

//#################### configuração

        avr_bitbang_stop(&(p->bit_bang));
        avr_bitbang_reset(avr, &(p->bit_bang));
        if (avr_regbit_get(avr, p->twen)) {
            p->bit_bang.clk_phase = 1; //FIXME move to init
            p->bit_bang.clk_pol = 1;
            p->bit_bang.data_order = 1;
            //FIXME if (avr_regbit_get(avr, p->mstr)) 
            if(1)
            {	

                int clock_divider_ix = (avr_regbit_get(avr, p->spr[0]) | avr_regbit_get(avr, p->spr[1]));
                int clock_divider = 1;
                switch (clock_divider_ix) {
                    case 0:
                        clock_divider = 4;
                        break;
                    case 1:
                        clock_divider = 16;
                        break;
                    case 2:
                        clock_divider = 64;
                        break;
                    case 3:
                        clock_divider = 128;
                        break;
                }
                if (avr_regbit_get(avr, p->spr[2]))
                    clock_divider /= 2;
                p->bit_bang.clk_cycles = clock_divider<<2; //FIXME: <<2 for picsimlab compatibility
				
                avr_twi_bitbang_switch_mode(p, 1);

            } else {
                avr_twi_bitbang_switch_mode(p, 0);
            }
        }
}
 */

/*
//static 
void avr_spi_ss_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
    avr_twi_t * p = (avr_twi_t *)param;

    //FIXME if (avr_regbit_get(p->bit_bang.avr, p->mstr))  // master mode
    if(1)
    {
        //avr_ioport_state_t iostate;
        uint8_t dir = 0;

        //avr_ioctl(p->bit_bang.avr, AVR_IOCTL_IOPORT_GETSTATE( p->p_ss.port ), &iostate);
        //dir = ( iostate.ddr >> p->p_ss.pin ) & 1;
        if (!dir) {
            if (!value) {
                // other master is active, reset to slave mode
                avr_bitbang_stop(&(p->bit_bang));
                avr_bitbang_reset(p->bit_bang.avr, &(p->bit_bang));
                //avr_regbit_setto(p->bit_bang.avr, p->mstr, 0);
                avr_twi_bitbang_switch_mode(p, 0);
                avr_twi_raise(p);
            }
        }
    } else { // slave mode
        if (value) {
            avr_bitbang_stop(&(p->bit_bang));
            avr_bitbang_reset(p->bit_bang.avr, &(p->bit_bang));
        }
    }

}
 */


static void
avr_twi_write(
              struct avr_t * avr,
              avr_io_addr_t addr,
              uint8_t v,
              void * param)
{
 avr_twi_t * p = (avr_twi_t *) param;

 uint8_t twen = avr_regbit_get (avr, p->twen);
 uint8_t twsta = avr_regbit_get (avr, p->twsta);
 uint8_t twsto = avr_regbit_get (avr, p->twsto);
 uint8_t twint = avr_regbit_get (avr, p->twi.raised);

 avr_core_watch_write (avr, addr, v);
#if AVR_TWI_DEBUG
 AVR_TRACE (avr, "%s %02x START:%d STOP:%d ACK:%d INT:%d TWSR:%02x (state %02x)\n",
            __func__, v,
            avr_regbit_get (avr, p->twsta),
            avr_regbit_get (avr, p->twsto),
            avr_regbit_get (avr, p->twea),
            avr_regbit_get (avr, p->twi.raised),
            avr_regbit_get_raw (p->io.avr, p->twsr), p->state);
#endif
 if (twen != avr_regbit_get (avr, p->twen))
  {
   twen = !twen;
   if (!twen)
    { // if we were running, now now are not
     avr_regbit_clear (avr, p->twea);
     avr_regbit_clear (avr, p->twsta);
     avr_regbit_clear (avr, p->twsto);
     avr_clear_interrupt (avr, &p->twi);
     avr_core_watch_write (avr, p->r_twdr, 0xff);
     _avr_twi_status_set (p, TWI_NO_STATE, 0);
     p->state = 0;
     p->peer_addr = 0;
    }
   AVR_TRACE (avr, "TWEN: %d\n", twen);
   if (avr->data[p->r_twar])
    {
     AVR_TRACE (avr, "TWEN Slave: %02x&%02x\n", avr->data[p->r_twar] >> 1, avr->data[p->r_twamr] >> 1);
     p->state |= TWI_COND_SLAVE;
    }
  }
 if (!twen)
  return;

 uint8_t cleared = avr_regbit_get (avr, p->twi.raised);

 /*int cleared = */
 avr_clear_interrupt_if (avr, &p->twi, twint);
 //	AVR_TRACE(avr, "cleared %d\n", cleared);

 if (!twsto && avr_regbit_get (avr, p->twsto))
  {
   // generate a stop condition
#if AVR_TWI_DEBUG
   AVR_TRACE (avr, "<<<<< I2C stop\n");
#endif
   if (p->state)
    { // doing stuff
     if (p->state & TWI_COND_START)
      {
       avr_raise_irq (p->io.irq + TWI_IRQ_OUTPUT,
                      avr_twi_irq_msg (TWI_COND_STOP, p->peer_addr, 1));
      }
    }

   if (p->p_sda.port)
    {
     avr_raise_irq (avr_io_getirq (avr, AVR_IOCTL_IOPORT_GETIRQ (p->p_sda.port), p->p_sda.pin), 0);
     avr_raise_irq (avr_io_getirq (avr, AVR_IOCTL_IOPORT_GETIRQ (p->p_scl.port), p->p_scl.pin), 0);
    }
   p->state |= TWI_COND_STOP;

   avr_cycle_timer_register (avr, p->bit_bang.clk_cycles / 2, avr_stop_clk_timer, p);



  }
 if (!twsta && avr_regbit_get (avr, p->twsta))
  {
#if AVR_TWI_DEBUG
   AVR_TRACE (avr, ">>>>> I2C %sstart\n", p->state & TWI_COND_START ? "RE" : "");
#endif
   
   if(p->state & TWI_COND_STOP)
    { //hack to wait previous stop condition finish
      avr_cycle_timer_register (avr, p->bit_bang.clk_cycles * 20, avr_start_clk_timer, p);
    }
   else
    {
      avr_cycle_timer_register (avr, 0 , avr_start_clk_timer, p);
    }

  }

 int data = cleared &&
     !avr_regbit_get (avr, p->twsta) &&
     !avr_regbit_get (avr, p->twsto);

 if (!data)
  return;

 int do_read = p->peer_addr & 1;
 int do_ack = avr_regbit_get (avr, p->twea) != 0;

 if (p->state & TWI_COND_SLAVE)
  {
   // writing or reading a byte
   if (p->state & TWI_COND_ADDR)
    {
#if AVR_TWI_DEBUG
     if (do_read)
      AVR_TRACE (avr, "I2C slave READ byte\n");
     else
      AVR_TRACE (avr, "I2C slave WRITE byte\n");
#endif
     if (do_read)
      {
       if (p->state & TWI_COND_WRITE)
        {
         avr_raise_irq (p->io.irq + TWI_IRQ_OUTPUT,
                        avr_twi_irq_msg (TWI_COND_READ | TWI_COND_ACK, p->peer_addr, avr->data[p->r_twdr]));
        }
#if AVR_TWI_DEBUG
       else
        AVR_TRACE (avr, "I2C latch is not ready, do nothing\n");
#endif
      }
     else
      {
       avr_raise_irq (p->io.irq + TWI_IRQ_OUTPUT,
                      avr_twi_irq_msg (TWI_COND_ACK, p->peer_addr, 0));
      }
     avr_raise_irq (p->io.irq + TWI_IRQ_OUTPUT,
                    avr_twi_irq_msg (TWI_COND_ADDR + (do_ack ? TWI_COND_ACK : 0), p->peer_addr, avr->data[p->r_twdr]));
    }
   else
    { // address, acknowledge it
     p->state |= TWI_COND_ADDR;
     avr_raise_irq (p->io.irq + TWI_IRQ_OUTPUT,
                    avr_twi_irq_msg (
                                     TWI_COND_ADDR |
                                     (do_ack ? TWI_COND_ACK : 0) |
                                     (p->state & TWI_COND_WRITE ? TWI_COND_READ : 0),
                                     p->peer_addr, avr->data[p->r_twdr]));
    }
  }
 else
  {

   // writing or reading a byte
   if (p->state & TWI_COND_ADDR)
    {
#if AVR_TWI_DEBUG
     if (do_read)
      AVR_TRACE (avr, "I2C READ byte from %02x\n", p->peer_addr);
     else
      AVR_TRACE (avr, "I2C WRITE byte %02x to %02x\n", avr->data[p->r_twdr], p->peer_addr);
#endif
     // a normal data byte
     uint8_t msgv = do_read ? TWI_COND_READ : TWI_COND_WRITE;

     if (do_ack)
      msgv |= TWI_COND_ACK;

     p->state &= ~TWI_COND_ACK; // clear ACK bit

     AVR_TRACE (avr, "state %02x want %02x\n", p->state, msgv);

     // if the latch is ready... as set by writing/reading the TWDR
     if (p->state & msgv)
      {
       unsigned int data;
       
       p->bit_bang.clk_generate = 1;
       p->bit_bang.buffer_size = 8;
       p->bit_bang.clk_phase = 0;
       p->bit_bang.clk_pol = 0;
       p->bit_bang.data_order = 0;
       p->bit_bang.p_clk = p->p_scl;

       if (do_read)
        {
         p->bit_bang.p_out.port = 0;
         data=0;
         //update pin direction
         avr_set_r (p->bit_bang.avr, p->r_ddrx, avr_core_watch_read (p->bit_bang.avr, p->r_ddrx)& ~(1 << p->p_sda.pin));
        }
       else
        {
         p->bit_bang.p_out = p->p_sda;
         data=avr->data[p->r_twdr];
        }

       avr_bitbang_stop (&(p->bit_bang));
       avr_bitbang_reset (avr, &(p->bit_bang));
       p->bit_bang.data = data;
       avr_bitbang_start (&(p->bit_bang));

      }
#if AVR_TWI_DEBUG
     else
      AVR_TRACE (avr, "I2C latch is not ready, do nothing\n");
#endif
    }
   else if (p->state)
    {
#if AVR_TWI_DEBUG
     AVR_TRACE (avr, "I2C Master address %02x\n", avr->data[p->r_twdr]);
#endif
     // send the address
     p->state |= TWI_COND_ADDR;
     p->peer_addr = avr->data[p->r_twdr];
     p->state &= ~TWI_COND_ACK; // clear ACK bit


     p->bit_bang.clk_generate = 1;
     p->bit_bang.buffer_size = 8;
     p->bit_bang.clk_phase = 0;
     p->bit_bang.clk_pol = 0;
     p->bit_bang.data_order = 0;
     p->bit_bang.p_clk = p->p_scl;
     p->bit_bang.p_out = p->p_sda;

     avr_bitbang_stop (&(p->bit_bang));
     avr_bitbang_reset (avr, &(p->bit_bang));
     p->bit_bang.data = p->peer_addr;
     avr_bitbang_start (&(p->bit_bang));

     p->state |= TWI_COND_ACK_ADDR;

    }

  }
}

/*
 * Write data to the latch, tell the system we have something
 * to send next
 */
static void
avr_twi_write_data(
                   struct avr_t * avr,
                   avr_io_addr_t addr,
                   uint8_t v,
                   void * param)
{
 avr_twi_t * p = (avr_twi_t *) param;

 avr_core_watch_write (avr, addr, v);
 // tell system we have something in the write latch
 p->state |= TWI_COND_WRITE;
}

/*
 * Read data from the latch, tell the system can receive a new byte
 */
static uint8_t
avr_twi_read_data(
                  struct avr_t * avr,
                  avr_io_addr_t addr,
                  void * param)
{
 avr_twi_t * p = (avr_twi_t *) param;

 // tell system we can receive another byte
 p->state |= TWI_COND_READ;
 return avr->data[p->r_twdr];
}

/*
 * prevent code from rewriting out status bits, since we actually use them!
 */
static void
avr_twi_write_status(
                     struct avr_t * avr,
                     avr_io_addr_t addr,
                     uint8_t v,
                     void * param)
{
 avr_twi_t * p = (avr_twi_t *) param; 
 uint8_t sr = avr_regbit_get (avr, p->twsr);
 uint8_t c = avr_regbit_get (avr, p->twps);

 avr_core_watch_write (avr, addr, v); 
 avr_regbit_setto (avr, p->twsr, sr); // force restore


 if (c != avr_regbit_get (avr, p->twps))
  {
   // prescaler bits changed...
  }
}

static void
avr_twi_irq_input(
                  struct avr_irq_t * irq,
                  uint32_t value,
                  void * param)
{
 avr_twi_t * p = (avr_twi_t *) param;
 avr_t * avr = p->io.avr;

 // check to see if we are enabled
 if (!avr_regbit_get (avr, p->twen))
  return;
 avr_twi_msg_irq_t msg;
 msg.u.v = value;

 AVR_TRACE (avr, "%s %08x\n", __func__, value);

 // receiving an attempt at waking a slave
 if (msg.u.twi.msg & TWI_COND_START)
  {
   p->state = 0;
   p->peer_addr = 0;
   if (msg.u.twi.msg & TWI_COND_ADDR)
    {
     uint8_t mask = ~avr->data[p->r_twamr] >> 1;
     AVR_TRACE (avr, "I2C slave start %2x (want %02x&%02x)\n",
                msg.u.twi.addr, avr->data[p->r_twar] >> 1, mask);
     p->peer_addr = msg.u.twi.addr & mask;
     if (p->peer_addr == ((avr->data[p->r_twar] >> 1) & mask))
      {
       // address match, we're talking
       p->state = TWI_COND_SLAVE;
       // INVERSE logic here
       if (!(msg.u.twi.msg & TWI_COND_WRITE))
        p->peer_addr |= 1;
       _avr_twi_status_set (p, //TODO
                            msg.u.twi.msg & TWI_COND_WRITE ?
                            TWI_SRX_ADR_ACK : TWI_STX_ADR_ACK,1);
      }
    }
   else
    {
     // "general call" address
     AVR_TRACE (avr, "I2C slave start without address?\n");
     if (avr->data[p->r_twar] & 1)
      {
       // TODO
      }
    }
  }

 if (msg.u.twi.msg & TWI_COND_STOP)
  {
   _avr_twi_status_set (p, //TODO
                        msg.u.twi.msg & TWI_COND_WRITE ?
                        TWI_SRX_ADR_ACK : TWI_STX_ADR_ACK,1);
  }

 // receiving an acknowledge bit
 if (msg.u.twi.msg & TWI_COND_ACK)
  {
#if AVR_TWI_DEBUG
   AVR_TRACE (avr, "I2C received ACK:%d\n", msg.u.twi.data & 1);
#endif
   if (msg.u.twi.data & 1)
    p->state |= TWI_COND_ACK;
   else
    p->state &= ~TWI_COND_ACK;
  }
 if (p->state & TWI_COND_SLAVE)
  {
   if (msg.u.twi.msg & TWI_COND_WRITE)
    {
     avr->data[p->r_twdr] = msg.u.twi.data;
     _avr_twi_status_set (p, //TODO 
                          TWI_SRX_ADR_DATA_ACK,1);
    }
  }
 else
  {
   // receive a data byte from a slave
   if (msg.u.twi.msg & TWI_COND_READ)
    {
#if AVR_TWI_DEBUG
     AVR_TRACE (avr, "I2C received %02x\n", msg.u.twi.data);
#endif
     avr->data[p->r_twdr] = msg.u.twi.data;
    }
  }
}

void
avr_twi_reset(struct avr_io_t *io)
{
 avr_twi_t * p = (avr_twi_t *) io;
 avr_bitbang_reset (p->bit_bang.avr, &(p->bit_bang));
 avr_bitbang_reset (p->bit_bang_ack.avr, &(p->bit_bang_ack));

 avr_irq_register_notify (p->io.irq + TWI_IRQ_INPUT, avr_twi_irq_input, p);
 p->state = p->peer_addr = 0;
 avr_regbit_setto_raw (p->io.avr, p->twsr, TWI_NO_STATE);

}

static const char * irq_names[TWI_IRQ_COUNT] = {
 [TWI_IRQ_INPUT] = "8<input",
 [TWI_IRQ_OUTPUT] = "32>output",
 [TWI_IRQ_STATUS] = "8>status",
};

static avr_io_t _io = {
 .kind = "twi",
 .reset = avr_twi_reset,
 .irq_names = irq_names,
};

void
avr_twi_init(avr_t * avr, avr_twi_t * p)
{
 p->io = _io;

 p->bit_bang.avr = avr;
 p->bit_bang.callback_transfer_finished = avr_twi_transfer_finished;
 p->bit_bang.callback_param = p;
 p->bit_bang.buffer_size = 8;
 p->bit_bang.clk_cycles = 8000;

 p->bit_bang_ack.avr = avr;
 p->bit_bang_ack.callback_transfer_finished = avr_twi_ack_finished;
 p->bit_bang_ack.callback_param = p;
 p->bit_bang_ack.buffer_size = 1;
 p->bit_bang_ack.clk_cycles = 8000;

 p->bit_bang_ack.p_clk = p->bit_bang.p_clk;
 p->bit_bang_ack.p_in = p->bit_bang.p_in;

 p->r_ddrx += (p->p_sda.port - 'B')*3;

 avr_raise_irq (avr_io_getirq (avr, AVR_IOCTL_IOPORT_GETIRQ (p->p_sda.port), IOPORT_IRQ_DIRECTION_ALL), 1);

 avr_raise_irq (avr_io_getirq (avr, AVR_IOCTL_IOPORT_GETIRQ (p->p_sda.port), p->p_sda.pin), 1);
 avr_raise_irq (avr_io_getirq (avr, AVR_IOCTL_IOPORT_GETIRQ (p->p_scl.port), p->p_scl.pin), 1);


 avr_register_io (avr, &p->io);
 avr_register_vector (avr, &p->twi);

 //printf("%s TWI%c init\n", __FUNCTION__, p->name);

 // allocate this module's IRQ
 avr_io_setirqs (&p->io, AVR_IOCTL_TWI_GETIRQ (p->name), TWI_IRQ_COUNT, NULL);

 avr_register_io_write (avr, p->twen.reg, avr_twi_write, p);
 avr_register_io_write (avr, p->r_twdr, avr_twi_write_data, p);
 avr_register_io_read (avr, p->r_twdr, avr_twi_read_data, p);
 avr_register_io_write (avr, p->twsr.reg, avr_twi_write_status, p);

}

uint32_t
avr_twi_irq_msg(
                uint8_t msg,
                uint8_t addr,
                uint8_t data)
{
 avr_twi_msg_irq_t _msg = {
  .u.twi.msg = msg,
  .u.twi.addr = addr,
  .u.twi.data = data,
 };
 return _msg.u.v;
}
