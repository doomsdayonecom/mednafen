/*
    Copyright (C) 1998, 1999, 2000, 2001, 2002, 2003  Charles Mac Donald

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#include "shared.h"

namespace MDFN_IEN_MD
{

uint8 (MDFN_FASTCALL *MD_ExtRead8)(uint32 address) = NULL;
uint16 (MDFN_FASTCALL *MD_ExtRead16)(uint32 address) = NULL;
void (MDFN_FASTCALL *MD_ExtWrite8)(uint32 address, uint8 value) = NULL;
void (MDFN_FASTCALL *MD_ExtWrite16)(uint32 address, uint16 value) = NULL;

alignas(8) uint8 work_ram[0x10000];    /* 68K work RAM */
alignas(8) uint8 zram[0x2000];         /* Z80 work RAM */
uint8 zbusreq;              /* /BUSREQ from Z80 */
uint8 zreset;               /* /RESET to Z80 */
uint8 zbusack;              /* /BUSACK to Z80 */
uint8 zirq;                 /* /IRQ to Z80 */
uint32 zbank;               /* Address of Z80 bank window */

uint8 gen_running;

M68K Main68K;
MDVDP MainVDP;

/*--------------------------------------------------------------------------*/
/* Init, reset, shutdown functions                                          */
/*--------------------------------------------------------------------------*/
void gen_init(void)
{
 Main68K.BusIntAck = Main68K_BusIntAck;
 Main68K.BusReadInstr = Main68K_BusReadInstr;

 Main68K.BusRead8 = Main68K_BusRead8;
 Main68K.BusRead16 = Main68K_BusRead16;

 Main68K.BusWrite8 = Main68K_BusWrite8;
 Main68K.BusWrite16 = Main68K_BusWrite16;

 Main68K.BusRMW = Main68K_BusRMW;

 Main68K.timestamp = 0;
}

void gen_reset(bool poweron)
{
    /* Clear RAM */
    if(poweron)
    {
     memset(work_ram, 0, sizeof(work_ram));
     memset(zram, 0, sizeof(zram));
    }

    gen_running = 1;
    zreset  = 0;    /* Z80 is reset */
    zbusreq = 0;    /* Z80 has control of the Z bus */
    zbusack = 1;    /* Z80 is busy using the Z bus */
    zbank   = 0;    /* Assume default bank is 000000-007FFF */
    zirq    = 0;    /* No interrupts occuring */

    if(poweron)
    {
     gen_io_reset();
    }

    Main68K.Reset(poweron);
    z80_reset();
}

void gen_shutdown(void)
{

}

/*--------------------------------------------------------------------------*/
/* Bus controller chip functions                                            */
/*--------------------------------------------------------------------------*/

int gen_busack_r(void)
{
	//printf("busack_r: %d, %d\n", zbusack, md_timestamp);
    return (zbusack & 1);
}

void gen_busreq_w(int state)
{
    //printf("BUSREQ: %d, %d, %d\n", state, md_timestamp, scanline);
    zbusreq = (state & 1);
    zbusack = 1 ^ (zbusreq & zreset);
}

void gen_reset_w(int state)
{
	//printf("ZRESET: %d, %d\n", state, md_timestamp);
    zreset = (state & 1);
    zbusack = 1 ^ (zbusreq & zreset);

    MDSound_SetYM2612Reset(!zreset);

    if(zreset == 0)
    {
     z80_reset();
    }
}


void gen_bank_w(int state)
{
    zbank = ((zbank >> 1) | ((state & 1) << 23)) & 0xFF8000;
}

/* --- RRDC control accessors (see src/control/md_control.cpp) -------------- *
 * extern "C" so the portable control backend can reach MD state without pulling
 * in the MD headers; the same shape as PCFX_Get* in pcfx/pcfx.cpp. */
extern "C" uint8_t *MD_GetRAM(uint32_t *size_out)
{
    if(size_out) *size_out = sizeof(work_ram);   /* 64 KB 68000 work RAM */
    return work_ram;
}

extern "C" void MD_GetRegs(uint32_t *out)   /* [0]=pc [1]=sr [2..9]=d0-7 [10..17]=a0-7 */
{
    out[0] = Main68K.GetRegister(M68K::GSREG_PC);
    out[1] = Main68K.GetRegister(M68K::GSREG_SR);
    for(unsigned i = 0; i < 8; i++) out[2 + i]  = Main68K.GetRegister(M68K::GSREG_D0 + i);
    for(unsigned i = 0; i < 8; i++) out[10 + i] = Main68K.GetRegister(M68K::GSREG_A0 + i);
}

extern "C" void MD_ControlReset(void) { gen_reset(true); }

}
