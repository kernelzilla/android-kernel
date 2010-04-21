/****************************************************************************
**+-----------------------------------------------------------------------+**
**|                                                                       |**
**| Copyright(c) 1998 - 2008 Texas Instruments. All rights reserved.      |**
**| All rights reserved.                                                  |**
**|                                                                       |**
**| Redistribution and use in source and binary forms, with or without    |**
**| modification, are permitted provided that the following conditions    |**
**| are met:                                                              |**
**|                                                                       |**
**|  * Redistributions of source code must retain the above copyright     |**
**|    notice, this list of conditions and the following disclaimer.      |**
**|  * Redistributions in binary form must reproduce the above copyright  |**
**|    notice, this list of conditions and the following disclaimer in    |**
**|    the documentation and/or other materials provided with the         |**
**|    distribution.                                                      |**
**|  * Neither the name Texas Instruments nor the names of its            |**
**|    contributors may be used to endorse or promote products derived    |**
**|    from this software without specific prior written permission.      |**
**|                                                                       |**
**| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   |**
**| "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     |**
**| LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR |**
**| A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  |**
**| OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, |**
**| SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      |**
**| LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, |**
**| DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY |**
**| THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   |**
**| (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE |**
**| OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  |**
**|                                                                       |**
**+-----------------------------------------------------------------------+**
****************************************************************************/


#include <linux/stddef.h>

unsigned long check_stack(unsigned long *base)
{
	
	register unsigned long sp asm ("sp");
	unsigned long retval = sp;
	*base = ((sp & ~0x1fff) + 0x380);
	return retval;
}

unsigned long check_stack_start(unsigned long *base)
{
    unsigned long i;
	unsigned long from,to;

	to = check_stack(&from);
	*base = from;

	/* run from the stack pointer down to the base */
	for (i = from;i<to;i+=4) 
	{
		/* fill up the pattern */
		*(long *)i = 0xdeadbeef;
	}
/*	printk("check_stack_start: from =%x to=%x data=%x\n",from,to,*(long *)(from+4));*/
	return to;
}

unsigned long check_stack_stop(unsigned long *base)
{
    unsigned long i;
	unsigned long from,to;

	to = check_stack(&from);
	*base = from;

	/* run from the stack pointer down to the base */
	for (i = from;i<to;i+=4) 
	{
		/* check up the pattern */
		if ((*(long *)i) != 0xdeadbeef)
			break;
	}

	/*printk("check_stack_stop: from =%x to=%x data=%x data=%x i=0x%x\n",from,to,*(long *)from,*(long *)(from+4),i);*/
	/* return the first time when the pattern doesn't match */
	return i;
}


