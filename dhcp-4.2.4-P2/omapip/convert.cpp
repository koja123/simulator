/* convert.c

   Safe copying of option values into and out of the option buffer, which
   can't be assumed to be aligned. */

/*
 * Copyright (c) 2004,2007,2009 by Internet Systems Consortium, Inc. ("ISC")
 * Copyright (c) 1996-2003 by Internet Software Consortium
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS.  IN NO EVENT SHALL ISC BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *   Internet Systems Consortium, Inc.
 *   950 Charter Street
 *   Redwood City, CA 94063
 *   <info@isc.org>
 *   https://www.isc.org/
 *
 * This software has been written for Internet Systems Consortium
 * by Ted Lemon in cooperation with Vixie Enterprises and Nominum, Inc.
 * To learn more about Internet Systems Consortium, see
 * ``https://www.isc.org/''.  To learn more about Vixie Enterprises,
 * see ``http://www.vix.com''.   To learn more about Nominum, Inc., see
 * ``http://www.nominum.com''.
 */
#include <iscdhcp_porting.h>//ScenSim-Port//
namespace IscDhcpPort {//ScenSim-Port//

//ScenSim-Port//#include "dhcpd.h"

//ScenSim-Port//#include <omapip/omapip_p.h>

//ScenSim-Port//u_int32_t getULong (buf)
//ScenSim-Port//	const unsigned char *buf;
u_int32_t getULong (const unsigned char *buf)//ScenSim-Port//
{
	u_int32_t ibuf;

	memcpy (&ibuf, buf, sizeof (u_int32_t));
	return ntohl (ibuf);
}

//ScenSim-Port//int32_t getLong (buf)
//ScenSim-Port//	const unsigned char *buf;
int32_t getLong (const unsigned char *buf)//ScenSim-Port//
{
	int32_t ibuf;

	memcpy (&ibuf, buf, sizeof (int32_t));
	return ntohl (ibuf);
}

//ScenSim-Port//u_int32_t getUShort (buf)
//ScenSim-Port//	const unsigned char *buf;
u_int32_t getUShort (const unsigned char *buf)//ScenSim-Port//
{
	unsigned short ibuf;

	memcpy (&ibuf, buf, sizeof (u_int16_t));
	return ntohs (ibuf);
}

//ScenSim-Port//int32_t getShort (buf)
//ScenSim-Port//	const unsigned char *buf;
int32_t getShort (const unsigned char *buf)//ScenSim-Port//
{
	short ibuf;

	memcpy (&ibuf, buf, sizeof (int16_t));
	return ntohs (ibuf);
}

//ScenSim-Port//void putULong (obuf, val)
//ScenSim-Port//	unsigned char *obuf;
//ScenSim-Port//	u_int32_t val;
void putULong (unsigned char *obuf, u_int32_t val)//ScenSim-Port//
{
	u_int32_t tmp = htonl (val);
	memcpy (obuf, &tmp, sizeof tmp);
}

//ScenSim-Port//void putLong (obuf, val)
//ScenSim-Port//	unsigned char *obuf;
//ScenSim-Port//	int32_t val;
void putLong (unsigned char *obuf, int32_t val)//ScenSim-Port//
{
	int32_t tmp = htonl (val);
	memcpy (obuf, &tmp, sizeof tmp);
}

//ScenSim-Port//void putUShort (obuf, val)
//ScenSim-Port//	unsigned char *obuf;
//ScenSim-Port//	u_int32_t val;
void putUShort (unsigned char *obuf, u_int32_t val)//ScenSim-Port//
{
	u_int16_t tmp = htons (val);
	memcpy (obuf, &tmp, sizeof tmp);
}

//ScenSim-Port//void putShort (obuf, val)
//ScenSim-Port//	unsigned char *obuf;
//ScenSim-Port//	int32_t val;
void putShort (unsigned char *obuf, int32_t val)//ScenSim-Port//
{
	int16_t tmp = htons (val);
	memcpy (obuf, &tmp, sizeof tmp);
}

//ScenSim-Port//void putUChar (obuf, val)
//ScenSim-Port//	unsigned char *obuf;
//ScenSim-Port//	u_int32_t val;
void putUChar (unsigned char *obuf, u_int32_t val)//ScenSim-Port//
{
	*obuf = val;
}

//ScenSim-Port//u_int32_t getUChar (obuf)
//ScenSim-Port//	const unsigned char *obuf;
u_int32_t getUChar (const unsigned char *obuf)//ScenSim-Port//
{
	return obuf [0];
}

//ScenSim-Port//int converted_length (buf, base, width)
//ScenSim-Port//	const unsigned char *buf;
//ScenSim-Port//	unsigned int base;
//ScenSim-Port//	unsigned int width;
int converted_length (const unsigned char *buf, unsigned int base, unsigned int width)//ScenSim-Port//
{
	u_int32_t number;
	u_int32_t column;
	int power = 1;
	u_int32_t newcolumn = base;

	if (base > 16)
		return 0;

	if (width == 1)
		number = getUChar (buf);
	else if (width == 2)
		number = getUShort (buf);
	else if (width == 4)
		number = getULong (buf);
	else
		return 0;

	do {
		column = newcolumn;

		if (number < column)
			return power;
		power++;
		newcolumn = column * base;
		/* If we wrap around, it must be the next power of two up. */
	} while (newcolumn > column);

	return power;
}

//ScenSim-Port//int binary_to_ascii (outbuf, inbuf, base, width)
//ScenSim-Port//	unsigned char *outbuf;
//ScenSim-Port//	const unsigned char *inbuf;
//ScenSim-Port//	unsigned int base;
//ScenSim-Port//	unsigned int width;
int binary_to_ascii (unsigned char *outbuf, const unsigned char *inbuf, unsigned int base, unsigned int width)//ScenSim-Port//
{
	u_int32_t number;
	static char h2a [] = "0123456789abcdef";
	int power = converted_length (inbuf, base, width);
	int i;

	if (base > 16)
		return 0;

	if (width == 1)
		number = getUChar (inbuf);
	else if (width == 2)
		number = getUShort (inbuf);
	else if (width == 4)
		number = getULong (inbuf);
	else
		return 0;

	for (i = power - 1 ; i >= 0; i--) {
		outbuf [i] = h2a [number % base];
		number /= base;
	}

	return power;
}
}//namespace IscDhcpPort////ScenSim-Port//
