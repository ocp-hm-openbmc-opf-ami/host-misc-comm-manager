/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 Aspeed Technology Inc.
 */
#ifndef _ASPEED_ESPI_IOC_H
#define _ASPEED_ESPI_IOC_H

#include <linux/ioctl.h>
#include <linux/types.h>


#define __ASPEED_ESPI_IOCTL_MAGIC	0xb8

/*
 * The IOCTL-based interface works in the eSPI packet in/out paradigm.
 *
 * Only the virtual wire IOCTL is a special case which does not send
 * or receive an eSPI packet. However, to keep a more consisten use from
 * userspace, we make all of the four channel drivers serve through the
 * IOCTL interface.
 *
 * For the eSPI packet format, refer to
 *   Section 5.1 Cycle Types and Packet Format,
 *   Intel eSPI Interface Base Specification, Rev 1.0, Jan. 2016.
 *
 * For the example user apps using these IOCTL, refer to
 *   https://github.com/AspeedTech-BMC/aspeed_app/tree/master/espi_test
 */


/*
 * Virtual Wire Channel (CH1)
 *  - ASPEED_ESPI_VW_GET_GPIO_VAL
 *      Read the input value of GPIO over the VW channel
 *  - ASPEED_ESPI_VW_PUT_GPIO_VAL
 *      Write the output value of GPIO over the VW channel
 */
#define ASPEED_ESPI_VW_GET_GPIO_VAL	_IOR(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x10, uint8_t)
#define ASPEED_ESPI_VW_PUT_GPIO_VAL	_IOW(__ASPEED_ESPI_IOCTL_MAGIC, \
					     0x11, uint8_t)
#endif
