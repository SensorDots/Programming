/***************************************************************************
*   Copyright (C) 10/2010 by Olaf Rempel                                  *
*   razzor@kopf-tisch.de                                                  *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; version 2 of the License,               *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

//Modified to work with USB Serial based Arduino I2C Firmware.

  /* | 1byte | 1byte |  1byte  |  1byte  |num_bytes| 0xFF  |
     |ADDRESS|COMMAND|RET_BYTES|NUM_BYTES|  BYTES  |  END  | */
	 
	
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/termios.h>


#include "chipinfo_avr.h"
#include "filedata.h"
#include "list.h"
#include "multiboot.h"
#include "optarg.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*x))

#define twi_arduino_DEFAULT_DEVICE  "/dev/ttyUSB0"

#define READ_BLOCK_SIZE		 64 //128	/* bytes in one flash/eeprom read request */
#define WRITE_BLOCK_SIZE	 16	/* bytes in one eeprom write request */

#define I2C_BOOTLOADER_ADDR 0x07

#define I2C_WRITE 0x00
#define I2C_READ  0x01

#define READ_DELAY 10000
#define WRITE_DELAY 20000


/* SLA+R */
#define CMD_WAIT		    0x00
#define CMD_READ_VERSION	0x01
#define CMD_READ_MEMORY		0x02

/* SLA+W */
#define CMD_SWITCH_APPLICATION	CMD_READ_VERSION
#define CMD_WRITE_MEMORY	CMD_READ_MEMORY

/* CMD_SWITCH_APPLICATION parameter */
#define BOOTTYPE_BOOTLOADER	0x00				/* only in APP */
#define BOOTTYPE_APPLICATION	0x80

/* CMD_{READ|WRITE}_* parameter */
#define MEMTYPE_CHIPINFO	0x00
#define MEMTYPE_FLASH		0x01
#define MEMTYPE_EEPROM		0x02
#define MEMTYPE_PARAMETERS	0x03				/* only in APP */

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		fprintf(stderr, "error %d from tcgetattr\n", errno);
		return -1;
	}
	
	

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		fprintf(stderr, "error %d from tcsetattr\n", errno);
		return -1;
	}
	
	tcsetattr(fd, TCSAFLUSH, &tty);
	
	return 0;
}

int set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
			fprintf(stderr, "error %d from tggetattr\n", errno);
			return -1;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0) {
			fprintf(stderr, "error %d setting term attributes\n", errno);
			return -1;
	}
	return 0;
}


struct multiboot_ops twi_arduino_ops;

struct twi_arduino_privdata {
    char        *device;
    uint8_t     address;
	uint8_t     auto_address;
    int         fd;
    int         connected;
	int         reboot;

    uint8_t     pagesize;
    uint16_t    flashsize;
    uint16_t    eepromsize;
};

static struct option twi_arduino_optargs[] = {
    {"address",     1, 0, 'a'}, /* -a <addr>       */
    {"device",      1, 0, 'd'}, /* [ -d <device> ] */
    {"reboot",      0, 0, 's'}, /* [ -s ] */
};

static int twi_arduino_switch_application(struct twi_arduino_privdata *twi, uint8_t application)
{
	
   /* | 1byte | 1byte |  1byte  |num_bytes| 0xFF  |
	* |ADDRESS|COMMAND|NUM_BYTES|  BYTES  |  END  |
	*/
 
    uint8_t cmd[] = { twi->address, I2C_WRITE, 0, 2, CMD_SWITCH_APPLICATION, application, 0xFF };

    return (write(twi->fd, cmd, sizeof(cmd)) != sizeof(cmd));
}

static int twi_arduino_wait(struct twi_arduino_privdata *twi)
{
 
    uint8_t cmd[] = { twi->address, I2C_WRITE, 0, 1, CMD_WAIT, 0xFF };

    return (write(twi->fd, cmd, sizeof(cmd)) != sizeof(cmd));
}

static int twi_arduino_read_version(struct twi_arduino_privdata *twi, char *version, int length)
{
    uint8_t cmd[] = { twi->address, I2C_READ, length, 1, CMD_READ_VERSION, 0xFF };

    if (write(twi->fd, cmd, sizeof(cmd)) != sizeof(cmd))
        return -1;
	
    //Flush Serial
    tcflush(twi->fd,TCIOFLUSH);

    memset(version, 0, length);
	
	//Wait for data
	usleep(READ_DELAY);
	
	int error = read(twi->fd, version, length);
    if (error != length)
	{ 
        //fprintf(stderr, "length wrong, expected %d, got %d.\n", length, error);
        return -1;
	}
	    

    int i;
    for (i = 0; i < length; i++)
        version[i] &= ~0x80;


    return 0;
}

static int twi_arduino_read_memory(struct twi_arduino_privdata *twi, uint8_t *buffer, uint8_t size, uint8_t memtype, uint16_t address)
{
    uint8_t cmd[] = { twi->address, I2C_READ, size, 4, CMD_READ_MEMORY, memtype, (address >> 8) & 0xFF, (address & 0xFF), 0xFF };
    if (write(twi->fd, cmd, sizeof(cmd)) != sizeof(cmd))
        return -1;
	
	//Flush Serial
    tcflush(twi->fd,TCIOFLUSH);
	
    //Wait for data
	usleep(READ_DELAY);
    return (read(twi->fd, buffer, size) != size);
}

static int twi_arduino_write_memory(struct twi_arduino_privdata *twi, uint8_t *buffer, uint8_t size, uint8_t memtype, uint16_t address)
{
    int bufsize;
    if (memtype == MEMTYPE_FLASH) {
        if ((address & (twi->pagesize -1)) != 0x00) {
            fprintf(stderr, "twi_arduino_write_memory(): address 0x%04x not aligned to pagesize 0x%02x\n", address, twi->pagesize);
            return -1;
        }
        bufsize = 4 + twi->pagesize;

    } else {
        bufsize = 4 + size;
    }
	
	//+5 for extra command structure for arduino firmware
	bufsize = bufsize + 5;
    
    uint8_t *cmd = malloc(bufsize);
    if (cmd == NULL)
        return -1;
	
	cmd[0] = twi->address;
	cmd[1] = I2C_WRITE;
	cmd[2] = 0;
	cmd[3] = twi->pagesize + 4; //Instructions plus memory
    cmd[4] = CMD_WRITE_MEMORY;
    cmd[5] = memtype; 
    cmd[6] = (address >> 8) & 0xFF;
    cmd[7] = (address & 0xFF);
    memcpy(cmd +8, buffer, size);

    if (memtype == MEMTYPE_FLASH) {
        memset(cmd +8 +size, 0xFF, twi->pagesize - size);
    }
	
	//Stop byte
	cmd[bufsize - 1] = 0xFF;

    int result = write(twi->fd, cmd, bufsize);
    free(cmd);
	
	//Flush Serial
    tcflush(twi->fd,TCIOFLUSH);
	
	//Wait for write to occur
	usleep(WRITE_DELAY);

    return (result != bufsize);
}

static void twi_arduino_close_device(struct twi_arduino_privdata *twi)
{
    if (twi->connected)
        close(twi->fd);

    twi->connected = 0;
}

static int twi_arduino_open_device(struct twi_arduino_privdata *twi)
{
	twi->fd =  open(twi->device, O_RDWR | O_NOCTTY | O_SYNC);
    if (twi->fd < 0) {
        fprintf(stderr, "failed to open '%s': %s\n", twi->device, strerror(errno));
        return -1;
    }
	
	if (set_interface_attribs (twi->fd, B115200, 0) < 0) return -1;  // set speed to 115,200 bps, 8n1 (no parity)
    if (set_blocking (twi->fd, 0) < 0) return -1;               // set no blocking
	

    twi->connected = 1;
    return 0;
}

static int twi_arduino_close(struct multiboot *mboot)
{
    struct twi_arduino_privdata *twi = (struct twi_arduino_privdata *)mboot->privdata;
 
 
    //TODO: change this to manual application restart
	
    //if (twi->connected)
    //    twi_arduino_switch_application(twi, BOOTTYPE_APPLICATION);

    twi_arduino_close_device(twi);
    return 0;
}

static int twi_arduino_open(struct multiboot *mboot)
{
    struct twi_arduino_privdata *twi = (struct twi_arduino_privdata *)mboot->privdata;

    if (twi->address == 0) {
        fprintf(stderr, "abort: no address given\n");
        return -1;
    }

    if (twi->device == NULL) {
        twi->device = strdup(twi_arduino_DEFAULT_DEVICE);
        if (twi->device == NULL) {
            perror("strdup()");
            return -1;
        }
    }

    if (twi_arduino_open_device(twi) != 0)
        return -1;
	
    if (twi_arduino_switch_application(twi, BOOTTYPE_BOOTLOADER)) {
        fprintf(stderr, "failed to switch to bootloader (invalid address?): %s\n", strerror(errno));
        twi_arduino_close(mboot);
        return -1;
    }
	
	/* Change to new bootloader address */
	twi->address = I2C_BOOTLOADER_ADDR;

    /* wait for watchdog and startup time */
    usleep(100000);
	
	/* reboot device */
	if (twi->connected && twi->reboot) {
		printf("rebooting device: 0x%02X\n", twi->auto_address);
        twi_arduino_switch_application(twi, BOOTTYPE_APPLICATION);
	} else {
	
		/* Stop bootloader from going into application mode */
		twi_arduino_wait(twi);
		
		
		char version[16];
		if (twi_arduino_read_version(twi, version, sizeof(version))) {
			fprintf(stderr, "failed to get bootloader version: %s\n", strerror(errno));
			twi_arduino_close(mboot);
			return -1;
		}

		uint8_t chipinfo[8];
		if (twi_arduino_read_memory(twi, chipinfo, sizeof(chipinfo), MEMTYPE_CHIPINFO, 0x0000)) {
			fprintf(stderr, "failed to get chipinfo: %s\n", strerror(errno));
			twi_arduino_close(mboot);
			return -1;
		}

		const char *chipname = chipinfo_get_avr_name(chipinfo);

		twi->pagesize   = chipinfo[3];
		twi->flashsize  = (chipinfo[4] << 8) + chipinfo[5];
		twi->eepromsize = (chipinfo[6] << 8) + chipinfo[7];

		printf("device         : %-16s (address: 0x%02X)\n", twi->device, twi->auto_address);
		printf("version        : %-16s (sig: 0x%02x 0x%02x 0x%02x => %s)\n", version, chipinfo[0], chipinfo[1], chipinfo[2], chipname);
		printf("flash size     : 0x%04x / %5d   (0x%02x bytes/page)\n", twi->flashsize, twi->flashsize, twi->pagesize);
		printf("eeprom size    : 0x%04x / %5d\n", twi->eepromsize, twi->eepromsize);
	}

    return 0;
}

static int twi_arduino_read(struct multiboot *mboot, struct databuf *dbuf, int memtype)
{
    struct twi_arduino_privdata *twi = (struct twi_arduino_privdata *)mboot->privdata;
    char *progress_msg = (memtype == MEMTYPE_FLASH) ? "reading flash" : "reading eeprom";

    int pos = 0;
    int size = (memtype == MEMTYPE_FLASH) ? twi->flashsize : twi->eepromsize;
    while (pos < size) {
        mboot->progress_cb(progress_msg, pos, size);
         
        int len = MIN(READ_BLOCK_SIZE, size - pos);
        if (twi_arduino_read_memory(twi, dbuf->data + pos, len, memtype, pos)) {
            mboot->progress_cb(progress_msg, -1, -1);
            return -1;
        }

        pos += len;
    }

    dbuf->length = pos;

    mboot->progress_cb(progress_msg, pos, size);
	
    return 0;
}

static int twi_arduino_write(struct multiboot *mboot, struct databuf *dbuf, int memtype)
{
    struct twi_arduino_privdata *twi = (struct twi_arduino_privdata *)mboot->privdata;
    char *progress_msg = (memtype == MEMTYPE_FLASH) ? "writing flash" : "writing eeprom";

    int pos = 0;
	int len = 0;
    while (pos < dbuf->length) {
        mboot->progress_cb(progress_msg, pos, dbuf->length);

        len = (memtype == MEMTYPE_FLASH) ? twi->pagesize : WRITE_BLOCK_SIZE;

        len = MIN(len, dbuf->length - pos);
        if (twi_arduino_write_memory(twi, dbuf->data + pos, len, memtype, pos)) {
            mboot->progress_cb(progress_msg, -1, -1);
            return -1;
        }

        pos += len;
    }

    mboot->progress_cb(progress_msg, pos, dbuf->length);
	
	//Sleep while adapter goes into known state (command timeout)
	//in case a write failed and caused the adapter to become stuck.
	usleep(1000000);
	
    return 0;
}

static int twi_arduino_verify(struct multiboot *mboot, struct databuf *dbuf, int memtype)
{
    struct twi_arduino_privdata *twi = (struct twi_arduino_privdata *)mboot->privdata;
    char *progress_msg = (memtype == MEMTYPE_FLASH) ? "verifing flash" : "verifing eeprom";


    int pos = 0;
    uint8_t comp[READ_BLOCK_SIZE];
    while (pos < dbuf->length) {
        mboot->progress_cb(progress_msg, pos, dbuf->length);

        int len = MIN(READ_BLOCK_SIZE, dbuf->length - pos);
        if (twi_arduino_read_memory(twi, comp, len, memtype, pos)) {
            mboot->progress_cb(progress_msg, -1, -1);
            return -1;
        }		

        if (memcmp(comp, dbuf->data + pos, len) != 0x00) {
            mboot->progress_cb(progress_msg, -1, -1);
            fprintf(stderr, "verify failed at page 0x%04x!!\n", pos);
            return -1;
        }

        pos += len;
		
    }

    dbuf->length = pos;

    mboot->progress_cb(progress_msg, pos, dbuf->length);
	
    return 0;
}

static int twi_arduino_optarg_cb(int val, const char *arg, void *privdata)
{
    struct twi_arduino_privdata *twi = (struct twi_arduino_privdata *)privdata;

    switch (val) {
    case 'a': /* address */
        {
            char *endptr;
            twi->address = strtol(arg, &endptr, 16);
			twi->auto_address = strtol(arg, &endptr, 16);
            if (*endptr != '\0' || twi->address < 0x01 || twi->address > 0x7F) {
                fprintf(stderr, "invalid address: '%s'\n", arg);
                return -1;
            }
        }
        break;
	

    case 'd': /* device */
        {
            if (twi->device != NULL) {
                fprintf(stderr, "invalid device: '%s'\n", optarg);
                return -1;
            }

            twi->device = strdup(optarg);
            if (twi->device == NULL) {
                perror("strdup()");
                return -1;
            }
        }
        break;
		
	case 's': /* reboot after verify */
		{
			twi->reboot = 1;
		}
		break;

    case 'h':
    case '?': /* error */
            fprintf(stderr, "Usage: twiboot [options]\n"
                "  -a <address>                 - selects i2c address (0x01 - 0x7F)\n"
                "  -d <device>                  - selects Arduino I2C->USB device  (default: /dev/ttyUSB0)\n"
                "  -r <flash|eeprom>:<file>     - reads flash/eeprom to file   (.bin | .hex | -)\n"
                "  -w <flash|eeprom>:<file>     - write flash/eeprom from file (.bin | .hex)\n"
				"  -s                           - reboot into application mode\n"
                "  -n                           - disable verify after write\n"
                "  -p <0|1|2>                   - progress bar mode\n"
                "\n"
                "Example: twiboot -a 0x22 -w flash:blmc.hex -w flash:blmc_eeprom.hex\n"
                "\n");
            return -1;

    default:
        return 1;
    }

    return 0;
}

static struct multiboot * twi_arduino_alloc(void)
{
    struct multiboot * mboot = malloc(sizeof(struct multiboot));
    if (mboot == NULL)
        return NULL;

    memset(mboot, 0x00, sizeof(struct multiboot));
    mboot->ops  = &twi_arduino_ops;

    struct twi_arduino_privdata *twi = malloc(sizeof(struct twi_arduino_privdata));
    if (twi == NULL) {
        free(mboot);
        return NULL;
    }

    memset(twi, 0x00, sizeof(struct twi_arduino_privdata));
    twi->device  = NULL;
    twi->address = 0;
	twi->reboot = 0;

    optarg_register(twi_arduino_optargs, ARRAY_SIZE(twi_arduino_optargs), twi_arduino_optarg_cb, (void *)twi);

    mboot->privdata = twi;
    return mboot;
}

static void twi_arduino_free(struct multiboot *mboot)
{
    struct twi_arduino_privdata *twi = (struct twi_arduino_privdata *)mboot->privdata;

    if (twi->device != NULL)
        free(twi->device);

    free(twi);
    free(mboot);
}

static int twi_arduino_get_memtype(struct multiboot *mboot, const char *memname)
{
    if (strcmp(memname, "flash") == 0)
        return MEMTYPE_FLASH;

    else if (strcmp(memname, "eeprom") == 0)
        return MEMTYPE_EEPROM;

    return -1;
}

static int twi_arduino_get_memsize(struct multiboot *mboot, int memtype)
{
    struct twi_arduino_privdata *twi = (struct twi_arduino_privdata *)mboot->privdata;

    if (!twi->connected)
        return 0;

    switch (memtype) {
    case MEMTYPE_FLASH:
        return twi->flashsize;

    case MEMTYPE_EEPROM:
        return twi->eepromsize;

    default:
        return 0;
    }
}

struct multiboot_ops twi_arduino_ops = {
    .alloc          = twi_arduino_alloc,
    .free           = twi_arduino_free,
    .get_memtype    = twi_arduino_get_memtype,
    .get_memsize    = twi_arduino_get_memsize,

    .open           = twi_arduino_open,
    .close          = twi_arduino_close,
    .read           = twi_arduino_read,
    .write          = twi_arduino_write,
    .verify         = twi_arduino_verify,
};
