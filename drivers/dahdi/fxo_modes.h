/*
 * FXO port mode settings for various regions
 *
 * Copyright (C) 2008 Digium, Inc.
 *
 * extracted from wctdm.c by
 * Kevin P. Fleming <kpfleming@digium.com>
 *
 * All rights reserved.
 */

/*
 * See http://www.asterisk.org for more information about
 * the Asterisk project. Please do not directly contact
 * any of the maintainers of this project for assistance;
 * the project provides a web site, mailing lists and IRC
 * channels for your use.
 *
 * This program is free software, distributed under the terms of
 * the GNU General Public License Version 2 as published by the
 * Free Software Foundation. See the LICENSE file included with
 * this program for more details.
 */

#ifndef _FXO_MODES_H
#define _FXO_MODES_H

static struct fxo_mode {
	char *name;
	int ohs;
	int ohs2;
	int rz;
	int rt;
	int ilim;
	int dcv;
	int mini;
	int acim;
	int ring_osc;
	int ring_x;
	unsigned int battdebounce; /* in milliseconds */
	unsigned int battalarm; /* in milliseconds */
	unsigned int battthresh; /* unknown units */
} fxo_modes[] =
{
 	/* US, Canada */
	{ .name = "FCC",
	  .rt = 1,
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	/* Austria, Belgium, Denmark, Finland, France, Germany, 
	   Greece, Iceland, Ireland, Italy, Luxembourg, Netherlands,
	   Norway, Portugal, Spain, Sweden, Switzerland, and UK */
	{ .name = "TBR21",
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .ring_osc = 0x7e6c,
	  .ring_x = 0x023a,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "ARGENTINA",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "AUSTRALIA",
	  .ohs = 1,
	  .mini = 0x3,
	  .acim = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "AUSTRIA",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "BAHRAIN",
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "BELGIUM",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "BRAZIL",
	  .mini = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "BULGARIA",
	  .ilim = 1,
	  .dcv = 0x3,
	  .mini = 0x0,
	  .acim = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "CANADA",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "CHILE",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "CHINA",
	  .mini = 0x3,
	  .acim = 0xf,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "COLOMBIA",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "CROATIA",
	  .ilim = 1,
	  .dcv = 0x3,
	  .mini = 0,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "CYPRUS",
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "CZECH",
	  .ilim = 1,
	  .dcv = 0x3,
	  .mini = 0,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "DENMARK",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "ECUADOR",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "EGYPT",
	  .mini = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "ELSALVADOR",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "FINLAND",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "FRANCE",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .mini = 0,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "GERMANY",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "GREECE",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "GUAM",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "HONGKONG",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "HUNGARY",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "ICELAND",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "INDIA",
	  .dcv = 0x3,
	  .acim = 0x4,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "INDONESIA",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "IRELAND",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "ISRAEL",
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "ITALY",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "JAPAN",
	  .mini = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "JORDAN",
	  .mini = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "KAZAKHSTAN",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "KUWAIT",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "LATVIA",
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "LEBANON",
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "LUXEMBOURG",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "MACAO",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	/* Current loop >= 20ma */
	{ .name = "MALAYSIA",
	  .mini = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "MALTA",
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "MEXICO",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "MOROCCO",
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "NETHERLANDS",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "NEWZEALAND",
	  .dcv = 0x3,
	  .acim = 0x4,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "NIGERIA",
	  .ilim = 0x1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "NORWAY",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "OMAN",
	  .mini = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "PAKISTAN",
	  .mini = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "PERU",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "PHILIPPINES",
	  .mini = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "POLAND",
	  .rz = 1,
	  .rt = 1,
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "PORTUGAL",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "ROMANIA",
	  .dcv = 3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "RUSSIA",
	  .mini = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "SAUDIARABIA",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "SINGAPORE",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "SLOVAKIA",
	  .dcv = 0x3,
	  .acim = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "SLOVENIA",
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "SOUTHAFRICA",
	  .ohs = 1,
	  .rz = 1,
	  .dcv = 0x3,
	  .acim = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "SOUTHKOREA",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "SPAIN",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "SWEDEN",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "SWITZERLAND",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x2,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "SYRIA",
	  .mini = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "TAIWAN",
	  .mini = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "THAILAND",
	  .mini = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "UAE",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "UK",
	  .ohs2 = 1,
	  .ilim = 1,
	  .dcv = 0x3,
	  .acim = 0x5,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "USA",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
	{ .name = "YEMEN",
	  .dcv = 0x3,
	  .battdebounce = 64,
	  .battalarm = 1000,
	  .battthresh = 3,
	},
};

#endif /* _FXO_MODES_H */
