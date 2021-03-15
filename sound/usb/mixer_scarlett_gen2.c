// SPDX-License-Identifier: GPL-2.0
/*
 *   Focusrite Scarlett 6i6/18i8/18i20 Gen 2 and 4i4/8i6/18i8/18i20
 *   Gen 3 Driver for ALSA
 *
 *   Copyright (c) 2018-2020 by Geoffrey D. Bennett <g at b4.vu>
 *   Copyright (c) 2020 by Vladimir Sadovnikov <sadko4u at gmail.com>
 *
 *   Based on the Scarlett (Gen 1) Driver for ALSA:
 *
 *   Copyright (c) 2013 by Tobias Hoffmann
 *   Copyright (c) 2013 by Robin Gareus <robin at gareus.org>
 *   Copyright (c) 2002 by Takashi Iwai <tiwai at suse.de>
 *   Copyright (c) 2014 by Chris J Arges <chris.j.arges at canonical.com>
 *
 *   Many codes borrowed from audio.c by
 *     Alan Cox (alan at lxorguk.ukuu.org.uk)
 *     Thomas Sailer (sailer at ife.ee.ethz.ch)
 *
 *   Code cleanup:
 *   David Henningsson <david.henningsson at canonical.com>
 */

/* Mixer Interface for the Focusrite Scarlett 6i6/18i8/18i20 Gen 2 and
 * 4i4/8i6/18i8/18i20 Gen 3 audio interfaces. Based on the Gen 1
 * driver and rewritten.
 */

/* The protocol was reverse engineered by looking at the communication
 * between Focusrite Control 2.3.4 and the Focusrite(R) Scarlett 18i20
 * (firmware 1083) using usbmon in July-August 2018.
 *
 * Scarlett 18i8 Gen 2 support added in April 2019.
 *
 * Scarlett 6i6 Gen 2 support added in June 2019 (thanks to Martin
 * Wittmann for providing usbmon output and testing).
 *
 * Scarlett 4i4/8i6 Gen 3 support added in May 2020 (thanks to Laurent
 * Debricon for donating a 4i4 and to Fredrik Unger for providing 8i6
 * usbmon output and testing).
 *
 * Scarlett 18i8/18i20 Gen 3 support added in June 2020 (thanks to
 * Darren Jaeckel, Alex Sedlack, and Clovis Lunel for providing usbmon
 * output and testing).
 *
 * This ALSA mixer gives access to (model-dependent):
 *  - input, output, mixer-matrix muxes
 *  - 18x10 mixer-matrix gain stages
 *  - gain/volume controls
 *  - level meters
 *  - line/inst level, pad, and air controls
 *  - enable/disable MSD mode
 *  - main/alt speaker switching
 *
 * <ditaa>
 *    /--------------\    18chn            20chn     /--------------\
 *    | Hardware  in +--+------\    /-------------+--+ ALSA PCM out |
 *    \--------------/  |      |    |             |  \--------------/
 *                      |      |    |    /-----\  |
 *                      |      |    |    |     |  |
 *                      |      v    v    v     |  |
 *                      |   +---------------+  |  |
 *                      |    \ Matrix  Mux /   |  |
 *                      |     +-----+-----+    |  |
 *                      |           |          |  |
 *                      |           |18chn     |  |
 *                      |           |          |  |
 *                      |           |     10chn|  |
 *                      |           v          |  |
 *                      |     +------------+   |  |
 *                      |     | Mixer      |   |  |
 *                      |     |     Matrix |   |  |
 *                      |     |            |   |  |
 *                      |     | 18x10 Gain |   |  |
 *                      |     |   stages   |   |  |
 *                      |     +-----+------+   |  |
 *                      |           |          |  |
 *                      |18chn      |10chn     |  |20chn
 *                      |           |          |  |
 *                      |           +----------/  |
 *                      |           |             |
 *                      v           v             v
 *                      ===========================
 *               +---------------+       +--—------------+
 *                \ Output  Mux /         \ Capture Mux /
 *                 +---+---+---+           +-----+-----+
 *                     |   |                     |
 *                10chn|   |                     |18chn
 *                     |   |                     |
 *  /--------------\   |   |                     |   /--------------\
 *  | S/PDIF, ADAT |<--/   |10chn                \-->| ALSA PCM in  |
 *  | Hardware out |       |                         \--------------/
 *  \--------------/       |
 *                         v
 *                  +-------------+    Software gain per channel.
 *                  | Master Gain |<-- 18i20 only: Switch per channel
 *                  +------+------+    to select HW or SW gain control.
 *                         |
 *                         |10chn
 *  /--------------\       |
 *  | Analogue     |<------/
 *  | Hardware out |
 *  \--------------/
 * </ditaa>
 *
 * Gen 3 devices have a Mass Storage Device (MSD) mode where a small
 * disk with registration and driver download information is presented
 * to the host. To access the full functionality of the device without
 * proprietary software, MSD mode can be disabled by:
 * - holding down the 48V button for five seconds while powering on
 *   the device, or
 * - using this driver and alsamixer to change the "MSD Mode" setting
 *   to Off, waiting two seconds, then power-cycling the device
 */

#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/moduleparam.h>

#include <sound/control.h>
#include <sound/tlv.h>

#include "usbaudio.h"
#include "mixer.h"
#include "helper.h"

#include "mixer_scarlett_gen2.h"

/* device_setup value to enable */
#define SCARLETT2_ENABLE 0x01

/* device_setup value to allow turning MSD mode back on */
#define SCARLETT2_MSD_ENABLE 0x02

/* some gui mixers can't handle negative ctl values */
#define SCARLETT2_VOLUME_BIAS 127

/* mixer range from -80dB to +6dB in 0.5dB steps */
#define SCARLETT2_MIXER_MIN_DB -80
#define SCARLETT2_MIXER_BIAS (-SCARLETT2_MIXER_MIN_DB * 2)
#define SCARLETT2_MIXER_MAX_DB 6
#define SCARLETT2_MIXER_MAX_VALUE \
	((SCARLETT2_MIXER_MAX_DB - SCARLETT2_MIXER_MIN_DB) * 2)

/* map from (dB + 80) * 2 to mixer value
 * for dB in 0 .. 172: int(8192 * pow(10, ((dB - 160) / 2 / 20)))
 */
static const u16 scarlett2_mixer_values[173] = {
	/* 8 items per row */
	0, 0, 0, 0, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	2, 2, 2, 2, 2, 2, 2, 3,
	3, 3, 3, 3, 4, 4, 4, 4,
	5, 5, 5, 6, 6, 6, 7, 7,
	8, 8, 9, 9, 10, 10, 11, 12,
	12, 13, 14, 15, 16, 17, 18, 19,
	20, 21, 23, 24, 25, 27, 29, 30,
	32, 34, 36, 38, 41, 43, 46, 48,
	51, 54, 57, 61, 65, 68, 73, 77,
	81, 86, 91, 97, 103, 109, 115, 122,
	129, 137, 145, 154, 163, 173, 183, 194,
	205, 217, 230, 244, 259, 274, 290, 307,
	326, 345, 365, 387, 410, 434, 460, 487,
	516, 547, 579, 614, 650, 689, 730, 773,
	819, 867, 919, 973, 1031, 1092, 1157, 1225,
	1298, 1375, 1456, 1543, 1634, 1731, 1833, 1942,
	2057, 2179, 2308, 2445, 2590, 2744, 2906, 3078,
	3261, 3454, 3659, 3876, 4105, 4349, 4606, 4879,
	5168, 5475, 5799, 6143, 6507, 6892, 7301, 7733,
	8192, 8677, 9191, 9736, 10313, 10924, 11571, 12257,
	12983, 13752, 14567, 15430, 16345
};

/* This is array of high parts of the 32-bit floating point values
 * which are matching the -80..+6 dB level with 0.5 dB step
 * The lowest value is encoded as -128.0f for compatibility with
 * the original software
 */
static const u16 scarlett2_sw_config_mixer_values[173] = {
	/* 8 items per row */
	0xc300, 0xc29f, 0xc29e, 0xc29d, 0xc29c, 0xc29b, 0xc29a, 0xc299,
	0xc298, 0xc297, 0xc296, 0xc295, 0xc294, 0xc293, 0xc292, 0xc291,
	0xc290, 0xc28f, 0xc28e, 0xc28d, 0xc28c, 0xc28b, 0xc28a, 0xc289,
	0xc288, 0xc287, 0xc286, 0xc285, 0xc284, 0xc283, 0xc282, 0xc281,
	0xc280, 0xc27e, 0xc27c, 0xc27a, 0xc278, 0xc276, 0xc274, 0xc272,
	0xc270, 0xc26e, 0xc26c, 0xc26a, 0xc268, 0xc266, 0xc264, 0xc262,
	0xc260, 0xc25e, 0xc25c, 0xc25a, 0xc258, 0xc256, 0xc254, 0xc252,
	0xc250, 0xc24e, 0xc24c, 0xc24a, 0xc248, 0xc246, 0xc244, 0xc242,
	0xc240, 0xc23e, 0xc23c, 0xc23a, 0xc238, 0xc236, 0xc234, 0xc232,
	0xc230, 0xc22e, 0xc22c, 0xc22a, 0xc228, 0xc226, 0xc224, 0xc222,
	0xc220, 0xc21e, 0xc21c, 0xc21a, 0xc218, 0xc216, 0xc214, 0xc212,
	0xc210, 0xc20e, 0xc20c, 0xc20a, 0xc208, 0xc206, 0xc204, 0xc202,
	0xc200, 0xc1fc, 0xc1f8, 0xc1f4, 0xc1f0, 0xc1ec, 0xc1e8, 0xc1e4,
	0xc1e0, 0xc1dc, 0xc1d8, 0xc1d4, 0xc1d0, 0xc1cc, 0xc1c8, 0xc1c4,
	0xc1c0, 0xc1bc, 0xc1b8, 0xc1b4, 0xc1b0, 0xc1ac, 0xc1a8, 0xc1a4,
	0xc1a0, 0xc19c, 0xc198, 0xc194, 0xc190, 0xc18c, 0xc188, 0xc184,
	0xc180, 0xc178, 0xc170, 0xc168, 0xc160, 0xc158, 0xc150, 0xc148,
	0xc140, 0xc138, 0xc130, 0xc128, 0xc120, 0xc118, 0xc110, 0xc108,
	0xc100, 0xc0f0, 0xc0e0, 0xc0d0, 0xc0c0, 0xc0b0, 0xc0a0, 0xc090,
	0xc080, 0xc060, 0xc040, 0xc020, 0xc000, 0xbfc0, 0xbf80, 0xbf00,
	0x0000, 0x3f00, 0x3f80, 0x3fc0, 0x4000, 0x4020, 0x4040, 0x4060,
	0x4080, 0x4090, 0x40a0, 0x40b0, 0x40c0
};

/* Maximum number of analogue outputs */
#define SCARLETT2_ANALOGUE_MAX 10

/* Maximum number of level, pad, and air switches */
#define SCARLETT2_LEVEL_SWITCH_MAX 2
#define SCARLETT2_PAD_SWITCH_MAX 8
#define SCARLETT2_AIR_SWITCH_MAX 8
#define SCARLETT2_48V_SWITCH_MAX 2

/* Maximum number of inputs to the mixer */
#define SCARLETT2_INPUT_MIX_MAX 24

/* Maximum number of outputs from the mixer */
#define SCARLETT2_OUTPUT_MIX_MAX 12

/* Maximum size of the data in the USB mux assignment message:
 * 18 inputs, 2 loopbacks, 20 outputs, 24 mixer inputs, 13 spare
 */
#define SCARLETT2_MUX_MAX 77

/* Number of meters:
 * 18 inputs, 20 outputs, 18 matrix inputs (XX FIXME)
 */
#define SCARLETT2_NUM_METERS 56

/* Hardware port types:
 * - None (no input to mux)
 * - Analogue I/O
 * - S/PDIF I/O
 * - ADAT I/O
 * - Mixer I/O
 * - PCM I/O
 */
enum {
	SCARLETT2_PORT_TYPE_ANALOGUE = 0,       /* Analogue input/output */
	SCARLETT2_PORT_TYPE_SPDIF = 1,          /* S/PDIF input/oputput  */
	SCARLETT2_PORT_TYPE_ADAT = 2,           /* ADAT input/output     */
	SCARLETT2_PORT_TYPE_MIX = 3,            /* Mixer input/output    */
	SCARLETT2_PORT_TYPE_PCM = 4,            /* PCM input/output      */
	SCARLETT2_PORT_TYPE_INT_MIC = 5,        /* Internal microphone   */
	SCARLETT2_PORT_TYPE_TALKBACK = 6,       /* Talkback source       */
	SCARLETT2_PORT_TYPE_COUNT = 7
};

enum {
	SCARLETT2_PORT_ID_NONE = 0,
	SCARLETT2_PORT_ID_ANALOGUE = 0x80,
	SCARLETT2_PORT_ID_SPDIF = 0x180,
	SCARLETT2_PORT_ID_ADAT = 0x200,
	SCARLETT2_PORT_ID_MIX = 0x300,
	SCARLETT2_PORT_ID_PCM = 0x600,
	SCARLETT2_PORT_ID_MASK  = 0x0f80,
	SCARLETT2_PORT_NUM_MASK = 0x007f
};

/* Count of total I/O and number available at each sample rate */
enum {
	SCARLETT2_PORT_IN = 0,
	SCARLETT2_PORT_OUT = 1,
	SCARLETT2_PORT_OUT_44 = 2,
	SCARLETT2_PORT_OUT_88 = 3,
	SCARLETT2_PORT_OUT_176 = 4,
	SCARLETT2_PORT_DIRECTIONS = 5,
};

/* Hardware buttons on the 18i20 */
#define SCARLETT2_BUTTON_MAX 2

static const char *const scarlett2_button_names[SCARLETT2_BUTTON_MAX] = {
	"Mute", "Dim"
};

struct scarlett2_port_name {
	s8 direction;  /* Direction of port */
	s8 type;  /* Type of port - SCARLETT2_PORT_TYPE_* */
	s8 index; /* Index of port */
	const char *name; /* The name of port */
};

/* Description of each hardware port type:
 * - id: hardware ID for this port type
 * - num: number of sources/destinations of this port type
 * - src_descr: printf format string for mux input selections
 * - src_num_offset: added to channel number for the fprintf
 * - dst_descr: printf format string for mixer controls
 */
struct scarlett2_ports {
	u16 id;
	int num[SCARLETT2_PORT_DIRECTIONS];
	const char * const src_descr;
	int src_num_offset;
	const char * const dst_descr;
};

struct scarlett2_device_info {
	u32 usb_id; /* USB device identifier */
	u8 line_out_hw_vol; /* line out hw volume is sw controlled */
	u8 button_count; /* number of buttons */
	u8 level_input_count; /* inputs with level selectable */
	u8 pad_input_count; /* inputs with pad selectable */
	u8 air_input_count; /* inputs with air selectable */
	u8 power_48v_count; /* 48V phantom power */
	u8 has_msd_mode; /* Gen 3 devices have an internal MSD mode switch */
	u8 has_speaker_switching; /* main/alt speaker switching */
	u8 has_talkback; /* 18i20 Gen 3 has 'talkback' feature */
	const char * const line_out_descrs[SCARLETT2_ANALOGUE_MAX];
	const struct scarlett2_port_name * const port_names; /* Special names of ports */
	const u8 mux_size[SCARLETT2_PORT_DIRECTIONS]; /* The maximum number of elements per mux */
	struct scarlett2_ports ports[SCARLETT2_PORT_TYPE_COUNT];
};

struct scarlett2_sw_cfg_volume {
	__le16 volume;  /* volume */
	u8 changed;     /* change flag */
	u8 flags;       /* some flags? */
};

struct scarlett2_mixer_data {
	struct usb_mixer_interface *mixer;
	struct mutex usb_mutex; /* prevent sending concurrent USB requests */
	struct mutex data_mutex; /* lock access to this data */
	struct delayed_work work;
	const struct scarlett2_device_info *info;
	__u8 interface; /* vendor-specific interface number */
	__u8 endpoint; /* interrupt endpoint address */
	__le16 maxpacketsize;
	__u8 interval;
	int num_inputs; /* Overall number of inputs */
	int num_outputs; /* Overall number of outputs */
	u16 scarlett2_seq;
	u8 vol_updated; /* Flag that indicates that volume has been updated */
	u8 line_ctl_updated; /* Flag that indicates that state of PAD, INST buttons have been updated */
	u8 speaker_updated; /* Flag that indicates that speaker/talkback has been updated */
	u8 master_vol;
	u8 vol[SCARLETT2_ANALOGUE_MAX];
	u8 vol_sw_hw_switch[SCARLETT2_ANALOGUE_MAX];
	u8 level_switch[SCARLETT2_LEVEL_SWITCH_MAX];
	u8 pad_switch[SCARLETT2_PAD_SWITCH_MAX];
	u8 air_switch[SCARLETT2_AIR_SWITCH_MAX];
	u8 pow_switch[SCARLETT2_48V_SWITCH_MAX];
	u8 msd_switch;
	u8 speaker_switch;
	u8 talkback_switch;
	u8 buttons[SCARLETT2_BUTTON_MAX];
	struct snd_kcontrol *master_vol_ctl;
	struct snd_kcontrol *speaker_ctl;
	struct snd_kcontrol *talkback_ctl;
	struct snd_kcontrol *vol_ctls[SCARLETT2_ANALOGUE_MAX];
	struct snd_kcontrol *pad_ctls[SCARLETT2_PAD_SWITCH_MAX];
	struct snd_kcontrol *level_ctls[SCARLETT2_LEVEL_SWITCH_MAX];
	struct snd_kcontrol *pow_ctls[SCARLETT2_48V_SWITCH_MAX];
	struct snd_kcontrol *button_ctls[SCARLETT2_BUTTON_MAX];
	struct snd_kcontrol *mix_talkback_ctls[SCARLETT2_OUTPUT_MIX_MAX]; /* Talkback controls for each mix */
	s8 mux[SCARLETT2_MUX_MAX];                                        /* Routing of outputs */
	u8 mix[SCARLETT2_INPUT_MIX_MAX * SCARLETT2_OUTPUT_MIX_MAX];       /* Matrix mixer */
	u8 mix_talkback[SCARLETT2_OUTPUT_MIX_MAX];                        /* Talkback enable for mixer output */

	/* Software configuration */
	int sw_cfg_offset;                                                /* Software configuration offset */
	int sw_cfg_size;                                                  /* Software configuration size   */
	u8 *sw_cfg_data;                                                  /* Software configuration (data) */
	__le32 *sw_cfg_stereo;                                            /* Bitmask containing flags that output is in STEREO mode */
	struct scarlett2_sw_cfg_volume *sw_cfg_volume;                    /* Output volumes of the device, f32le */
	__le32 *sw_cfg_mixer;                                             /* Mixer data stored in the software config, matrix of f32le values */
	int sw_cfg_mixer_size;                                            /* Size of software mixer area */
	__le32 *sw_cfg_cksum;                                             /* Software configuration checksum */
};

/*** Model-specific data ***/
static const struct scarlett2_port_name s6i6_gen2_ports[] = {
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 0, "Headphones 1 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 1, "Headphones 1 R" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 2, "Headphones 2 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 3, "Headphones 2 R" },
	{ -1, -1, -1, NULL }
};

static const struct scarlett2_device_info s6i6_gen2_info = {
	.usb_id = USB_ID(0x1235, 0x8203),
	
	/* The first two analogue inputs can be switched between line
	 * and instrument levels.
	 */
	.level_input_count = 2,

	/* The first two analogue inputs have an optional pad. */
	.pad_input_count = 2,

	.line_out_descrs = {
		"Headphones 1 L",
		"Headphones 1 R",
		"Headphones 2 L",
		"Headphones 2 R",
	},

	.port_names = s6i6_gen2_ports,

	.mux_size = { 77, 77, 77, 73, 46 },

	.ports = {
		[SCARLETT2_PORT_TYPE_ANALOGUE] = {
			.id = SCARLETT2_PORT_ID_ANALOGUE,
			.num = { 4, 4, 4, 4, 4 },
			.src_descr = "Analogue In %02d",
			.src_num_offset = 1,
			.dst_descr = "Analogue Out %02d"
		},
		[SCARLETT2_PORT_TYPE_SPDIF] = {
			.id = SCARLETT2_PORT_ID_SPDIF,
			.num = { 2, 2, 2, 2, 2 },
			.src_descr = "S/PDIF In %d",
			.src_num_offset = 1,
			.dst_descr = "S/PDIF Out %d"
		},
		[SCARLETT2_PORT_TYPE_MIX] = {
			.id = SCARLETT2_PORT_ID_MIX,
			.num = { 10, 18, 18, 18, 18 },
			.src_descr = "Mix %c Out",
			.src_num_offset = 'A',
			.dst_descr = "Mix In %02d"
		},
		[SCARLETT2_PORT_TYPE_PCM] = {
			.id = SCARLETT2_PORT_ID_PCM,
			.num = { 6, 6, 6, 6, 6 },
			.src_descr = "PCM In %d",
			.src_num_offset = 1,
			.dst_descr = "PCM Out %02d"
		},
	},
};

static const struct scarlett2_port_name s18i8_gen2_port_names[] = {
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 0, "Monitor L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 1, "Monitor R" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 2, "Headphones 1 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 3, "Headphones 1 R" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 4, "Headphones 2 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 5, "Headphones 2 R" },
	{ -1, -1, -1, NULL }
};

static const struct scarlett2_device_info s18i8_gen2_info = {
	.usb_id = USB_ID(0x1235, 0x8204),

	/* The first two analogue inputs can be switched between line
	 * and instrument levels.
	 */
	.level_input_count = 2,

	/* The first four analogue inputs have an optional pad. */
	.pad_input_count = 4,

	.line_out_descrs = {
		"Monitor L",
		"Monitor R",
		"Headphones 1 L",
		"Headphones 1 R",
		"Headphones 2 L",
		"Headphones 2 R",
	},

	.port_names = s18i8_gen2_port_names,

	.mux_size = { 77, 77, 77, 73, 46 },

	.ports = {
		[SCARLETT2_PORT_TYPE_ANALOGUE] = {
			.id = SCARLETT2_PORT_ID_ANALOGUE,
			.num = { 8, 6, 6, 6, 6 },
			.src_descr = "Analogue In %02d",
			.src_num_offset = 1,
			.dst_descr = "Analogue Out %02d"
		},
		[SCARLETT2_PORT_TYPE_SPDIF] = {
			.id = SCARLETT2_PORT_ID_SPDIF,
			/* S/PDIF outputs aren't available at 192kHz
			 * but are included in the USB mux I/O
			 * assignment message anyway
			 */
			.num = { 2, 2, 2, 2, 2 },
			.src_descr = "S/PDIF In %d",
			.src_num_offset = 1,
			.dst_descr = "S/PDIF Out %d"
		},
		[SCARLETT2_PORT_TYPE_ADAT] = {
			.id = SCARLETT2_PORT_ID_ADAT,
			.num = { 8, 0, 0, 0, 0 },
			.src_descr = "ADAT In %d",
			.src_num_offset = 1,
			.dst_descr = "ADAT Out %d",
		},
		[SCARLETT2_PORT_TYPE_MIX] = {
			.id = SCARLETT2_PORT_ID_MIX,
			.num = { 10, 18, 18, 18, 18 },
			.src_descr = "Mix %c Out",
			.src_num_offset = 'A',
			.dst_descr = "Mix In %02d"
		},
		[SCARLETT2_PORT_TYPE_PCM] = {
			.id = SCARLETT2_PORT_ID_PCM,
			.num = { 8, 18, 18, 14, 10 },
			.src_descr = "PCM In %02d",
			.src_num_offset = 1,
			.dst_descr = "PCM Out %02d"
		},
	},
};

static const struct scarlett2_port_name s18i20_gen2_port_names[] = {
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 0, "Monitor L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 1, "Monitor R" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 6, "Headphones 1 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 7, "Headphones 1 R" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 8, "Headphones 2 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 9, "Headphones 2 R" },
	{ -1, -1, -1, NULL }
};

static const struct scarlett2_device_info s18i20_gen2_info = {
	.usb_id = USB_ID(0x1235, 0x8201),

	/* The analogue line outputs on the 18i20 can be switched
	 * between software and hardware volume control
	 */
	.line_out_hw_vol = 1,

	/* Mute and dim buttons */
	.button_count = 2,

	.line_out_descrs = {
		"Monitor L",
		"Monitor R",
		NULL,
		NULL,
		NULL,
		NULL,
		"Headphones 1 L",
		"Headphones 1 R",
		"Headphones 2 L",
		"Headphones 2 R",
	},

	.port_names = s18i8_gen2_port_names,

	.mux_size = { 77, 77, 77, 73, 46 },

	.ports = {
		[SCARLETT2_PORT_TYPE_ANALOGUE] = {
			.id = SCARLETT2_PORT_ID_ANALOGUE,
			.num = { 8, 10, 10, 10, 10 },
			.src_descr = "Analogue In %02d",
			.src_num_offset = 1,
			.dst_descr = "Analogue Out %02d"
		},
		[SCARLETT2_PORT_TYPE_SPDIF] = {
			/* S/PDIF outputs aren't available at 192kHz
			 * but are included in the USB mux I/O
			 * assignment message anyway
			 */
			.id = SCARLETT2_PORT_ID_SPDIF,
			.num = { 2, 2, 2, 2, 2 },
			.src_descr = "S/PDIF In %d",
			.src_num_offset = 1,
			.dst_descr = "S/PDIF Out %d"
		},
		[SCARLETT2_PORT_TYPE_ADAT] = {
			.id = SCARLETT2_PORT_ID_ADAT,
			.num = { 8, 8, 8, 4, 0 },
			.src_descr = "ADAT In %d",
			.src_num_offset = 1,
			.dst_descr = "ADAT Out %d"
		},
		[SCARLETT2_PORT_TYPE_MIX] = {
			.id = SCARLETT2_PORT_ID_MIX,
			.num = { 10, 18, 18, 18, 18 },
			.src_descr = "Mix %c Out",
			.src_num_offset = 'A',
			.dst_descr = "Mix In %02d"
		},
		[SCARLETT2_PORT_TYPE_PCM] = {
			.id = SCARLETT2_PORT_ID_PCM,
			.num = { 20, 18, 18, 14, 10 },
			.src_descr = "PCM In %02d",
			.src_num_offset = 1,
			.dst_descr = "PCM Out %02d"
		},
	},
};

static const struct scarlett2_port_name s4i4_gen3_port_names[] = {
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 0, "Monitor L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 1, "Monitor R" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 2, "Headphones L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 3, "Headphones R" },
	{ -1, -1, -1, NULL }
};

static const struct scarlett2_device_info s4i4_gen3_info = {
	.usb_id = USB_ID(0x1235, 0x8212),

	/* The first two analogue inputs can be switched between line
	 * and instrument levels.
	 */
	.level_input_count = 2,

	/* The first two analogue inputs have an optional pad. */
	.pad_input_count = 2,

	/* The first two analogue inputs have an optional "air" feature. */
	.air_input_count = 2,
	
	/* One 48V phantom power switch */
	.power_48v_count = 1,

	/* Gen 3 devices have an MSD mode */
	.has_msd_mode = 1,

	.line_out_descrs = {
		"Monitor L",
		"Monitor R",
		"Headphones L",
		"Headphones R",
	},

	.port_names = s4i4_gen3_port_names,

	.mux_size = { 77, 77, 77, 73, 46 },

	.ports = {
		[SCARLETT2_PORT_TYPE_ANALOGUE] = {
			.id = SCARLETT2_PORT_ID_ANALOGUE,
			.num = { 4, 4, 4, 4, 4 },
			.src_descr = "Analogue In %02d",
			.src_num_offset = 1,
			.dst_descr = "Analogue Out %02d"
		},
		[SCARLETT2_PORT_TYPE_MIX] = {
			.id = SCARLETT2_PORT_ID_MIX,
			.num = { 6, 8, 8, 8, 8 },
			.src_descr = "Mix %c Out",
			.src_num_offset = 'A',
			.dst_descr = "Mix In %02d"
		},
		[SCARLETT2_PORT_TYPE_PCM] = {
			.id = SCARLETT2_PORT_ID_PCM,
			.num = { 4, 6, 6, 6, 6 },
			.src_descr = "PCM In %02d",
			.src_num_offset = 1,
			.dst_descr = "PCM Out %02d"
		},
	},
};

static const struct scarlett2_port_name s8i6_gen3_port_names[] = {
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 0, "Headphones 1 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 1, "Headphones 1 R" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 2, "Headphones 2 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 3, "Headphones 3 R" },
	{ -1, -1, -1, NULL }
};

static const struct scarlett2_device_info s8i6_gen3_info = {
	.usb_id = USB_ID(0x1235, 0x8213),

	/* The first two analogue inputs can be switched between line
	 * and instrument levels.
	 */
	.level_input_count = 2,

	/* The first two analogue inputs have an optional pad. */
	.pad_input_count = 2,

	/* The first two analogue inputs have an optional "air" feature. */
	.air_input_count = 2,
	
	/* One 48V phantom power switch */
	.power_48v_count = 1,

	/* Gen 3 devices have an MSD mode */
	.has_msd_mode = 1,

	.line_out_descrs = {
		"Headphones 1 L",
		"Headphones 1 R",
		"Headphones 2 L",
		"Headphones 2 R",
	},

	.port_names = s8i6_gen3_port_names,

	.mux_size = { 77, 77, 77, 73, 46 },

	.ports = {
		[SCARLETT2_PORT_TYPE_ANALOGUE] = {
			.id = SCARLETT2_PORT_ID_ANALOGUE,
			.num = { 6, 4, 4, 4, 4 },
			.src_descr = "Analogue In %02d",
			.src_num_offset = 1,
			.dst_descr = "Analogue Out %02d"
		},
		[SCARLETT2_PORT_TYPE_SPDIF] = {
			.id = SCARLETT2_PORT_ID_SPDIF,
			.num = { 2, 2, 2, 2, 2 },
			.src_descr = "S/PDIF In %d",
			.src_num_offset = 1,
			.dst_descr = "S/PDIF Out %d"
		},
		[SCARLETT2_PORT_TYPE_MIX] = {
			.id = SCARLETT2_PORT_ID_MIX,
			.num = { 8, 8, 8, 8, 8 },
			.src_descr = "Mix %c Out",
			.src_num_offset = 'A',
			.dst_descr = "Mix In %02d"
		},
		[SCARLETT2_PORT_TYPE_PCM] = {
			.id = SCARLETT2_PORT_ID_PCM,
			.num = { 6, 10, 10, 10, 10 },
			.src_descr = "PCM In %02d",
			.src_num_offset = 1,
			.dst_descr = "PCM Out %02d"
		},
	},
};

static const struct scarlett2_port_name s18i8_gen3_port_names[] = {
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 0, "Monitor 1 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 1, "Monitor 1 R" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 2, "Headphones 1 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 3, "Headphones 1 R" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 4, "Headphones 2 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 5, "Headphones 2 R" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 6, "Monitor 2 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 7, "Monitor 2 R" },
	{ -1, -1, -1, NULL }
};

static const struct scarlett2_device_info s18i8_gen3_info = {
	.usb_id = USB_ID(0x1235, 0x8214),

	/* The analogue line outputs on the 18i8 can be switched
	 * between software and hardware volume control
	 */
	.line_out_hw_vol = 1,

	/* Virtual mute and dim buttons */
	.button_count = 2,

	/* The first two analogue inputs can be switched between line
	 * and instrument levels.
	 */
	.level_input_count = 2,

	/* The first four analogue inputs have an optional pad. */
	.pad_input_count = 4,

	/* The first four analogue inputs have an optional "air" feature. */
	.air_input_count = 4,

	/* Two 48V phantom power switches */
	.power_48v_count = 2,

	/* Gen 3 devices have an MSD mode */
	.has_msd_mode = 1,

	/* Has a main/alt speaker switch */
	.has_speaker_switching = 1,

	.line_out_descrs = {
		"Monitor 1 L",
		"Monitor 1 R",
		"Headphones 1 L",
		"Headphones 1 R",
		"Headphones 2 L",
		"Headphones 2 R",
		"Monitor 2 L",
		"Monitor 2 R",
	},

	.port_names = s18i8_gen3_port_names,

	.mux_size = { 77, 77, 77, 73, 46 },

	.ports = {
		[SCARLETT2_PORT_TYPE_ANALOGUE] = {
			.id = SCARLETT2_PORT_ID_ANALOGUE,
			.num = { 8, 8, 8, 8, 8 },
			.src_descr = "Analogue In %02d",
			.src_num_offset = 1,
			.dst_descr = "Analogue Out %02d"
		},
		[SCARLETT2_PORT_TYPE_SPDIF] = {
			.id = SCARLETT2_PORT_ID_SPDIF,
			.num = { 2, 2, 2, 2, 2 },
			.src_descr = "S/PDIF In %d",
			.src_num_offset = 1,
			.dst_descr = "S/PDIF Out %d"
		},
		[SCARLETT2_PORT_TYPE_ADAT] = {
			.id = SCARLETT2_PORT_ID_ADAT,
			.num = { 8, 0, 0, 0, 0 },
			.src_descr = "ADAT In %d",
			.src_num_offset = 1,
			.dst_descr = "ADAT Out %d"
		},
		[SCARLETT2_PORT_TYPE_MIX] = {
			.id = SCARLETT2_PORT_ID_MIX,
			.num = { 10, 20, 20, 20, 20 },
			.src_descr = "Mix %c Out",
			.src_num_offset = 'A',
			.dst_descr = "Mix In %02d"
		},
		[SCARLETT2_PORT_TYPE_PCM] = {
			.id = SCARLETT2_PORT_ID_PCM,
			.num = { 8, 20, 20, 16, 10 },
			.src_descr = "PCM In %02d",
			.src_num_offset = 1,
			.dst_descr = "PCM Out %02d"
		},

	},
};

static const struct scarlett2_port_name s18i20_gen3_port_names[] = {
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 0, "Monitor 1 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 1, "Monitor 1 R" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 2, "Monitor 2 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 3, "Monitor 2 R" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 6, "Headphones 1 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 7, "Headphones 1 R" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 8, "Headphones 2 L" },
	{ SCARLETT2_PORT_OUT, SCARLETT2_PORT_TYPE_ANALOGUE, 9, "Headphones 2 R" },

	{ -1, -1, -1, NULL }
};

static const struct scarlett2_device_info s18i20_gen3_info = {
	.usb_id = USB_ID(0x1235, 0x8215),

	/* The analogue line outputs on the 18i20 can be switched
	 * between software and hardware volume control
	 */
	.line_out_hw_vol = 1,

	/* Mute and dim buttons */
	.button_count = 2,

	/* The first two analogue inputs can be switched between line
	 * and instrument levels.
	 */
	.level_input_count = 2,

	/* The first eight analogue inputs have an optional pad. */
	.pad_input_count = 8,

	/* The first eight analogue inputs have an optional "air" feature. */
	.air_input_count = 8,
	
	/* Two 48V phantom power switches */
	.power_48v_count = 2,

	/* Gen 3 devices have an MSD mode */
	.has_msd_mode = 1,

	/* Has a main/alt speaker switch */
	.has_speaker_switching = 1,
	
	/* Has a talkback speaker switch */
	.has_talkback = 1,

	.line_out_descrs = {
		"Monitor 1 L",
		"Monitor 1 R",
		"Monitor 2 L",
		"Monitor 2 R",
		NULL,
		NULL,
		"Headphones 1 L",
		"Headphones 1 R",
		"Headphones 2 L",
		"Headphones 2 R",
	},

	.port_names = s18i20_gen3_port_names,

	.mux_size = { 77, 77, 77, 73, 46 },

	.ports = {
		[SCARLETT2_PORT_TYPE_ANALOGUE] = {
			.id = SCARLETT2_PORT_ID_ANALOGUE,
			.num = { 8, 10, 10, 10, 10 },
			.src_descr = "Analogue In %02d",
			.src_num_offset = 1,
			.dst_descr = "Analogue Out %02d"
		},
		[SCARLETT2_PORT_TYPE_SPDIF] = {
			.id = SCARLETT2_PORT_ID_SPDIF,
			.num = { 2, 2, 2, 2, 2 },
			.src_descr = "S/PDIF In %d",
			.src_num_offset = 1,
			.dst_descr = "S/PDIF Out %d"
		},
		[SCARLETT2_PORT_TYPE_ADAT] = {
			.id = SCARLETT2_PORT_ID_ADAT,
			.num = { 8, 8, 8, 8, 0 },
			.src_descr = "ADAT In %d",
			.src_num_offset = 1,
			.dst_descr = "ADAT Out %d"
		},
		[SCARLETT2_PORT_TYPE_MIX] = {
			.id = SCARLETT2_PORT_ID_MIX,
			.num = { 12, 24, 24, 24, 24 },
			.src_descr = "Mix %c Out",
			.src_num_offset = 'A',
			.dst_descr = "Mix In %02d"
		},
		[SCARLETT2_PORT_TYPE_PCM] = {
			.id = SCARLETT2_PORT_ID_PCM,
			.num = { 20, 20, 20, 18, 10 },
			.src_descr = "PCM In %02d",
			.src_num_offset = 1,
			.dst_descr = "PCM Out %02d"
		},
		[SCARLETT2_PORT_TYPE_INT_MIC] = {
			.id = SCARLETT2_PORT_ID_ANALOGUE + 8,
			.num = { 1, 0, 0, 0, 0 },
			.src_descr = "Internal Mic"
		},
		[SCARLETT2_PORT_TYPE_TALKBACK] = {
			.id = SCARLETT2_PORT_ID_MIX + 24,
			.num = { 0, 1, 1, 1, 1 },
			.dst_descr = "Talkback"
		},
	},
};

static const struct scarlett2_device_info *scarlett2_supported_devices[] = {
	/* Supported Gen2 devices */
	&s6i6_gen2_info,
	&s18i8_gen2_info,
	&s18i20_gen2_info,

	/* Supported Gen3 devices */
	&s4i4_gen3_info,
	&s8i6_gen3_info,
	&s18i8_gen3_info,
	&s18i20_gen3_info,

	/* End of list */
	NULL
};

/*** USB Interactions ***/

/* Interrupt flags for volume, mute/dim button, and sync changes */
#define SCARLETT2_USB_INTERRUPT_ACK 0x00000001
#define SCARLETT2_USB_INTERRUPT_SYNC_CHANGE 0x00000008
#define SCARLETT2_USB_INTERRUPT_BUTTON_CHANGE 0x00200000
#define SCARLETT2_USB_INTERRUPT_VOL_CHANGE 0x00400000
#define SCARLETT2_USB_INTERRUPT_LINE_CTL_CHANGE 0x00800000
#define SCARLETT2_USB_INTERRUPT_SPEAKER_CHANGE 0x01000000

/* Commands for sending/receiving requests/responses */
#define SCARLETT2_USB_CMD_INIT 0
#define SCARLETT2_USB_CMD_REQ 2
#define SCARLETT2_USB_CMD_RESP 3

#define SCARLETT2_USB_INIT_1                     0x00000000
#define SCARLETT2_USB_INIT_2                     0x00000002
#define SCARLETT2_USB_CONFIG_SAVE                0x00000006
#define SCARLETT2_USB_GET_METER_LEVELS           0x00001001
#define SCARLETT2_USB_SET_MIX                    0x00002002
#define SCARLETT2_USB_GET_MUX                    0x00003001
#define SCARLETT2_USB_SET_MUX                    0x00003002
#define SCARLETT2_USB_GET_DATA                   0x00800000
#define SCARLETT2_USB_SET_DATA                   0x00800001
#define SCARLETT2_USB_DATA_CMD                   0x00800002

#define SCARLETT2_USB_VOLUME_STATUS_OFFSET       0x31
#define SCARLETT2_VOLUMES_BASE                   0x34
#define SCARLETT2_SW_CONFIG_BASE                 0xec

#define SCARLETT2_SW_CONFIG_PACKET_SIZE          1024     /* The maximum packet size used to transfer data */
#define SCARLETT2_SW_CONFIG_MIXER_INPUTS         30       /* 30 inputs per one mixer in config */

#define SCARLETT2_SW_CONFIG_SIZE_OFFSET          0x08     /* 0xf4   - 0xec */
#define SCARLETT2_SW_CONFIG_STEREO_BITS_OFFSET   0x0c8    /* 0x1b4  - 0xec */
#define SCARLETT2_SW_CONFIG_VOLUMES_OFFSET       0x0d0    /* 0x1bc  - 0xec */
#define SCARLETT2_SW_CONFIG_MIXER_OFFSET         0xf04    /* 0xff0  - 0xec */

#define SCARLETT2_USB_METER_LEVELS_GET_MAGIC 1

/* volume status is read together (matches scarlett2_config_items[]) */
struct scarlett2_usb_volume_status {
	/* mute & dim buttons */
	u8 buttons[SCARLETT2_BUTTON_MAX]; /* 0x31 */

	u8 pad1; /* 0x33 */

	/* software volume setting */
	s16 sw_vol[SCARLETT2_ANALOGUE_MAX]; /* 0x34 */

	/* actual volume of output inc. dim (-18dB) */
	s16 hw_vol[SCARLETT2_ANALOGUE_MAX]; /* 0x48 */

        /* mute? */
	u8 mute[SCARLETT2_ANALOGUE_MAX]; /* 0x5C */

	/* sw (0) or hw (1) controlled */
	u8 sw_hw_switch[SCARLETT2_ANALOGUE_MAX]; /* 0x66 */

	u8 pad3[6]; /* 0x70 */

	/* front panel volume knob */
	s16 master_vol; /* 0x76 */
} __packed;

/* Configuration parameters that can be read and written */
enum {
	SCARLETT2_CONFIG_BUTTONS = 0,
	SCARLETT2_CONFIG_LINE_OUT_VOLUME = 1,
	SCARLETT2_CONFIG_SW_HW_SWITCH = 2,
	SCARLETT2_CONFIG_LEVEL_SWITCH = 3,
	SCARLETT2_CONFIG_PAD_SWITCH = 4,
	SCARLETT2_CONFIG_AIR_SWITCH = 5,
	SCARLETT2_CONFIG_SPDIF_SWITCH = 6,
	SCARLETT2_CONFIG_48V_SWITCH = 7,
	SCARLETT2_CONFIG_MSD_SWITCH = 8,
	SCARLETT2_CONFIG_MAIN_ALT_SPEAKER_SWITCH = 9,
	SCARLETT2_CONFIG_SPEAKER_SWITCHING_SWITCH = 10,
	SCARLETT2_CONFIG_GAIN_HALO_1 = 11,
	SCARLETT2_CONFIG_GAIN_HALO_2 = 12,
	SCARLETT2_CONFIG_MIX_TALKBACK = 13,
	SCARLETT2_CONFIG_COUNT = 14
};

/* Location, size, and activation command number for the configuration
 * parameters
 */
struct scarlett2_config {
	u8 offset;
	u8 size;
	u8 activate;
};

static const struct scarlett2_config
		scarlett2_config_items[SCARLETT2_CONFIG_COUNT] = {
	/* Mute/Dim Buttons */
	{
		.offset = 0x31,
		.size = 1,
		.activate = 2
	},

	/* Line Out Volume */
	{
		.offset = 0x34,
		.size = 2,
		.activate = 1
	},

	/* SW/HW Volume Switch */
	{
		.offset = 0x66,
		.size = 1,
		.activate = 3
	},

	/* Inst Switch */
	{
		.offset = 0x7c,
		.size = 1,
		.activate = 7
	},

	/* Pad Switch */
	{
		.offset = 0x84,
		.size = 1,
		.activate = 8
	},

	/* Air Switch */
	{
		.offset = 0x8c,
		.size = 1,
		.activate = 8
	},

	/* S/PDIF Source */
	{
		.offset = 0x94,
		.size = 1,
		.activate = 6
	},

	/* Phantom (48V) power */
	{
		.offset = 0x9c,
		.size = 1,
		.activate = 8
	},

	/* MSD Mode */
	{
		.offset = 0x9d,
		.size = 1,
		.activate = 6
	},

	/* Alternate Speaker and Talkback switches */
	{
		.offset = 0x9f,
		.size = 1,
		.activate = 10
	},

	/* Speaker Switching */
	{
		.offset = 0xa0,
		.size = 1,
		.activate = 10
	},

	/* Gain Halos */
	{
		.offset = 0xa1,
		.size = 1,
		.activate = 9
	},

	/* Gain Halo? */
	{
		.offset = 0xa8,
		.size = 1,
		.activate = 11
	},
	
	/* Talkback enable flags for each output of internal mixer */
	{
		.offset = 0xb0,
		.size = 2,
		.activate = 10
	}
};

/* proprietary request/response format */
struct scarlett2_usb_packet {
	__le32 cmd;
	__le16 size;
	__le16 seq;
	__le32 error;
	__le32 pad;
	u8 data[];
};

/* Decode floating-point value into valid scarlett gain
 * The input floating-point value may be any (including Infs and NaNs)
 * The output integer value is in range of -160 to 12 (dB with 0.5 step)
 */
static int scarlett2_float_to_mixer_level(u32 v)
{
	u32 exp, frac;
	int sign, res;

	exp  = (v >> 23) & 0xff;
	if (exp < 0x7e) /* abs(v) < 0.5f */
		return 0;

	sign = (v >> 31);
	if (exp > 0x85) /* abs(v) > 80.0f ? */
		return (sign) ? -160 : 12;

	/* Compute the fraction part */
	frac = (v & 0x007fffffu) | 0x00800000u; /* 24 bits normalized */
	frac = frac >> (0x95 - exp); /* 0x7f - exp + 22 */
	res = (sign) ? -frac : frac;

	/* Limit the value and return */
	if (res < -160)
		return -160;
	return (res < 12) ? res : 12;
}

/* Convert a port number index (per info->ports) to a hardware ID */
static u32 scarlett2_id_to_mux(const struct scarlett2_ports *ports, int direction, int num)
{
	int port_type;

	if ((direction < 0) || (direction >= SCARLETT2_PORT_DIRECTIONS) || (num < 0))
		return 0;

	for (port_type = 0; port_type < SCARLETT2_PORT_TYPE_COUNT; ++port_type) {
		if (num < ports[port_type].num[direction])
			return ports[port_type].id + num;
		num -= ports[port_type].num[direction];
	}

	return 0; /* Could not encode port */
}

static int scarlett2_count_ports(const struct scarlett2_ports *ports, int direction) {
	int port_type, count = 0;

	for (port_type=0; port_type < SCARLETT2_PORT_TYPE_COUNT; ++port_type)
		count += ports[port_type].num[direction];

	return count;
}

/* Convert a hardware ID to port number index (per info->ports) */
static int scarlett2_mux_to_id(const struct scarlett2_ports *ports, int direction, u32 mux_id)
{
	int port_type, port_id, port_num, port_base;

	if ((direction < 0) || (direction >= SCARLETT2_PORT_DIRECTIONS))
		return -1; /* Could not decode port */

	port_id  = mux_id & SCARLETT2_PORT_ID_MASK;
	if (port_id == SCARLETT2_PORT_ID_NONE)
		return -1;
	
	port_num = mux_id & SCARLETT2_PORT_NUM_MASK;
	port_base= 0;

	for (port_type=0; port_type < SCARLETT2_PORT_TYPE_COUNT; ++port_type) {
		/* Port identifier matches? */
		if (port_id == (ports[port_type].id & SCARLETT2_PORT_ID_MASK)) {
			if (port_num < ports[port_type].num[direction])
				return port_base + port_num;
			port_num -= ports[port_type].num[direction];
		}

		port_base += ports[port_type].num[direction];
	}

	return -1; /* Could not decode port */
}

/* Format port number to the proper port name */
static char *scarlett2_fmt_port_name(char *out, int len, const struct scarlett2_device_info *info, int direction, int num)
{
	int port_type;
	const char *fmt;
	char name[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];
	const struct scarlett2_ports *ports = info->ports;
	const struct scarlett2_port_name *port_name;

	strncpy(out, "Off", len);
	if ((direction < 0) || (direction >= SCARLETT2_PORT_DIRECTIONS) || (num < 0))
		return out;

	for (port_type = 0; port_type < SCARLETT2_PORT_TYPE_COUNT; ++port_type) {
		/* The number does match the range ? */
		if (num < ports[port_type].num[direction]) {
			/* Is there 'special' name found ? */
			name[0] = '\0';
			for (port_name = info->port_names; port_name->name != NULL; ++port_name) {
				if ((port_name->direction == direction) &&
				    (port_name->type == port_type) &&
				    (port_name->index == num)) {
					snprintf(name, SNDRV_CTL_ELEM_ID_NAME_MAXLEN, " (%s)", port_name->name);
					break;
				}
			}

			/* Format the physical port name */
			fmt  = (direction == SCARLETT2_PORT_IN) ? ports[port_type].src_descr : ports[port_type].dst_descr;
			num += (direction == SCARLETT2_PORT_IN) ? ports[port_type].src_num_offset : 1;
			snprintf(out, len, fmt, num);
			strncat(out, name, len);
			return out;
		}

		num -= ports[port_type].num[direction];
	}

	return out;
}

/* get the starting port index number for a given port type/direction */
static int scarlett2_get_port_num(const struct scarlett2_ports *ports, int direction, int type, int num)
{
	int i;
	for (i = 0; i < type; i++)
		num += ports[i].num[direction];

	return num;
}

static void scarlett2_fill_request_header(struct scarlett2_mixer_data *private,
					  struct scarlett2_usb_packet *req,
					  u32 cmd, u16 req_size)
{
	/* sequence must go up by 1 for each request */
	u16 seq = private->scarlett2_seq++;

	req->cmd = cpu_to_le32(cmd);
	req->size = cpu_to_le16(req_size);
	req->seq = cpu_to_le16(seq);
	req->error = 0;
	req->pad = 0;
}

static int scarlett2_usb_tx(struct usb_device *dev, int interface,
			    void *buf, u16 size)
{
	return snd_usb_ctl_msg(dev, usb_sndctrlpipe(dev, 0),
			SCARLETT2_USB_CMD_REQ,
			USB_RECIP_INTERFACE | USB_TYPE_CLASS | USB_DIR_OUT,
			0, interface, buf, size);
}

static int scarlett2_usb_rx(struct usb_device *dev, int interface,
			    u32 usb_req, void *buf, u16 size)
{
	return snd_usb_ctl_msg(dev, usb_sndctrlpipe(dev, 0),
			usb_req,
			USB_RECIP_INTERFACE | USB_TYPE_CLASS | USB_DIR_IN,
			0, interface, buf, size);
}

static int scarlett2_usb(
	struct usb_mixer_interface *mixer, u32 cmd,
	void *req_data, u16 req_size, void *resp_data, u16 resp_size);

static int scarlett2_commit_software_config(
	struct usb_mixer_interface *mixer,
	void *ptr, /* the pointer of the first changed byte in the configuration */
	int bytes /* the actual number of bytes changed in the configuration */
);

/* Cargo cult proprietary initialisation sequence */
static int scarlett2_usb_init(struct usb_mixer_interface *mixer)
{
	struct snd_usb_audio *chip = mixer->chip;
	struct usb_device *dev = chip->dev;
	struct scarlett2_mixer_data *private = mixer->private_data;
	u16 buf_size = sizeof(struct scarlett2_usb_packet) + 8;
	struct scarlett2_usb_packet *buf;
	int err;

	if (snd_usb_pipe_sanity_check(dev, usb_sndctrlpipe(dev, 0))) {
		return -EINVAL;
	}

	buf = kmalloc(buf_size, GFP_KERNEL);
	if (!buf) {
		err = -ENOMEM;
		goto error;
	}

	// step 0
	err = scarlett2_usb_rx(dev, private->interface, SCARLETT2_USB_CMD_INIT,
			       buf, buf_size);
	if (err < 0)
		goto error;

	// step 1
	private->scarlett2_seq = 1;
	err = scarlett2_usb(mixer, SCARLETT2_USB_INIT_1, NULL, 0, NULL, 0);
	if (err < 0)
		goto error;

	// step 2
	private->scarlett2_seq = 1;
	err = scarlett2_usb(mixer, SCARLETT2_USB_INIT_2, NULL, 0, NULL, 84);
	if (err < 0)
		goto error;

	err = 0;

error:
	kfree(buf);
	return err;
}

/* Send a proprietary format request to the Scarlett interface */
static int scarlett2_usb(
	struct usb_mixer_interface *mixer, u32 cmd,
	void *req_data, u16 req_size, void *resp_data, u16 resp_size)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	struct usb_device *dev = mixer->chip->dev;
	u16 req_buf_size = sizeof(struct scarlett2_usb_packet) + req_size;
	u16 resp_buf_size = sizeof(struct scarlett2_usb_packet) + resp_size;
	struct scarlett2_usb_packet *req, *resp = NULL;
	int err;

	req = kmalloc(req_buf_size, GFP_KERNEL);
	if (!req) {
		err = -ENOMEM;
		goto error;
	}

	resp = kmalloc(resp_buf_size, GFP_KERNEL);
	if (!resp) {
		err = -ENOMEM;
		goto error;
	}

	mutex_lock(&private->usb_mutex);

	/* build request message and send it */

	scarlett2_fill_request_header(private, req, cmd, req_size);

	if (req_size)
		memcpy(req->data, req_data, req_size);

	err = scarlett2_usb_tx(dev, private->interface, req, req_buf_size);

	if (err != req_buf_size) {
		usb_audio_err(
			mixer->chip,
			"Scarlett Gen 2 USB request result cmd %x was %d\n",
			cmd, err);
		err = -EINVAL;
		goto unlock;
	}

	/* send a second message to get the response */

	err = scarlett2_usb_rx(dev, private->interface, SCARLETT2_USB_CMD_RESP,
			       resp, resp_buf_size);

	/* validate the response */

	if (err != resp_buf_size) {
		usb_audio_err(
			mixer->chip,
			"Scarlett Gen 2 USB response result cmd %x was %d expected %d\n",
			cmd, err, resp_buf_size);
		err = -EINVAL;
		goto unlock;
	}

	/* cmd/seq/size should match except when initialising
	 * seq sent = 1, response = 0
	 */
	if (resp->cmd != req->cmd ||
	    (resp->seq != req->seq && (req->seq != 1 || resp->seq != 0)) ||
	    resp_size != le16_to_cpu(resp->size) ||
	    resp->error ||
	    resp->pad) {
		usb_audio_err(
			mixer->chip,
			"Scarlett Gen 2 USB invalid response; "
			   "cmd tx/rx %d/%d seq %d/%d size %d/%d "
			   "error %d pad %d\n",
			le32_to_cpu(req->cmd), le32_to_cpu(resp->cmd),
			le16_to_cpu(req->seq), le16_to_cpu(resp->seq),
			resp_size, le16_to_cpu(resp->size),
			le32_to_cpu(resp->error),
			le32_to_cpu(resp->pad));
		err = -EINVAL;
		goto unlock;
	}

	if (resp_data && resp_size > 0)
		memcpy(resp_data, resp->data, resp_size);

unlock:
	mutex_unlock(&private->usb_mutex);
error:
	kfree(req);
	kfree(resp);
	return err;
}

/* Send SCARLETT2_USB_DATA_CMD SCARLETT2_USB_CONFIG_SAVE */
static void scarlett2_config_save(struct usb_mixer_interface *mixer)
{
	__le32 req = cpu_to_le32(SCARLETT2_USB_CONFIG_SAVE);

	scarlett2_usb(mixer, SCARLETT2_USB_DATA_CMD,
		      &req, sizeof(u32),
		      NULL, 0);
}

/* Delayed work to save config */
static void scarlett2_config_save_work(struct work_struct *work)
{
	struct scarlett2_mixer_data *private =
		container_of(work, struct scarlett2_mixer_data, work.work);

	scarlett2_config_save(private->mixer);
}

/* Send a USB message to set a configuration parameter (volume level,
 * sw/hw volume switch, line/inst level switch, pad, or air switch)
 */
static int scarlett2_usb_set_config(
	struct usb_mixer_interface *mixer,
	int config_item_num, int index, int value)
{
	const struct scarlett2_config config_item =
	       scarlett2_config_items[config_item_num];
	struct {
		__le32 offset;
		__le32 bytes;
		__le32 value;
	} __packed req;
	__le32 req2;
	int err;
	struct scarlett2_mixer_data *private = mixer->private_data;

	/* Cancel any pending NVRAM save */
	cancel_delayed_work_sync(&private->work);

	/* Send the configuration parameter data */
	req.offset = cpu_to_le32(config_item.offset + index * config_item.size);
	req.bytes = cpu_to_le32(config_item.size);
	req.value = cpu_to_le32(value);
	err = scarlett2_usb(mixer, SCARLETT2_USB_SET_DATA,
			    &req, sizeof(u32) * 2 + config_item.size,
			    NULL, 0);
	if (err < 0)
		return err;

	/* Activate the change */
	req2 = cpu_to_le32(config_item.activate);
	err = scarlett2_usb(mixer, SCARLETT2_USB_DATA_CMD,
			    &req2, sizeof(req2), NULL, 0);
	if (err < 0)
		return err;

	/* Schedule the change to be written to NVRAM */
	schedule_delayed_work(&private->work, msecs_to_jiffies(2000));

	return 0;
}

/* Send a USB message to get data; result placed in *buf */
static int scarlett2_usb_get(
	struct usb_mixer_interface *mixer,
	int offset, void *buf, int size)
{
	struct {
		__le32 offset;
		__le32 size;
	} __packed req;

	req.offset = cpu_to_le32(offset);
	req.size = cpu_to_le32(size);
	return scarlett2_usb(mixer, SCARLETT2_USB_GET_DATA,
			     &req, sizeof(req), buf, size);
}

/* Send a USB message to get configuration parameters; result placed in *buf */
static int scarlett2_usb_get_config(
	struct usb_mixer_interface *mixer,
	int config_item_num, int count, void *buf)
{
	const struct scarlett2_config config_item =
	       scarlett2_config_items[config_item_num];
	int size = config_item.size * count;

	return scarlett2_usb_get(mixer, config_item.offset, buf, size);
}

/* Send a USB message to get volume status; result placed in *buf */
static int scarlett2_usb_get_volume_status(
	struct usb_mixer_interface *mixer,
	struct scarlett2_usb_volume_status *buf)
{
	return scarlett2_usb_get(mixer, SCARLETT2_USB_VOLUME_STATUS_OFFSET,
				 buf, sizeof(*buf));
}

/* Send a USB message to set the volumes for all inputs of one mix
 * (values obtained from private->mix[])
 */
static int scarlett2_usb_set_mix(struct usb_mixer_interface *mixer,
				     int mix_num)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;

	struct {
		__le16 mix_num;
		__le16 data[SCARLETT2_INPUT_MIX_MAX];
	} __packed req;

	int i, j;
	int num_mixer_in =
		info->ports[SCARLETT2_PORT_TYPE_MIX].num[SCARLETT2_PORT_OUT];

	req.mix_num = cpu_to_le16(mix_num);

	for (i = 0, j = mix_num * num_mixer_in; i < num_mixer_in; i++, j++)
		req.data[i] = cpu_to_le16(
			scarlett2_mixer_values[private->mix[j]]
		);

	return scarlett2_usb(mixer, SCARLETT2_USB_SET_MIX,
			     &req, (num_mixer_in + 1) * sizeof(u16),
			     NULL, 0);
}

/* Send USB messages to get mux inputs */
static int scarlett2_usb_get_mux(struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const struct scarlett2_ports *ports = info->ports;
	int mux_size, i, src_port, dst_port, err;
	u32 mux_id;
	struct {
		__le16 num;
		__le16 count;
	} __packed req;

	__le32 data[SCARLETT2_MUX_MAX];

	mux_size  = info->mux_size[SCARLETT2_PORT_OUT];

	/* Send the request */
	req.num   = 0;
	req.count = cpu_to_le16(mux_size);
	err = scarlett2_usb(mixer, SCARLETT2_USB_GET_MUX, &req, 2 * sizeof(__le16), data, sizeof(__le32) * mux_size);
	if (err < 0)
		return err;

	/* Clear mux state */
	for (i=0 ; i < SCARLETT2_MUX_MAX; ++i)
		private->mux[i] = 0;

	for (i=0; i<mux_size; ++i) {
		/* Decode input and output port indexes, remember mux state for the port */
		mux_id = le32_to_cpu(data[i]);
		src_port = scarlett2_mux_to_id(ports, SCARLETT2_PORT_IN, mux_id >> 12);
		dst_port = scarlett2_mux_to_id(ports, SCARLETT2_PORT_OUT, mux_id);
		if ((dst_port >= 0) && (dst_port < SCARLETT2_MUX_MAX))
			private->mux[dst_port] = src_port;
	}

	return err;
}

/* Send USB messages to set mux inputs */
static int scarlett2_usb_set_mux(struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const struct scarlett2_ports *ports = info->ports;
	int direction, conn_id, order, port, port_idx, err = 0;
	u32 src_mux, dst_mux;

	static const int assignment_order[] = {
		SCARLETT2_PORT_TYPE_PCM,
		SCARLETT2_PORT_TYPE_ANALOGUE,
		SCARLETT2_PORT_TYPE_SPDIF,
		SCARLETT2_PORT_TYPE_ADAT,
		SCARLETT2_PORT_TYPE_MIX,
		SCARLETT2_PORT_TYPE_TALKBACK
	};

	struct {
		__le16 pad;
		__le16 num;
		__le32 data[SCARLETT2_MUX_MAX];
	} __packed req;

	/* mux settings for each rate */
	for (direction = SCARLETT2_PORT_OUT_44; direction <= SCARLETT2_PORT_OUT_176; ++direction) {
		/* init request */
		req.pad = 0;
		req.num = cpu_to_le16(direction - SCARLETT2_PORT_OUT_44);

		/* form the request data */
		conn_id = 0;
		for (order = 0; order < sizeof(assignment_order)/sizeof(int); ++order) {
			int port_type = assignment_order[order];

			for (port = 0; port < ports[port_type].num[direction]; ++port) {
				port_idx = scarlett2_get_port_num(ports, SCARLETT2_PORT_OUT, port_type, port);

				/* Lower 12 bits for destination, next 12 bits for source */
				src_mux = scarlett2_id_to_mux(ports, SCARLETT2_PORT_IN, private->mux[port_idx]);
				dst_mux = scarlett2_id_to_mux(ports, SCARLETT2_PORT_OUT, port_idx);
				req.data[conn_id++] = cpu_to_le32((src_mux << 12) | dst_mux);
			}
		}

		/* Fill rest mux data with zeros */
		for ( ; conn_id < info->mux_size[direction]; ++conn_id)
			req.data[conn_id] = 0;

		/* Send the SET_MUX notification */
		err = scarlett2_usb(mixer, SCARLETT2_USB_SET_MUX,
				    &req, 2 * sizeof(__le16) + conn_id * sizeof(__le32),
				    NULL, 0);
		if (err < 0)
			return err;
	}

	return err;
}

/* Send USB message to get meter levels */
static int scarlett2_usb_get_meter_levels(struct usb_mixer_interface *mixer,
					  u16 *levels)
{
	struct {
		__le16 pad;
		__le16 num_meters;
		__le32 magic;
	} __packed req;
	u32 resp[SCARLETT2_NUM_METERS];
	int i, err;

	req.pad = 0;
	req.num_meters = cpu_to_le16(SCARLETT2_NUM_METERS);
	req.magic = cpu_to_le32(SCARLETT2_USB_METER_LEVELS_GET_MAGIC);
	err = scarlett2_usb(mixer, SCARLETT2_USB_GET_METER_LEVELS,
			    &req, sizeof(req), resp, sizeof(resp));
	if (err < 0)
		return err;

	/* copy, convert to u16 */
	for (i = 0; i < SCARLETT2_NUM_METERS; i++)
		levels[i] = resp[i];

	return 0;
}

/*** Control Functions ***/

/* helper function to create a new control */
static int scarlett2_add_new_ctl(struct usb_mixer_interface *mixer,
				 const struct snd_kcontrol_new *ncontrol,
				 int index, int channels, const char *name,
				 struct snd_kcontrol **kctl_return)
{
	struct snd_kcontrol *kctl;
	struct usb_mixer_elem_info *elem;
	int err;

	elem = kzalloc(sizeof(*elem), GFP_KERNEL);
	if (!elem)
		return -ENOMEM;

	elem->head.mixer = mixer;
	elem->control = index;
	elem->head.id = index;
	elem->channels = channels;

	kctl = snd_ctl_new1(ncontrol, elem);
	if (!kctl) {
		kfree(elem);
		return -ENOMEM;
	}
	kctl->private_free = snd_usb_mixer_elem_free;

	strlcpy(kctl->id.name, name, sizeof(kctl->id.name));

	err = snd_usb_mixer_add_control(&elem->head, kctl);
	if (err < 0)
		return err;

	if (kctl_return)
		*kctl_return = kctl;

	return 0;
}

/*** Analogue Line Out Volume Controls ***/

/* Update hardware volume controls after receiving notification that
 * they have changed
 */
static int scarlett2_update_volumes(struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_ports *ports = private->info->ports;
	struct scarlett2_usb_volume_status volume_status;
	int num_line_out =
		ports[SCARLETT2_PORT_TYPE_ANALOGUE].num[SCARLETT2_PORT_OUT];
	int err, i;
	s16 volume;

	private->vol_updated = 0;

	err = scarlett2_usb_get_volume_status(mixer, &volume_status);
	if (err < 0)
		return err;
	
	private->master_vol = clamp(
		volume_status.master_vol + SCARLETT2_VOLUME_BIAS,
		0, SCARLETT2_VOLUME_BIAS);
	
	for (i = 0; i < num_line_out; i++) {
		/* If volume is software-controlled, try to read it's value from software configuration */
		if ((private->sw_cfg_volume) &&
		    (!private->vol_sw_hw_switch[i])) {
			volume = le16_to_cpu(private->sw_cfg_volume[i].volume);
			private->vol[i] = clamp(volume + SCARLETT2_VOLUME_BIAS, 0, SCARLETT2_VOLUME_BIAS); /* Reset to sw-configured volume */
		}
		else
			private->vol[i] = private->master_vol; /* Reset to master volume */
	}

	for (i = 0; i < private->info->button_count; i++)
		private->buttons[i] = !!volume_status.buttons[i];

	return 0;
}

static int scarlett2_volume_ctl_info(struct snd_kcontrol *kctl,
				     struct snd_ctl_elem_info *uinfo)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = elem->channels;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = SCARLETT2_VOLUME_BIAS;
	uinfo->value.integer.step = 1;
	return 0;
}

static int scarlett2_master_volume_ctl_get(struct snd_kcontrol *kctl,
					   struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;

	if (private->vol_updated) {
		mutex_lock(&private->data_mutex);
		scarlett2_update_volumes(mixer);
		mutex_unlock(&private->data_mutex);
	}

	ucontrol->value.integer.value[0] = private->master_vol;
	return 0;
}

static int scarlett2_volume_ctl_get(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;
	int index = elem->control;

	if (private->vol_updated) {
		mutex_lock(&private->data_mutex);
		scarlett2_update_volumes(mixer);
		mutex_unlock(&private->data_mutex);
	}

	ucontrol->value.integer.value[0] = private->vol[index];
	return 0;
}

static int scarlett2_volume_ctl_put(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;
	int index = elem->control;
	int oval, val, err = 0;
	u16 volume;
	
	mutex_lock(&private->data_mutex);

	oval = private->vol[index];
	val = ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->vol[index] = val;

	/* Update volume for the output */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_LINE_OUT_VOLUME,
				       index, val - SCARLETT2_VOLUME_BIAS);
	if (err != 0)
		goto unlock;

	/* Update software configuration if possible */
	if ((private->sw_cfg_volume) &&
	    (!private->vol_sw_hw_switch[index])) {
		volume = val - SCARLETT2_VOLUME_BIAS;
		private->sw_cfg_volume[index].volume  = cpu_to_le16(volume);
		private->sw_cfg_volume[index].changed = 1;
		err = scarlett2_commit_software_config(mixer, &private->sw_cfg_volume[index], sizeof(struct scarlett2_sw_cfg_volume));
		if (err < 0)
			goto unlock;
	}
	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const DECLARE_TLV_DB_MINMAX(
	db_scale_scarlett2_gain, -SCARLETT2_VOLUME_BIAS * 100, 0
);

static const struct snd_kcontrol_new scarlett2_master_volume_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READ |
		  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
	.name = "",
	.info = scarlett2_volume_ctl_info,
	.get  = scarlett2_master_volume_ctl_get,
	.private_value = 0, /* max value */
	.tlv = { .p = db_scale_scarlett2_gain }
};

static const struct snd_kcontrol_new scarlett2_line_out_volume_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
		  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
	.name = "",
	.info = scarlett2_volume_ctl_info,
	.get  = scarlett2_volume_ctl_get,
	.put  = scarlett2_volume_ctl_put,
	.private_value = 0, /* max value */
	.tlv = { .p = db_scale_scarlett2_gain }
};

/*** HW/SW Volume Switch Controls ***/

static int scarlett2_sw_hw_enum_ctl_info(struct snd_kcontrol *kctl,
					 struct snd_ctl_elem_info *uinfo)
{
	static const char *const values[2] = {
		"SW", "HW"
	};

	return snd_ctl_enum_info(uinfo, 1, 2, values);
}

static int scarlett2_sw_hw_enum_ctl_get(struct snd_kcontrol *kctl,
					struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_mixer_data *private = elem->head.mixer->private_data;

	ucontrol->value.enumerated.item[0] =
		private->vol_sw_hw_switch[elem->control];
	return 0;
}

static int scarlett2_sw_hw_enum_ctl_put(struct snd_kcontrol *kctl,
					struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;
	
	int index = elem->control;
	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->vol_sw_hw_switch[index];
	val = !!ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->vol_sw_hw_switch[index] = val;
	private->vol_updated = 1; /* Trigger volumes to be updated */

	/* Change access mode to RO (hardware controlled volume)
	 * or RW (software controlled volume)
	 */
	if (val)
		private->vol_ctls[index]->vd[0].access &=
			~SNDRV_CTL_ELEM_ACCESS_WRITE;
	else
		private->vol_ctls[index]->vd[0].access |=
			SNDRV_CTL_ELEM_ACCESS_WRITE;

	/* Set SW volume to current HW volume */
	err = scarlett2_usb_set_config(
		mixer, SCARLETT2_CONFIG_LINE_OUT_VOLUME,
		index, private->master_vol - SCARLETT2_VOLUME_BIAS);
	if (err < 0)
		goto unlock;

	/* Notify of RO/RW change */
	snd_ctl_notify(mixer->chip->card, SNDRV_CTL_EVENT_MASK_INFO | SNDRV_CTL_EVENT_MASK_VALUE,
		       &private->vol_ctls[index]->id);

	/* Send SW/HW switch change to the device */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_SW_HW_SWITCH,
				       index, val);

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_sw_hw_enum_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = scarlett2_sw_hw_enum_ctl_info,
	.get  = scarlett2_sw_hw_enum_ctl_get,
	.put  = scarlett2_sw_hw_enum_ctl_put,
};

/*** Line Level/Instrument Level Switch Controls ***/
static int scarlett2_update_line_ctl_switches(struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	u8 pad_switches[SCARLETT2_PAD_SWITCH_MAX];
	u8 level_switches[SCARLETT2_LEVEL_SWITCH_MAX];
	u8 pow_switch;

	int i, err = 0;

	private->line_ctl_updated = 0;

	/* Update pad settings */
	if (info->pad_input_count) {
		err = scarlett2_usb_get_config(
			mixer,
			SCARLETT2_CONFIG_PAD_SWITCH,
			info->pad_input_count,
			pad_switches);
		if (err < 0)
			return err;

		for (i = 0; i < info->pad_input_count; i++)
			private->pad_switch[i] = !!pad_switches[i];
	}

	/* Update level settings */
	if (info->level_input_count) {
		err = scarlett2_usb_get_config(
			mixer,
			SCARLETT2_CONFIG_LEVEL_SWITCH,
			info->level_input_count,
			level_switches);
		if (err < 0)
			return err;

		for (i = 0; i < info->level_input_count; i++)
			private->level_switch[i] = !!level_switches[i];
	}

	/* Update phantom power settings */
	if (info->power_48v_count) {
		err = scarlett2_usb_get_config(
			mixer,
			SCARLETT2_CONFIG_48V_SWITCH,
			1, &pow_switch);
		if (err < 0)
			return err;

		for (i = 0; i < info->power_48v_count; i++)
			private->pow_switch[i] = !! (pow_switch & (1 << i));
	}

	return 0;
}

static int scarlett2_level_enum_ctl_info(struct snd_kcontrol *kctl,
					 struct snd_ctl_elem_info *uinfo)
{
	static const char *const values[2] = {
		"Line", "Inst"
	};

	return snd_ctl_enum_info(uinfo, 1, 2, values);
}

static int scarlett2_level_enum_ctl_get(struct snd_kcontrol *kctl,
					struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;
	
	if (private->line_ctl_updated) {
		mutex_lock(&private->data_mutex);
		scarlett2_update_line_ctl_switches(mixer);
		mutex_unlock(&private->data_mutex);
	}

	ucontrol->value.enumerated.item[0] =
		private->level_switch[elem->control];
	return 0;
}

static int scarlett2_level_enum_ctl_put(struct snd_kcontrol *kctl,
					struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;

	int index = elem->control;
	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->level_switch[index];
	val = !!ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->level_switch[index] = val;

	/* Send inst change to the device */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_LEVEL_SWITCH,
				       index, val);

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_level_enum_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = scarlett2_level_enum_ctl_info,
	.get  = scarlett2_level_enum_ctl_get,
	.put  = scarlett2_level_enum_ctl_put,
};

/*** Pad Switch Controls ***/
static int scarlett2_pad_ctl_get(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;

	if (private->line_ctl_updated) {
		mutex_lock(&private->data_mutex);
		scarlett2_update_line_ctl_switches(mixer);
		mutex_unlock(&private->data_mutex);
	}

	ucontrol->value.enumerated.item[0] =
		private->pad_switch[elem->control];
	return 0;
}

static int scarlett2_pad_ctl_put(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;

	int index = elem->control;
	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->pad_switch[index];
	val = !!ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->pad_switch[index] = val;

	/* Send pad change to the device */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_PAD_SWITCH,
				       index, val);

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_pad_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_pad_ctl_get,
	.put  = scarlett2_pad_ctl_put,
};

/*** Air Switch Controls ***/

static int scarlett2_air_ctl_get(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_mixer_data *private = elem->head.mixer->private_data;

	ucontrol->value.enumerated.item[0] =
		private->air_switch[elem->control];
	return 0;
}

static int scarlett2_air_ctl_put(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;

	int index = elem->control;
	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->air_switch[index];
	val = !!ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->air_switch[index] = val;

	/* Send switch change to the device */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_AIR_SWITCH,
				       index, val);

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_air_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_air_ctl_get,
	.put  = scarlett2_air_ctl_put,
};

/*** 48V Switch Controls ***/
static int scarlett2_48v_ctl_get(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;
	
	if (private->line_ctl_updated) {
		mutex_lock(&private->data_mutex);
		scarlett2_update_line_ctl_switches(mixer);
		mutex_unlock(&private->data_mutex);
	}

	ucontrol->value.enumerated.item[0] =
		private->pow_switch[elem->control];
	return 0;
}

static int scarlett2_48v_ctl_put(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;

	int index = elem->control;
	int i, oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->pow_switch[index];
	val = !!ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->pow_switch[index] = val;
	val = 0;
	for (i = 0; i < info->power_48v_count; ++i)
		val |= (private->pow_switch[i] << i);

	/* Send switch change to the device */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_48V_SWITCH, 0, val);

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_48v_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_48v_ctl_get,
	.put  = scarlett2_48v_ctl_put
};

/*** Mute/Dim Controls ***/

static int scarlett2_button_ctl_get(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;

	if (private->vol_updated) {
		mutex_lock(&private->data_mutex);
		scarlett2_update_volumes(mixer);
		mutex_unlock(&private->data_mutex);
	}

	ucontrol->value.enumerated.item[0] = private->buttons[elem->control];
	return 0;
}

static int scarlett2_button_ctl_put(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;

	int index = elem->control;
	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->buttons[index];
	val = !!ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->buttons[index] = val;

	/* Send switch change to the device */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_BUTTONS,
				       index, val);

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_button_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_button_ctl_get,
	.put  = scarlett2_button_ctl_put
};

/*** Create the analogue output controls ***/

static int scarlett2_add_line_out_ctls(struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const struct scarlett2_ports *ports = info->ports;
	int num_line_out = ports[SCARLETT2_PORT_TYPE_ANALOGUE].num[SCARLETT2_PORT_OUT];
	int err, i;
	s16 level;
	char s[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];

	/* Add R/O HW volume control */
	if (info->line_out_hw_vol) {
		snprintf(s, sizeof(s), "Master HW Out Volume");
		err = scarlett2_add_new_ctl(mixer,
					    &scarlett2_master_volume_ctl,
					    0, 1, s, &private->master_vol_ctl);
		if (err < 0)
			return err;
	}

	/* Add volume controls */
	for (i = 0; i < num_line_out; i++) {
		/* Fader */
		if (info->line_out_descrs[i])
			snprintf(s, sizeof(s), "Line Out %02d (%s) Volume", i + 1, info->line_out_descrs[i]);
		else
			snprintf(s, sizeof(s), "Line Out %02d Volume", i + 1);
		err = scarlett2_add_new_ctl(mixer,
					    &scarlett2_line_out_volume_ctl,
					    i, 1, s, &private->vol_ctls[i]);
		if (err < 0)
			return err;

		/* Initialize the value with software configuration
		 * Make the fader read-only if the SW/HW switch is set to HW
		 */
		if ((private->sw_cfg_volume) && (!private->vol_sw_hw_switch[i])) {
			level  = le16_to_cpu(private->sw_cfg_volume[i].volume);
			private->vol[i] = clamp(level + SCARLETT2_VOLUME_BIAS, 0, SCARLETT2_VOLUME_BIAS);
			private->vol_ctls[i]->vd[0].access |= SNDRV_CTL_ELEM_ACCESS_WRITE;
		}
		else {
			private->vol[i] = private->master_vol;
			private->vol_ctls[i]->vd[0].access &= ~SNDRV_CTL_ELEM_ACCESS_WRITE;
		}

		/* SW/HW Switch */
		if (info->line_out_hw_vol) {
			snprintf(s, sizeof(s), "Line Out %02d Control", i + 1);
			err = scarlett2_add_new_ctl(mixer,
						    &scarlett2_sw_hw_enum_ctl,
						    i, 1, s, NULL);
			if (err < 0)
				return err;
		}
	}

	/* Add HW button controls */
	for (i = 0; i < private->info->button_count; i++) {
		err = scarlett2_add_new_ctl(mixer, &scarlett2_button_ctl,
					    i, 1, scarlett2_button_names[i],
					    &private->button_ctls[i]);
		if (err < 0)
			return err;
	}

	/* Commite actual volumes */
	if (private->sw_cfg_volume) {
		for (i = 0; i < num_line_out; i++) {
			err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_LINE_OUT_VOLUME,
			                               i, private->vol[i] - SCARLETT2_VOLUME_BIAS);
			if (err < 0)
				return err;
		}
	}

	return err;
}

/*** Create the analogue input controls ***/
static int scarlett2_add_line_in_ctls(struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	int err, i;
	char s[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];

	/* Add input level (line/inst) controls */
	for (i = 0; i < info->level_input_count; i++) {
		snprintf(s, sizeof(s), "Line In %d Mode Switch", i + 1);
		err = scarlett2_add_new_ctl(mixer, &scarlett2_level_enum_ctl,
					    i, 1, s, &private->level_ctls[i]);
		if (err < 0)
			return err;
	}

	/* Add input pad controls */
	for (i = 0; i < info->pad_input_count; i++) {
		snprintf(s, sizeof(s), "Line In %d Pad Switch", i + 1);
		err = scarlett2_add_new_ctl(mixer, &scarlett2_pad_ctl,
					    i, 1, s, &private->pad_ctls[i]);
		if (err < 0)
			return err;
	}

	/* Add input air controls */
	for (i = 0; i < info->air_input_count; i++) {
		snprintf(s, sizeof(s), "Line In %d Air Switch", i + 1);
		err = scarlett2_add_new_ctl(mixer, &scarlett2_air_ctl,
					    i, 1, s, NULL);
		if (err < 0)
			return err;
	}

	/* Add input 48v controls */
	for (i = 0; i < info->power_48v_count; i++) {
		snprintf(s, sizeof(s), "Line 48V Switch %d", i + 1);
		err = scarlett2_add_new_ctl(mixer, &scarlett2_48v_ctl,
					    i, 1, s, &private->pow_ctls[i]);
		if (err < 0)
			return err;
	}

	return 0;
}

/*** Mixer Volume Controls ***/

static int scarlett2_mixer_ctl_info(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_info *uinfo)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = elem->channels;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = SCARLETT2_MIXER_MAX_VALUE;
	uinfo->value.integer.step = 1;
	return 0;
}

static int scarlett2_mixer_ctl_get(struct snd_kcontrol *kctl,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_mixer_data *private = elem->head.mixer->private_data;

	ucontrol->value.integer.value[0] = private->mix[elem->control];
	return 0;
}

static int scarlett2_mixer_ctl_put(struct snd_kcontrol *kctl,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;
	int oval, val, mix_num, input_num, err = 0;
	int index = elem->control;
	int cfg_idx;
	u32 level;
	__le32 *gain;

	mutex_lock(&private->data_mutex);

	oval = private->mix[index];
	val = ucontrol->value.integer.value[0];
	mix_num = index / SCARLETT2_INPUT_MIX_MAX;
	input_num = index % SCARLETT2_INPUT_MIX_MAX;

	if (oval == val)
		goto unlock;

	private->mix[index] = val;
	err = scarlett2_usb_set_mix(mixer, mix_num);
	if (err < 0)
		goto unlock;
	
	/* Update software configuration data */
	cfg_idx = mix_num * SCARLETT2_SW_CONFIG_MIXER_INPUTS + input_num;
	if (cfg_idx < private->sw_cfg_mixer_size) {
		level = scarlett2_sw_config_mixer_values[val];
		gain = &private->sw_cfg_mixer[cfg_idx];
		*gain = cpu_to_le32(level << 16); /* Convert to F32LE */
		scarlett2_commit_software_config(mixer, gain, sizeof(__le32));
	}

	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const DECLARE_TLV_DB_MINMAX(
	db_scale_scarlett2_mixer,
	SCARLETT2_MIXER_MIN_DB * 100,
	SCARLETT2_MIXER_MAX_DB * 100
);

static const struct snd_kcontrol_new scarlett2_mixer_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
		  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
	.name = "",
	.info = scarlett2_mixer_ctl_info,
	.get  = scarlett2_mixer_ctl_get,
	.put  = scarlett2_mixer_ctl_put,
	.private_value = SCARLETT2_MIXER_MAX_DB, /* max value */
	.tlv = { .p = db_scale_scarlett2_mixer }
};

static int scarlett2_add_mixer_ctls(struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_ports *ports = private->info->ports;
	int err, i, j;
	int cfg_idx, mix_idx;
	char s[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];
	u32 level;

	int num_inputs = ports[SCARLETT2_PORT_TYPE_MIX].num[SCARLETT2_PORT_OUT];
	int num_outputs = ports[SCARLETT2_PORT_TYPE_MIX].num[SCARLETT2_PORT_IN];

	/* For each mixer */
	for (i=0; i<num_outputs; ++i) {
		mix_idx   = i * SCARLETT2_INPUT_MIX_MAX;
		cfg_idx   = i * SCARLETT2_SW_CONFIG_MIXER_INPUTS; /* TODO: This may change, need to verify for other models */

		/* Add Mix control */
		for (j = 0; j < num_inputs; ++j, ++cfg_idx, ++mix_idx) {
			snprintf(s, sizeof(s),
				 "Mix %c Input %02d Gain",
				 'A' + i, j + 1);

			level = (cfg_idx < private->sw_cfg_mixer_size) ?
			        le32_to_cpu(private->sw_cfg_mixer[cfg_idx]) : 0;

			private->mix[mix_idx] = scarlett2_float_to_mixer_level(level) -
			                        (SCARLETT2_MIXER_MIN_DB * 2);

			err = scarlett2_add_new_ctl(mixer, &scarlett2_mixer_ctl,
						    mix_idx, 1, s, NULL);
			if (err < 0)
				return err;
		}

		/* Commit the actual mix state at startup */
		err = scarlett2_usb_set_mix(mixer, i);
		if (err < 0)
			return err;
	}

	return 0;
}

/*** Mux Source Selection Controls ***/
static int scarlett2_mux_src_enum_ctl_info(struct snd_kcontrol *kctl,
					   struct snd_ctl_elem_info *uinfo)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;
	int item = uinfo->value.enumerated.item;
	int items = private->num_inputs + 1;
	int port = clamp(item, 0, private->num_inputs) - 1;

	uinfo->type  = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = elem->channels;
	uinfo->value.enumerated.items = items;
	if (!items)
		return 0;
	if (item >= items)
		item = uinfo->value.enumerated.item = items - 1;

	scarlett2_fmt_port_name(uinfo->value.enumerated.name,
	                        sizeof(uinfo->value.enumerated.name), private->info,
	                        SCARLETT2_PORT_IN, port);

	return 0;
}

static int scarlett2_mux_src_enum_ctl_get(struct snd_kcontrol *kctl,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_mixer_data *private = elem->head.mixer->private_data;
	
	ucontrol->value.enumerated.item[0] = private->mux[elem->control] + 1; /* Element 0 is always 'Off' */
	return 0;
}

static int scarlett2_mux_src_enum_ctl_put(struct snd_kcontrol *kctl,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;
	int index = elem->control;
	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->mux[index];
	val  = ucontrol->value.integer.value[0];
	val  = clamp(val, 0, private->num_inputs) - 1;

	if (oval == val)
		goto unlock;

	private->mux[index] = val;
	err = scarlett2_usb_set_mux(mixer);
	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_mux_src_enum_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = scarlett2_mux_src_enum_ctl_info,
	.get  = scarlett2_mux_src_enum_ctl_get,
	.put  = scarlett2_mux_src_enum_ctl_put,
};

static int scarlett2_init_mux(struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	int port, err;
	char name[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];

	/* Read MUX settings */
	err = scarlett2_usb_get_mux(mixer);
	if (err < 0)
		return err;

	/* Apply MUX settings */
	err = scarlett2_usb_set_mux(mixer);
	if (err < 0)
		return err;

	/* Create mux control ports based on output port list */
	for (port = 0; port < private->num_outputs; ++port) {
		/* Create port descriptor */
		scarlett2_fmt_port_name(name, SNDRV_CTL_ELEM_ID_NAME_MAXLEN, private->info, SCARLETT2_PORT_OUT, port);
		strncat(name, " Source", SNDRV_CTL_ELEM_ID_NAME_MAXLEN);

		err = scarlett2_add_new_ctl(mixer,
					    &scarlett2_mux_src_enum_ctl,
					    port, 1, name, NULL);
		if (err < 0)
			return err;
	}

	return err;
}

/*** Meter Controls ***/

static int scarlett2_meter_ctl_info(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_info *uinfo)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = elem->channels;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 4095;
	uinfo->value.integer.step = 1;
	return 0;
}

static int scarlett2_meter_ctl_get(struct snd_kcontrol *kctl,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	u16 meter_levels[SCARLETT2_NUM_METERS];
	int i, err;

	err = scarlett2_usb_get_meter_levels(elem->head.mixer, meter_levels);
	if (err < 0)
		return err;

	for (i = 0; i < elem->channels; i++)
		ucontrol->value.integer.value[i] = meter_levels[i];

	return 0;
}

static const struct snd_kcontrol_new scarlett2_meter_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_PCM,
	.access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	.name = "",
	.info = scarlett2_meter_ctl_info,
	.get  = scarlett2_meter_ctl_get
};

static int scarlett2_add_meter_ctl(struct usb_mixer_interface *mixer)
{
	return scarlett2_add_new_ctl(mixer, &scarlett2_meter_ctl,
				     0, SCARLETT2_NUM_METERS,
				     "Level Meter", NULL);
}

/*** MSD Controls ***/

static int scarlett2_msd_ctl_get(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_mixer_data *private = elem->head.mixer->private_data;

	ucontrol->value.enumerated.item[0] = private->msd_switch;
	return 0;
}

static int scarlett2_msd_ctl_put(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;

	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->msd_switch;
	val = !!ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->msd_switch = val;

	/* Send switch change to the device */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_MSD_SWITCH,
				       0, val);

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_msd_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_msd_ctl_get,
	.put  = scarlett2_msd_ctl_put,
};

static int scarlett2_add_msd_ctl(struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;

	if (!info->has_msd_mode)
		return 0;

	/* If MSD mode is off, hide the switch by default */
	if (!private->msd_switch && !(mixer->chip->setup & SCARLETT2_MSD_ENABLE))
		return 0;

	/* Add MSD control */
	return scarlett2_add_new_ctl(mixer, &scarlett2_msd_ctl,
				     0, 1, "MSD Mode", NULL);
}

/*** Speaker Switching Control ***/
static int scarlett2_update_speaker_switch_enum_ctl(struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	u8 speaker_switching, speaker_switch;
	int err = 0;

	private->speaker_updated = 0;
	if (!info->has_speaker_switching)
		return 0;

	/* check if speaker switching is enabled */
	err = scarlett2_usb_get_config(
		mixer,
		SCARLETT2_CONFIG_SPEAKER_SWITCHING_SWITCH,
		1, &speaker_switching);
	if (err < 0)
		return err;

	/* get actual speaker & talkback configuration */
	err = scarlett2_usb_get_config(
		mixer,
		SCARLETT2_CONFIG_MAIN_ALT_SPEAKER_SWITCH,
		1, &speaker_switch);
	if (err < 0)
		return err;

	/* decode speaker & talkback values */
	private->speaker_switch  = (speaker_switching) ? (speaker_switch & 1) + 1 : 0;
	if (info->has_talkback)
		private->talkback_switch = !!(speaker_switch & 2);

	return 0;
}

static int scarlett2_speaker_switch_enum_ctl_info(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_info *uinfo)
{
	static const char *const values[3] = {
		"Off", "Main", "Alt"
	};

	return snd_ctl_enum_info(uinfo, 1, 3, values);
}

static int scarlett2_speaker_switch_enum_ctl_get(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;
	
	if (private->speaker_updated) {
		mutex_lock(&private->data_mutex);
		scarlett2_update_speaker_switch_enum_ctl(mixer);
		mutex_unlock(&private->data_mutex);
	}

	ucontrol->value.enumerated.item[0] =
		private->speaker_switch;
	return 0;
}

static int scarlett2_talkback_switch_ctl_get(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;

	if (private->speaker_updated) {
		mutex_lock(&private->data_mutex);
		scarlett2_update_speaker_switch_enum_ctl(mixer);
		mutex_unlock(&private->data_mutex);
	}

	ucontrol->value.enumerated.item[0] =
		private->talkback_switch;
	return 0;
}

static int scarlett2_mix_talkback_switch_ctl_get(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;
	
	ucontrol->value.enumerated.item[0] =
		private->mix_talkback[elem->control];
	return 0;
}

static int scarlett2_speaker_switch_update_state(struct usb_mixer_interface *mixer, int alt, int talkback) {
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	int old_alt, old_talk, err = 0;

	mutex_lock(&private->data_mutex);

	old_alt = private->speaker_switch;
	old_talk = private->talkback_switch;

	if ((old_alt == alt) && (old_talk == talkback))
		goto unlock;

	private->speaker_switch = alt;
	private->talkback_switch = talkback;
	
	/* enable/disable speaker switching */
	if (old_alt == 0 || alt == 0) {
		err = scarlett2_usb_set_config(
			mixer, SCARLETT2_CONFIG_SPEAKER_SWITCHING_SWITCH,
			0, !!alt);
	}

	/* update talkback speaker and talkback */
	if (!err) {
		int val = (alt == 2);
		if (info->has_talkback)
			val |= talkback << 1;

		err = scarlett2_usb_set_config(
			mixer, SCARLETT2_CONFIG_MAIN_ALT_SPEAKER_SWITCH,
			0, val);
	}

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static int scarlett2_speaker_switch_enum_ctl_put(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;

	return scarlett2_speaker_switch_update_state(mixer, 
						     ucontrol->value.integer.value[0],
						     private->talkback_switch);
}

static int scarlett2_talkback_switch_ctl_put(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;

	return scarlett2_speaker_switch_update_state(mixer, 
						     private->speaker_switch,
						     ucontrol->value.integer.value[0]);
}

static int scarlett2_mix_talkback_switch_ctl_put(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_ports *ports = private->info->ports;
	int i, val, old_val, num_mixes, err = 0;

	mutex_lock(&private->data_mutex);

	old_val  = private->mix_talkback[elem->control];
	val      = !! ucontrol->value.integer.value[0];
	if (old_val == val)
		goto unlock;

	private->mix_talkback[elem->control] = val;

	/* Build bit mask for each mixer channel and commit to device */
	val      = 0;
	num_mixes = ports[SCARLETT2_PORT_TYPE_MIX].num[SCARLETT2_PORT_IN];
	for (i=0; i<num_mixes; ++i)
		val |= (private->mix_talkback[i]) << i;

	err = scarlett2_usb_set_config(
		mixer, SCARLETT2_CONFIG_MIX_TALKBACK,
		0, val);

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_speaker_switch_enum_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = scarlett2_speaker_switch_enum_ctl_info,
	.get  = scarlett2_speaker_switch_enum_ctl_get,
	.put  = scarlett2_speaker_switch_enum_ctl_put,
};

static const struct snd_kcontrol_new scarlett2_talkback_switch_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_talkback_switch_ctl_get,
	.put  = scarlett2_talkback_switch_ctl_put,
};

static const struct snd_kcontrol_new scarlett2_mix_talkback_switch_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_mix_talkback_switch_ctl_get,
	.put  = scarlett2_mix_talkback_switch_ctl_put,
};

static int scarlett2_add_speaker_switch_ctl(
	struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const struct scarlett2_ports *ports = private->info->ports;
	char s[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];
	int err = 0, num_mixes, i;

	if (info->has_speaker_switching) {
		/* Add speaker switching control */
		err = scarlett2_add_new_ctl(
			mixer, &scarlett2_speaker_switch_enum_ctl,
			0, 1, "Speaker Switching", &private->speaker_ctl);
		
		if (err < 0)
			return err;
	}

	if (info->has_talkback) {
		/* Add talkback switching control */
		err = scarlett2_add_new_ctl(
			mixer, &scarlett2_talkback_switch_ctl,
			0, 1, "Talkback Switching", &private->talkback_ctl);
		
		if (err < 0)
			return err;

		/* Add 'talkback enable' for mix */
		num_mixes = ports[SCARLETT2_PORT_TYPE_MIX].num[SCARLETT2_PORT_IN];
		for (i=0; i<num_mixes; ++i) {
			snprintf(s, sizeof(s), "Mix %c Talkback", 'A' + i);
			
			err = scarlett2_add_new_ctl(
				mixer, &scarlett2_mix_talkback_switch_ctl,
				i, 1, s, &private->mix_talkback_ctls[i]);
		
			if (err < 0)
				return err;
		}
	}

	return 0;
}

/*** Cleanup/Suspend Callbacks ***/

static void scarlett2_private_free(struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;

	cancel_delayed_work_sync(&private->work);
	if (private->sw_cfg_data != NULL)
		kfree(private->sw_cfg_data);
	kfree(private);
	mixer->private_data = NULL;
}

static void scarlett2_private_suspend(struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;

	if (cancel_delayed_work_sync(&private->work))
		scarlett2_config_save(private->mixer);
}

/* Look through the interface descriptors for the Focusrite Control
 * interface (bInterfaceClass = 255 Vendor Specific Class) and set the
 * interface number, endpoint address, packet size, and interval in
 * private
 */
static int scarlett2_find_fc_interface(struct usb_device *dev,
				       struct scarlett2_mixer_data *private) {
	struct usb_host_config *config = dev->actconfig;
	int i;

	for (i = 0; i < config->desc.bNumInterfaces; i++) {
		struct usb_interface *intf = config->interface[i];
		struct usb_interface_descriptor *desc =
			&intf->altsetting[0].desc;
		if (desc->bInterfaceClass == 255) {
			struct usb_endpoint_descriptor *epd =
				get_endpoint(intf->altsetting, 0);
			private->interface = desc->bInterfaceNumber;
			private->endpoint = epd->bEndpointAddress &
				USB_ENDPOINT_NUMBER_MASK;
			private->maxpacketsize = epd->wMaxPacketSize;
			private->interval = epd->bInterval;
			return 0;
		}
	}

	return -1;
}

/* Initialise private data, routing, sequence number */
static int scarlett2_init_private(struct usb_mixer_interface *mixer,
				  const struct scarlett2_device_info *info)
{
	struct scarlett2_mixer_data *private =
		kzalloc(sizeof(struct scarlett2_mixer_data), GFP_KERNEL);
	int err;

	if (!private)
		return -ENOMEM;

	mutex_init(&private->usb_mutex);
	mutex_init(&private->data_mutex);
	INIT_DELAYED_WORK(&private->work, scarlett2_config_save_work);
	mixer->private_data = private;
	mixer->private_free = scarlett2_private_free;
	mixer->private_suspend = scarlett2_private_suspend;
	private->info = info;
	private->num_inputs = scarlett2_count_ports(info->ports, SCARLETT2_PORT_IN);
	private->num_outputs = scarlett2_count_ports(info->ports, SCARLETT2_PORT_OUT);
	private->scarlett2_seq = 0;
	private->mixer = mixer;
	private->vol_updated = 0;
	private->line_ctl_updated = 0;
	private->speaker_updated = 0;
	private->speaker_switch = 0;
	private->talkback_switch = 0;

	private->sw_cfg_offset = 0;
	private->sw_cfg_size = 0;
	private->sw_cfg_data = NULL;
	private->sw_cfg_stereo = NULL;
	private->sw_cfg_volume = NULL;
	private->sw_cfg_mixer = NULL;
	private->sw_cfg_mixer_size = 0;
	private->sw_cfg_cksum = NULL;

	err = scarlett2_find_fc_interface(mixer->chip->dev, private);

	if (err < 0)
		return -EINVAL;

	return 0;
}

/* Read line-in config and line-out volume settings on start */
static int scarlett2_read_configs(struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const struct scarlett2_ports *ports = info->ports;
	int num_line_out =
		ports[SCARLETT2_PORT_TYPE_ANALOGUE].num[SCARLETT2_PORT_OUT];
	u8 level_switches[SCARLETT2_LEVEL_SWITCH_MAX];
	u8 pad_switches[SCARLETT2_PAD_SWITCH_MAX];
	u8 air_switches[SCARLETT2_AIR_SWITCH_MAX];
	u8 msd_switch;
	u8 speaker_switching, speaker_switch;
	u8 pow_switch;
	__le16 mix_talkbacks;
	struct scarlett2_usb_volume_status volume_status;
	int err, i, num_mixes, val;

	/* INST buttons */
	if (info->level_input_count) {
		err = scarlett2_usb_get_config(
			mixer,
			SCARLETT2_CONFIG_LEVEL_SWITCH,
			info->level_input_count,
			level_switches);
		if (err < 0)
			return err;
		for (i = 0; i < info->level_input_count; i++)
			private->level_switch[i] = level_switches[i];
	}

	/* PAD buttons */
	if (info->pad_input_count) {
		err = scarlett2_usb_get_config(
			mixer,
			SCARLETT2_CONFIG_PAD_SWITCH,
			info->pad_input_count,
			pad_switches);
		if (err < 0)
			return err;
		for (i = 0; i < info->pad_input_count; i++)
			private->pad_switch[i] = pad_switches[i];
	}

	/* AIR input settings */
	if (info->air_input_count) {
		err = scarlett2_usb_get_config(
			mixer,
			SCARLETT2_CONFIG_AIR_SWITCH,
			info->air_input_count,
			air_switches);
		if (err < 0)
			return err;
		for (i = 0; i < info->air_input_count; i++)
			private->air_switch[i] = air_switches[i];
	}

	/* Phantom power settings */
	if (info->power_48v_count) {
		err = scarlett2_usb_get_config(
			mixer,
			SCARLETT2_CONFIG_48V_SWITCH,
			1, &pow_switch);
		if (err < 0)
			return err;
		
		for (i = 0; i < info->power_48v_count; i++)
			private->pow_switch[i] = !! (pow_switch & (1 << i));
	}

	/* Mass Storage Device (MSD) mode */
	if (info->has_msd_mode) {
		err = scarlett2_usb_get_config(
			mixer,
			SCARLETT2_CONFIG_MSD_SWITCH,
			1,
			&msd_switch);
		if (err < 0)
			return err;
		private->msd_switch = msd_switch;
	}

	/* Speaker switching (ALT button) and optional TALKBACK button */
	if (info->has_speaker_switching) {
		/* check if speaker switching is enabled */
		err = scarlett2_usb_get_config(
			mixer,
			SCARLETT2_CONFIG_SPEAKER_SWITCHING_SWITCH,
			1, &speaker_switching);
		if (err < 0)
			return err;
	
		/* get actual speaker & talkback configuration */
		err = scarlett2_usb_get_config(
			mixer,
			SCARLETT2_CONFIG_MAIN_ALT_SPEAKER_SWITCH,
			1, &speaker_switch);
		if (err < 0)
			return err;

		/* decode speaker & talkback values */
		private->speaker_switch  = (speaker_switching) ? (speaker_switch & 1) + 1 : 0;
		if (info->has_talkback)
			private->talkback_switch = !!(speaker_switch & 2);
	}

	/* Talkback routing to each output of internal mixer */
	if (info->has_talkback) {
		err = scarlett2_usb_get_config(
			mixer,
			SCARLETT2_CONFIG_MIX_TALKBACK,
			1, &mix_talkbacks);
		if (err < 0)
			return err;
		
		/* Each talkback switch is just a bit assigned to the corresponding mixer output */
		val = le16_to_cpu(mix_talkbacks);
		num_mixes = ports[SCARLETT2_PORT_TYPE_MIX].num[SCARLETT2_PORT_IN];
		for (i=0; i<num_mixes; ++i)
			private->mix_talkback[i] = !!(val & (1 << i));
	}

	err = scarlett2_usb_get_volume_status(mixer, &volume_status);
	if (err < 0)
		return err;

	private->master_vol = clamp(
		volume_status.master_vol + SCARLETT2_VOLUME_BIAS,
		0, SCARLETT2_VOLUME_BIAS);

	for (i = 0; i < num_line_out; i++) {
		int volume;

		private->vol_sw_hw_switch[i] =
			info->line_out_hw_vol
				&& volume_status.sw_hw_switch[i];

		volume = private->vol_sw_hw_switch[i]
			   ? volume_status.master_vol
			   : volume_status.sw_vol[i];
		volume = clamp(volume + SCARLETT2_VOLUME_BIAS,
			       0, SCARLETT2_VOLUME_BIAS);
		private->vol[i] = volume;
	}

	for (i = 0; i < info->button_count; i++)
		private->buttons[i] = !!volume_status.buttons[i];

	return 0;
}

static int scarlett2_read_software_configs(struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_ports *ports = private->info->ports;
	int i, chunk, err;
	__le16 sw_cfg_size;
	int num_line_out = ports[SCARLETT2_PORT_TYPE_ANALOGUE].num[SCARLETT2_PORT_OUT];

	/* Currently it is static constant, maybe it will be properly detected in the future */
	private->sw_cfg_offset = SCARLETT2_SW_CONFIG_BASE;

	/* Obtain actual size of sofware config */
	err = scarlett2_usb_get(mixer, private->sw_cfg_offset + SCARLETT2_SW_CONFIG_SIZE_OFFSET, &sw_cfg_size, sizeof(sw_cfg_size));
	if (err < 0)
		return err;

	/* Allocate memory for the configuration space */
	private->sw_cfg_size  = le16_to_cpu(sw_cfg_size);
	private->sw_cfg_data  = kmalloc(private->sw_cfg_size, GFP_KERNEL);
	if (private->sw_cfg_data == NULL)
		return -ENOMEM;

	/* Associate pointers with their locations, last 4 bytes are checksum */
	private->sw_cfg_stereo  = (__le32 *)&private->sw_cfg_data[SCARLETT2_SW_CONFIG_STEREO_BITS_OFFSET];
	private->sw_cfg_volume  = (struct scarlett2_sw_cfg_volume *)&private->sw_cfg_data[SCARLETT2_SW_CONFIG_VOLUMES_OFFSET];
	private->sw_cfg_mixer   = (__le32 *)&private->sw_cfg_data[SCARLETT2_SW_CONFIG_MIXER_OFFSET];
	private->sw_cfg_cksum   = (__le32 *)&private->sw_cfg_data[private->sw_cfg_size - sizeof(__le32)];

	private->sw_cfg_mixer_size   = SCARLETT2_OUTPUT_MIX_MAX * SCARLETT2_SW_CONFIG_MIXER_INPUTS;

	/* Check that we don't get out of the configuration space */
	if (private->sw_cfg_cksum < (__le32 *)&private->sw_cfg_stereo[1])
		private->sw_cfg_stereo       = NULL;
	if (private->sw_cfg_cksum < (__le32 *)&private->sw_cfg_volume[num_line_out])
		private->sw_cfg_volume       = NULL;
	if (private->sw_cfg_cksum < (__le32 *)&private->sw_cfg_mixer[private->sw_cfg_mixer_size])
		private->sw_cfg_mixer_size   = 0;

	/* Request the software config with fixed-size data chunks */
	for (i=0; i<private->sw_cfg_size; ) {
		chunk = (private->sw_cfg_size - i);
		if (chunk > SCARLETT2_SW_CONFIG_PACKET_SIZE)
			chunk = SCARLETT2_SW_CONFIG_PACKET_SIZE;

		/* Request yet another chunk */
		err = scarlett2_usb_get(mixer, private->sw_cfg_offset + i, &private->sw_cfg_data[i], chunk);
		if (err < 0)
			return err;
		i += chunk;
	}

	return err;
}

static int scarlett2_commit_software_config(
	struct usb_mixer_interface *mixer,
	void *ptr, /* the pointer of the first changed byte in the configuration */
	int bytes /* the actual number of bytes changed in the configuration */
)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	struct {
		__le32 offset;
		__le32 bytes;
		u8 data[SCARLETT2_SW_CONFIG_PACKET_SIZE];
	} __packed req;
	int i, offset, chunk, err = 0;
	u8 *bptr;
	const __le32 *ckptr;
	s32 checksum;

	/* Check bounds, we should not exceed them */
	bptr = ptr;
	offset = bptr - private->sw_cfg_data;

	if ((private->sw_cfg_data == NULL) ||
	    (offset < 0) || 
	    ((offset + bytes) > private->sw_cfg_size))
		return -EINVAL;

	/* Re-compute the checksum of the software configuration area */
	checksum = 0;
	*(private->sw_cfg_cksum) = 0;
	ckptr = (__le32 *)private->sw_cfg_data;
	for (i=0; i<private->sw_cfg_size; i += sizeof(__le32))
		checksum -= le32_to_cpu(*(ckptr++));
	*(private->sw_cfg_cksum) = cpu_to_le32(checksum);

	/* Cancel any pending NVRAM save */
	cancel_delayed_work_sync(&private->work);

	/* Transfer the configuration with fixed-size data chunks */
	for (i=0; i<bytes; ) {
		chunk = (bytes - i);
		if (chunk > SCARLETT2_SW_CONFIG_PACKET_SIZE)
			chunk = SCARLETT2_SW_CONFIG_PACKET_SIZE;

		/* Send yet another chunk of data */
		req.offset = cpu_to_le32(SCARLETT2_SW_CONFIG_BASE + offset + i);
		req.bytes = cpu_to_le32(chunk);
		memcpy(req.data, &bptr[i], chunk);

		err = scarlett2_usb(mixer, SCARLETT2_USB_SET_DATA, &req, chunk + sizeof(__le32)*2, NULL, 0);
		if (err < 0)
			return err;

		i += chunk;
	}

	/* Transfer the actual checksum */
	req.offset = cpu_to_le32(SCARLETT2_SW_CONFIG_BASE + private->sw_cfg_size - sizeof(__le32));
	req.bytes = cpu_to_le32(sizeof(__le32));
	memcpy(req.data, private->sw_cfg_cksum, sizeof(__le32));

	err = scarlett2_usb(mixer, SCARLETT2_USB_SET_DATA, &req, sizeof(__le32)*3, NULL, 0);
	if (err < 0)
		return err;

	/* Schedule the change to be written to NVRAM */
	schedule_delayed_work(&private->work, msecs_to_jiffies(2000));

	return err;
}

/* Notify on volume change */
static void scarlett2_mixer_interrupt_vol_change(
	struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_ports *ports = private->info->ports;
	int num_line_out =
		ports[SCARLETT2_PORT_TYPE_ANALOGUE].num[SCARLETT2_PORT_OUT];
	int i;

	private->vol_updated = 1;

	snd_ctl_notify(mixer->chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
		       &private->master_vol_ctl->id);

	for (i = 0; i < num_line_out; i++) {
		if (!private->vol_sw_hw_switch[i])
			continue;
		snd_ctl_notify(mixer->chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &private->vol_ctls[i]->id);
	}
}

/* Notify on PAD/INST/48V button state change */
static void scarlett2_mixer_interrupt_line_in_ctl_change(
	struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	int i;

	/* Trigger all PAD inputs for changes */
	if (info->pad_input_count) {
		private->line_ctl_updated = 1;

		for (i = 0; i < info->pad_input_count; i++) {
			snd_ctl_notify(mixer->chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &private->pad_ctls[i]->id);
		}
	}
	
	/* Trigger all INST inputs for changes */
	if (info->level_input_count) {
		private->line_ctl_updated = 1;

		for (i = 0; i < info->level_input_count; i++) {
			snd_ctl_notify(mixer->chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &private->level_ctls[i]->id);
		}
	}

	/* Trigger all 48V inputs for changes */
	if (info->power_48v_count) {
		private->line_ctl_updated = 1;

		for (i = 0; i < info->power_48v_count; i++) {
			snd_ctl_notify(mixer->chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &private->pow_ctls[i]->id);
		}
	}
}

/* Notify on button change */
static void scarlett2_mixer_interrupt_button_change(
	struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;
	int i;

	private->vol_updated = 1;

	for (i = 0; i < private->info->button_count; i++) {
		snd_ctl_notify(mixer->chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &private->button_ctls[i]->id);
	}
}

/* Notify on speaker change */
static void scarlett2_mixer_interrupt_speaker_change(
	struct usb_mixer_interface *mixer)
{
	struct scarlett2_mixer_data *private = mixer->private_data;

	private->speaker_updated = 1;

	snd_ctl_notify(mixer->chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
		       &private->speaker_ctl->id);

	if (private->info->has_talkback)
		snd_ctl_notify(mixer->chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &private->talkback_ctl->id);
}

/* Interrupt callback */
static void scarlett2_mixer_interrupt(struct urb *urb)
{
	struct usb_mixer_interface *mixer = urb->context;
	int len = urb->actual_length;
	int ustatus = urb->status;

	if (ustatus != 0)
		goto requeue;

	if (len == 8) {
		u32 data = le32_to_cpu(*(__le32 *)urb->transfer_buffer);
		if (data & SCARLETT2_USB_INTERRUPT_VOL_CHANGE)
			scarlett2_mixer_interrupt_vol_change(mixer);
		if (data & SCARLETT2_USB_INTERRUPT_LINE_CTL_CHANGE)
			scarlett2_mixer_interrupt_line_in_ctl_change(mixer);
		if (data & SCARLETT2_USB_INTERRUPT_BUTTON_CHANGE)
			scarlett2_mixer_interrupt_button_change(mixer);
		if (data & SCARLETT2_USB_INTERRUPT_SPEAKER_CHANGE)
			scarlett2_mixer_interrupt_speaker_change(mixer);
	} else {
		usb_audio_err(mixer->chip,
			      "scarlett mixer interrupt length %d\n", len);
	}

requeue:
	if (ustatus != -ENOENT &&
	    ustatus != -ECONNRESET &&
	    ustatus != -ESHUTDOWN) {
		urb->dev = mixer->chip->dev;
		urb->actual_length = 0;
		*(u32 *)urb->transfer_buffer = 0;
		usb_submit_urb(urb, GFP_ATOMIC);
	}
}

static int scarlett2_mixer_status_create(struct usb_mixer_interface *mixer)
{
	struct usb_device *dev = mixer->chip->dev;
	struct scarlett2_mixer_data *private = mixer->private_data;
	unsigned int pipe = usb_rcvintpipe(dev, private->endpoint);
	void *transfer_buffer;

	if (mixer->urb) {
		usb_audio_err(mixer->chip,
			      "%s: mixer urb already in use!\n", __func__);
		return 0;
	}

	if (usb_pipe_type_check(dev, pipe))
		return -EINVAL;

	mixer->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!mixer->urb)
		return -ENOMEM;

	transfer_buffer = kmalloc(private->maxpacketsize, GFP_KERNEL);
	if (!transfer_buffer)
		return -ENOMEM;

	usb_fill_int_urb(mixer->urb, dev, pipe,
			 transfer_buffer, private->maxpacketsize,
			 scarlett2_mixer_interrupt, mixer,
			 private->interval);

	return usb_submit_urb(mixer->urb, GFP_KERNEL);
}

/* Entry point */
int snd_scarlett_gen2_controls_create(struct usb_mixer_interface *mixer)
{
	struct snd_usb_audio *chip = mixer->chip;
	const struct scarlett2_device_info *info;
	int i, err;

	/* only use UAC_VERSION_2 */
	if (!mixer->protocol)
		return 0;

	/* Find actual device descriptor */
	for (i=0; ; ++i) {
		info = scarlett2_supported_devices[i];
		if (info == NULL) /* End of list, device is not (yet) supported */
			return -EINVAL;
		if (info->usb_id == chip->usb_id)
			break;
	}

	if (!(chip->setup & SCARLETT2_ENABLE)) {
		usb_audio_err(chip,
			"Focusrite Scarlett Gen 2/3 Mixer Driver disabled; "
			"use options snd_usb_audio vid=0x%04x pid=0x%04x "
			"device_setup=1 to enable and report any issues "
			"to g@b4.vu",
			USB_ID_VENDOR(chip->usb_id),
			USB_ID_PRODUCT(chip->usb_id));
		return 0;
	}

	usb_audio_err(chip, "Focusrite Scarlett Gen 2/3 Mixer Driver enabled pid=0x%04x", USB_ID_PRODUCT(chip->usb_id));

	/* Initialise private data, routing, sequence number */
	err = scarlett2_init_private(mixer, info);
	if (err < 0)
		return err;

	/* Send proprietary USB initialisation sequence */
	err = scarlett2_usb_init(mixer);
	if (err < 0)
		return err;

	/* Read volume levels and controls from the interface */
	err = scarlett2_read_configs(mixer);
	if (err < 0)
		return err;

	/* Read software configuration containing mixer gains */
	err = scarlett2_read_software_configs(mixer);
	if (err < 0)
		return err;

	/* Create the analogue output controls */
	err = scarlett2_add_line_out_ctls(mixer);
	if (err < 0)
		return err;

	/* Create the analogue input controls */
	err = scarlett2_add_line_in_ctls(mixer);
	if (err < 0)
		return err;

	/* Create the input, output, and mixer mux input selections */
	err = scarlett2_init_mux(mixer);
	if (err < 0)
		return err;

	/* Create the matrix mixer controls */
	err = scarlett2_add_mixer_ctls(mixer);
	if (err < 0)
		return err;

	/* Create the level meter controls */
	err = scarlett2_add_meter_ctl(mixer);
	if (err < 0)
		return err;

	/* Create the MSD control */
	err = scarlett2_add_msd_ctl(mixer);
	if (err < 0)
		return err;

	/* Create the speaker switch control */
	err = scarlett2_add_speaker_switch_ctl(mixer);
	if (err < 0)
		return err;

	/* Set up the interrupt polling */
	err = scarlett2_mixer_status_create(mixer);
	if (err < 0)
		return err;

	return 0;
}
