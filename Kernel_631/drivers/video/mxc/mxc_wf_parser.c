#ifdef __KERNEL__  // compile into e-ink driver

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>

#define WDBG(s...) printk(s)
#define WINF(s...) printk(s)
#define WERR(s...) printk(s)

#define wbf_malloc(n) kmalloc(n, GFP_KERNEL)
#define wbf_free(ptr) kfree(ptr)

static uint8_t *wbf_data, *out_data;
static uint64_t wbf_pos, wbf_size;
static uint64_t out_pos, out_size, out_max;

static inline int wbf_fopen(void) { wbf_pos = 0; return 1; }
static inline void wbf_fclose(void) { }
static inline int64_t wbf_fseek(int64_t pos) { wbf_pos = pos; return 0; }
static inline int64_t wbf_ftell(void) { return wbf_pos; }
static inline uint8_t wbf_fgetc(void) { return wbf_data[wbf_pos++]; }
static inline int wbf_feof(void) { return (wbf_pos >= wbf_size); }

static inline int out_fopen(void) {out_pos = out_max = 0; return 1; }
static inline void out_fclose(void) { }
static inline int64_t out_fseek(int64_t pos) { out_pos = pos; return 0; }
static inline int64_t out_ftell(void) { return out_pos; }

static inline int out_fputc(char c)
{
	if (out_pos >= out_size) return -1;
	out_data[out_pos++] = (c);
	if (out_max < out_pos) out_max = out_pos;
	return (int)c;
}

static inline int out_fwrite(void *ptr, int count)
{
	if (out_pos + count > out_size) return -1;
	memcpy(&out_data[out_pos], ptr, count);
	out_pos += count;
	if (out_max < out_pos) out_max = out_pos;
	return 1;
}

/* compressed iMX7 output not supported in kernel */
static inline int wdff_fopen_r(int i, int j) { return 0; }
static inline int wdff_fopen_w(int i, int j) { return 0; }
static inline void wdff_fclose(void) { }
static inline int wdff_fseek(int64_t pos) { return -1; }
static inline int wdff_fputc(char c) { return -1; }
static inline int wdff_fread(uint8_t *ptr, int count) { return -1; }
static inline int wdff_remove(int i, int j) { return 0; }

static int wbf_convert_func(int o_type, int verbose);

int wbf_convert(uint8_t *iptr, int isize, uint8_t *optr, int *osize, int otype)
{
	int ret;

	wbf_data = iptr;
	wbf_size = isize;
	wbf_pos = 0;
	out_data = optr;
	out_size = *osize;
	out_pos = out_max = 0;
	ret = wbf_convert_func(otype, 0);
	*osize = out_max;
	return ret;
}

#else // userspace executable

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define WDBG(s...) fprintf(stderr, s)
#define WINF(s...) fprintf(stderr, s)
#define WERR(s...) fprintf(stderr, s)

#define wbf_malloc(n) malloc(n)
#define wbf_free(ptr) free(ptr)

#define wbf_fopen() (wbf = wbf_filename ? fopen(wbf_filename, "rb") : stdin)
#define wbf_fclose() fclose(wbf)
#define wbf_fseek(pos) fseek(wbf, pos, SEEK_SET)
#define wbf_ftell() ftell(wbf)
#define wbf_fgetc() fgetc(wbf)
#define wbf_feof()  feof(wbf)

#define out_fopen() (fw = out_filename ? fopen(out_filename, "wb") : stdout)
#define out_fclose() fclose(fw)
#define out_fseek(pos) fseek(fw, pos, SEEK_SET)
#define out_ftell() ftell(fw)
#define out_fputc(c) fputc(c, fw)
#define out_fwrite(ptr, count) fwrite(ptr, count, 1, fw)

#define wdff_fopen_r(i, j) (tfw = fopen(frmName(i, j), "rb"))
#define wdff_fopen_w(i, j) (tfw = fopen(frmName(i, j), "wb"))
#define wdff_fclose() { if (tfw) fclose(tfw); tfw = NULL; }
#define wdff_fseek(pos) fseek(tfw, pos, SEEK_SET)
#define wdff_fputc(c) fputc(c, tfw)
#define wdff_fread(ptr, count) fread(ptr, count, 1, tfw)
#define wdff_remove(i, j) remove(frmName(i, j))

static int wbf_convert_func(int o_type, int verbose);

const char *wbf_filename;
const char *out_filename;
char wdfFrames[32];

FILE *wbf;
FILE *fw;
FILE *tfw;
FILE *wdff; // [sp+54h] [bp-FCh]@617

static char *frmName(int32_t i, int32_t j)
{
	sprintf(wdfFrames, "frame_%d_%d.tmp", i, j);
	return wdfFrames;
}


static void usage(const char *a1)
{
	fprintf(
					 stderr,
					 "Usage: %s [-o file] [-v] [-h] file\n"
					 "Options:\n"
					 "-o <file>\tSend i.MX5xx or i.MX6xx style output to <file>\n"
					 "-o1 <file>\tSend i.MX7 output to <file>\n"
					 "-o2 <file>\tSend compressed i.MX7 output to <file>\n"
					 "-v \t\tVerbose output (Print debug information)\n"
					 "-h\t\tPrint help info\n",
					 a1);
}


int main(int argc, char **argv)
{
	int i, o_type=1, verbose=0;

	WINF("Freescale, Inc. - Gen II.c waveform file parser - build 20141216\n");
	for ( i = 1; i < argc; ++i )
	{
		if ( !strcmp(argv[i], "-h") || !strcmp(argv[i], "--help") )
		{
			usage(argv[0]);
			return 1;
		}
		if ( !strcmp(argv[i], "-o2") )
		{
			if ( (i + 1) >= argc )
			{
				usage(argv[0]);
				return 1;
			}
			out_filename = argv[++i];
			o_type = 3;
		}
		if ( !strcmp(argv[i], "-o1") )
		{
			if ( (i + 1) >= argc )
			{
				usage(argv[0]);
				return 1;
			}
			out_filename = argv[++i];
			o_type = 2;
		}
		else if ( !strcmp(argv[i], "-o") )
		{
			if ( (i + 1) >= argc )
			{
				usage(argv[0]);
				return 1;
			}
			out_filename = argv[++i];
		}
		else if ( !strcmp(argv[i], "-va") )
		{
			verbose = 2;
		}
		else if ( !strcmp(argv[i], "-v") )
		{
			verbose = 1;
		}
		else
		{
			wbf_filename = argv[i];
		}
	}

	return wbf_convert_func(o_type, verbose);
}

#endif


/* ------------------ converter --------------------- */

int32_t wbf_mc;
int32_t wbf_trc;
uint8_t checkSum;

static int32_t getData(int32_t len_1, uint32_t *result_ptr)
{
	int32_t cbyte; // ST1C_4@3
	bool gd_more_bytes; // al@5
	uint32_t word; // [sp+10h] [bp-18h]@1
	int32_t i; // [sp+18h] [bp-10h]@1

	word = 0;
	for ( i = 1; ; i <<= 8 )
	{
		gd_more_bytes = len_1-- != 0;
		if ( !gd_more_bytes )
			break;
		if ( wbf_feof() )
			return 1;
		cbyte = wbf_fgetc();
		checkSum = (uint8_t)(cbyte + checkSum);
		word += cbyte * i;
	}
	*result_ptr = word;
	return 0;
}

static int32_t getOffset(uint32_t *off_ptr)
{
	int32_t result; // eax@3
	int32_t temp3; // [sp+18h] [bp-10h]@1
	uint8_t cs_byte; // [sp+1Fh] [bp-9h]@2

	checkSum = 0;
	getData(3, &temp3);
	*off_ptr = temp3;
	if ( wbf_feof() )
	{
		result = 1;
	}
	else
	{
		cs_byte = wbf_fgetc();
		if ( cs_byte == checkSum )
		{
			result = 0;
		}
		else
		{
			WERR("Error getting offset address, checksum error!\n");
			result = 2;
		}
	}
	return result;
}

static void closeStreams()
{
	uint32_t j; // [sp+14h] [bp-14h]@5
	uint32_t i; // [sp+18h] [bp-10h]@4

	wdff_fclose();
	out_fclose();
	wbf_fclose();
	if (wdff_fopen_r(0, 0))
	{
		wdff_fclose();
		for ( i = 0; ; ++i )
		{
			if ( i >= wbf_mc )
				break;
			for ( j = 0; j < wbf_trc; ++j )
			{
				wdff_remove(i, j);
			}
		}
	}
}

static int wbf_convert_func(int o_type, int verbose)
{
	int32_t _wf_page_size; // eax@90
	uint32_t wf_luts_bits_54; // eax@92
	uint8_t *out_ptr; // ecx@174
	bool is_more_data; // al@285
	uint8_t *ptrptr; // ebx@293
	int64_t dataptr; // eax@293
	int32_t _unused3; // et1@302
	uint8_t *var12; // eax@307
	int32_t var13; // et1@307
	uint8_t *var14; // ecx@309
	uint8_t *var15; // eax@309
	int32_t var16; // edx@309
	bool is_more_bytes; // al@354
	int32_t temp5; // eax@375
	size_t nread; // eax@377
	int32_t aoff; // eax@395
	int32_t step1; // eax@399
	uint32_t rec_size2; // eax@403
	int32_t step2; // eax@407
	uint32_t n_max; // eax@411
	uint32_t m_max; // eax@416
	uint64_t pos2; // eax@436
	int32_t buffer_pos; // eax@438
	int32_t _unused7; // et1@438
	int32_t aa_byte; // eax@569
	uint32_t current_pos; // eax@572
	int32_t xwi_byte; // eax@608
	bool is_more_xwia; // al@611
	int32_t end_of_xwi; // eax@613
	uint64_t current_xwi_ptr[2]; // [sp+28h] [bp-128h]@1
	uint32_t _unused1; // [sp+30h] [bp-120h]@1
	int32_t _unused2; // [sp+34h] [bp-11Ch]@1
	int64_t f_temp; // [sp+38h] [bp-118h]@1
	int32_t unused6; // [sp+40h] [bp-110h]@171
	int32_t unused5; // [sp+44h] [bp-10Ch]@171
	uint32_t kk; // [sp+4Ch] [bp-104h]@622
	uint32_t jj; // [sp+50h] [bp-100h]@621
	int32_t xwia_count; // [sp+58h] [bp-F8h]@595
	int32_t rec_size1; // [sp+5Ch] [bp-F4h]@377
	int32_t temp4; // [sp+60h] [bp-F0h]@387
	uint32_t ii; // [sp+64h] [bp-ECh]@386
	uint32_t n; // [sp+68h] [bp-E8h]@385
	uint32_t m; // [sp+6Ch] [bp-E4h]@384
	void *page_memory; // [sp+70h] [bp-E0h]@373
	int32_t unused_counter; // [sp+74h] [bp-DCh]@373
	uint32_t l; // [sp+78h] [bp-D8h]@373
	int32_t temp_counter; // [sp+7Ch] [bp-D4h]@339
	int32_t c; // [sp+80h] [bp-D0h]@327
	int32_t temp_byte=0; // [sp+84h] [bp-CCh]@277
	uint32_t k; // [sp+88h] [bp-C8h]@305
	uint32_t cindex; // [sp+8Ch] [bp-C4h]@293
	uint32_t frame_count; // [sp+90h] [bp-C0h]@373
	int32_t _flag3; // [sp+94h] [bp-BCh]@293
	int32_t _flag2; // [sp+98h] [bp-B8h]@293
	uint32_t frame_data_size; // [sp+9Ch] [bp-B4h]@293
	int32_t _flag1; // [sp+A0h] [bp-B0h]@293
	int32_t vc_len; // [sp+A4h] [bp-ACh]@270
	char *vc_ptr; // [sp+A8h] [bp-A8h]@270
	int32_t current_tt_offset; // [sp+ACh] [bp-A4h]@260
	int32_t tt_offset; // [sp+B0h] [bp-A0h]@227
	uint64_t max_pos; // [sp+B8h] [bp-98h]@1
	void *aa_table; // [sp+BCh] [bp-94h]@1
	void *vc_table; // [sp+C0h] [bp-90h]@1
	int64_t last_pos; // [sp+C4h] [bp-8Ch]@438
	int32_t wtt_pos; // [sp+C8h] [bp-88h]@217
	int32_t current_wtt_pos; // [sp+CCh] [bp-84h]@210
	int32_t pos; // [sp+D0h] [bp-80h]@171
	int32_t xwia_ptr_pos; // [sp+D4h] [bp-7Ch]@196
	int32_t aa_ptr_pos; // [sp+D8h] [bp-78h]@187
	int32_t vc_ptr_pos; // [sp+DCh] [bp-74h]@180
	uint32_t fw_length; // [sp+E0h] [bp-70h]@47
	size_t aa_table_size; // [sp+E4h] [bp-6Ch]@187
	size_t vc_size; // [sp+E8h] [bp-68h]@180
	size_t ptr_table_size; // [sp+ECh] [bp-64h]@171
	size_t mode_table_size; // [sp+F0h] [bp-60h]@171
	uint32_t temp2; // [sp+F4h] [bp-5Ch]@43
	void *ptr_table; // [sp+F8h] [bp-58h]@171
	char *mode_table; // [sp+FCh] [bp-54h]@171
	int32_t xwia; // [sp+100h] [bp-50h]@56
	int32_t xwia_exists; // [sp+104h] [bp-4Ch]@56
	int32_t awv_bits_21; // [sp+108h] [bp-48h]@123
	int32_t awv_bits_10; // [sp+10Ch] [bp-44h]@123
	uint32_t wbf_sb; // [sp+110h] [bp-40h]@135
	uint32_t wbf_eb; // [sp+114h] [bp-3Ch]@129
	int32_t wf_parm_from_luts_54; // [sp+118h] [bp-38h]@97
	uint32_t wf_page_size; // [sp+11Ch] [bp-34h]@92
	int32_t luts_bits_10; // [sp+120h] [bp-30h]@89
	uint32_t wmta; // [sp+124h] [bp-2Ch]@70
	void *buffer; // [sp+128h] [bp-28h]@173
	uint32_t j; // [sp+12Ch] [bp-24h]@236
	uint32_t i; // [sp+130h] [bp-20h]@1

	f_temp = 0LL;
	current_xwi_ptr[0] = current_xwi_ptr[1] = 0LL;
	_unused1 = 0;
	_unused2 = 0;
	vc_table = 0;
	aa_table = 0;
	max_pos = 0;

	if (! wbf_fopen())
	{
		WERR("Cannot open the input stream!\n");
		return 30;
	}
	if (! out_fopen())
	{
		WERR("Cannot open firmware file!\n");
		wbf_fclose();
		return 31;
	}
	if ( 1 )
	{
		for ( i = 0; i <= 6; ++i )			// first 7 words are copied directly
		{
			if ( getData(4, &temp2) )
			{
				closeStreams();
				return 40;
			}
			if ( i == 1 )
			{
				fw_length = temp2;
				checkSum = 0;
			}
			if ( out_fwrite(&temp2, 4u) != 1 )
			{
				closeStreams();
				return 50;
			}
		}
		if ( getData(3, &temp2) )									 // XWIA:3
		{
			WERR("get wxia from input stream error!\n");
			closeStreams();
			return 41;
		}
		xwia = temp2;
		xwia_exists = temp2 != 0;
		if ( out_fwrite(&temp2, 3u) != 1 )
		{
			WERR("write wxia to file header error!\n");
			closeStreams();
			return 51;
		}
		checkSum = -checkSum;
		if ( getData(1, &temp2) )
		{
			WERR("get checksum1 from input stream error!\n");
			return 42;
		}
		if ( out_fwrite(&temp2, 1u) != 1 )
		{
			WERR("write file header error!\n");
			closeStreams();
			return 52 ;
		}
		if ( checkSum )
		{
			WERR("Warning: Checksum 1 error!\n");
			checkSum = 0;
		}
		if ( getData(3, &temp2) )									 // WMTA:3
		{
			WERR("get wmta from input stream error!\n");
			return 43;
		}
		wmta = temp2;
		if ( verbose )
			WDBG(" WMTA : 0x%04x\n", wmta);
		if ( out_fwrite(&temp2, 3u) != 1 )
		{
			WERR("write wmta to file header error!\n");
			closeStreams();
			return 53;
		}
		if ( getData(1, &temp2) )									 // FVSN:1
		{
			WERR("get fvsn from input stream error!\n");
			return 44;
		}
		if ( out_fwrite(&temp2, 1u) != 1 )
		{
			WERR("write fvsn to file header error!\n");
			closeStreams();
			return 54;
		}
		if ( getData(1, &temp2) )									 // LUTS:1
		{
			WERR("get luts from input stream error!\n");
			return 45;
		}
		if ( out_fwrite(&temp2, 1u) != 1 )
		{
			WERR("write luts to file header error!\n");
			closeStreams();
			return 55;
		}

		if ( verbose )
			WDBG(" LUTS : 0x%04x\n", temp2);
		luts_bits_10 = temp2 & 3;
		if ( (temp2 >> 2) & 3 )
			_wf_page_size = 1024;
		else
			_wf_page_size = 256;
		wf_page_size = _wf_page_size;
		wf_luts_bits_54 = (temp2 >> 4) & 3;
		if ( wf_luts_bits_54 == 1 )
		{
			wf_parm_from_luts_54 = 1024;
		}
		else if ( wf_luts_bits_54 < 1 )
		{
			wf_parm_from_luts_54 = 256;
		}
		else if ( wf_luts_bits_54 == 2 )
		{
			wf_parm_from_luts_54 = 2048;
		}
		else if ( wf_luts_bits_54 == 3 )
		{
			wf_parm_from_luts_54 = 4096;
		}
		if ( getData(1, &temp2) )									 // MC:1
		{
			WERR("get mode count from input stream error!\n");
			return 46;
		}
		if ( out_fwrite(&temp2, 1u) != 1 )
		{
			WERR("write mode count to file header error!\n");
			closeStreams();
			return 56;
		}
		wbf_mc = temp2 + 1;
		if ( verbose )
			WDBG(" MC : 0x%02x\n", wbf_mc);
		if ( getData(1, &temp2) )									 // TRC:1
		{
			WERR("get temperature range count from input stream error!\n");
			return 47;
		}
		if ( out_fwrite(&temp2, 1u) != 1 )
		{
			WERR("write trc to file header error!\n");
			closeStreams();
			return 57;
		}
		wbf_trc = temp2 + 1;
		if ( verbose )
			WDBG(" TRC : 0x%02x\n", wbf_trc);
		if ( getData(1, &temp2) )									 // AWV:1
		{
			WERR("get advance waveform flags from input stream error!\n");
			return 48;
		}
		if ( out_fwrite(&temp2, 1u) != 1 )
		{
			WERR("write awv to file header error!\n");
			closeStreams();
			return 58;
		}
		awv_bits_10 = temp2 & 3;
		awv_bits_21 = (temp2 >> 1) & 3;
		if ( getData(1, &temp2) )									 // EB:1
		{
			WERR("get end byte from input stream error!\n");
			return 49;
		}
		if ( out_fwrite(&temp2, 1u) != 1 )
		{
			WERR("write eb to file header error!\n");
			closeStreams();
			return 59;
		}
		wbf_eb = temp2;
		if ( getData(1, &temp2) )									 // SB:1
		{
			WERR("get start byte from input stream error!\n");
			return 61;
		}
		if ( out_fwrite(&temp2, 1u) != 1 )
		{
			WERR("write awv to file header error!\n");
			closeStreams();
			return 71;
		}
		wbf_sb = temp2;
		if ( getData(4, &temp2) )									 // AWFLAGS:5
		{
			WERR("get advance waveform flags from input stream error!\n");
			return 62;
		}
		if ( out_fwrite(&temp2, 4u) != 1 )
		{
			WERR("write awv to file header error!\n");
			closeStreams();
			return 72;
		}
		if ( getData(1, &temp2) )									 // AWFLAGS:5
		{
			WERR("get advance waveform flags from input stream error!\n");
			return 63;
		}
		if ( out_fwrite(&temp2, 1u) != 1 )
		{
			WERR("write awv to file header error!\n");
			closeStreams();
			return 73;
		}
		checkSum = -checkSum;
		if ( getData(1, &temp2) )									 // checksum
		{
			WERR("get checksum2 from input stream error!\n");
			return 64;
		}
		if ( out_fwrite(&temp2, 1u) != 1 )
		{
			WERR("write file header error!\n");
			closeStreams();
			return 74;
		}
		if ( checkSum )
		{
			WERR("Checksum 2 error!\n");
			closeStreams();
			return 65;
		}
		if ( verbose )
			WDBG(" Temperature ranges : ");
		for ( i = 0; i < wbf_trc + 1; ++i )
		{
			if ( getData(1, &temp2) )
			{
				WERR("get temp ranges from input stream error!\n");
				return 66;
			}
			if ( out_fwrite(&temp2, 1u) != 1 )
			{
				WERR("write temp ranges to file header error!\n");
				closeStreams();
				return 76;
			}
			if ( verbose )
				WDBG("%d ", (uint8_t)temp2);
		}
		if ( verbose )
			WDBG("\n");
		checkSum = -checkSum;
		if ( getData(1, &temp2) )
		{
			WERR("get temp range checksum from input stream error!\n");
			return 67;
		}
		if ( checkSum )
		{
			WERR("TRC Checksum error!\n");
			closeStreams();
			return 80;
		}
		pos = out_ftell();
		mode_table_size = 8 * wbf_mc;
		ptr_table_size = 8 * wbf_mc * wbf_trc;
		mode_table = (char *)wbf_malloc(8 * wbf_mc);
		ptr_table = wbf_malloc(ptr_table_size);
		unused6 = mode_table_size + ptr_table_size;
		unused5 = 0;
		if ( verbose == 2 )
			WDBG(" fw wmt position 0x%04lx\n", pos);
		buffer = wbf_malloc(12 * wbf_trc * wbf_mc);
		for ( i = 0; i < wbf_mc; ++i )
		{
			out_ptr = &mode_table[8 * i];
			*(uint32_t *)out_ptr = 8 * wbf_trc * i;
			*(uint32_t *)(out_ptr + 4) = 0;
		}
		if ( out_fwrite(mode_table, mode_table_size) != 1 )// write mode table
		{
			WERR("write wmta to file header error!\n");
		wbf_free(buffer);
		wbf_free(mode_table);
		wbf_free(ptr_table);
			closeStreams();
			return 10;
		}
		if ( awv_bits_10 )
		{
			vc_ptr_pos = out_ftell();
			vc_size = 16 * wbf_mc * wbf_trc;
			vc_table = wbf_malloc(16 * wbf_mc * wbf_trc);
			if ( out_fwrite(current_xwi_ptr, 8u) != 1 )// zeroes?
			{
				WERR("write vc data pointer error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( vc_table )
				wbf_free(vc_table);
				closeStreams();
				return 78;
			}
		}
		else
		{
			vc_ptr_pos = 0;
		}
		if ( awv_bits_21 )
		{
			aa_ptr_pos = out_ftell();
			aa_table_size = 4 * wbf_mc * wbf_trc;
			aa_table = wbf_malloc(4 * wbf_mc * wbf_trc);
			if ( out_fwrite(&current_xwi_ptr[0], 8u) != 1 || out_fwrite(&current_xwi_ptr[1], 8u) != 1 ) // zeroes?
			{
				WERR("write aa pointer error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( vc_table )
				wbf_free(vc_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 79;
			}
		}
		else
		{
			aa_ptr_pos = 0;
		}
		if ( xwia_exists )
		{
			xwia_ptr_pos = out_ftell();
			if ( out_fwrite(current_xwi_ptr, 8u) != 1 )// zeroes?
			{
				WERR("write xwi pointer error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( vc_table )
				wbf_free(vc_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 90;
			}
		}
		else
		{
			xwia_ptr_pos = 0;
		}
		if ( verbose == 2 )
			WDBG(" fw awv vc position 0x%04lx\n", vc_ptr_pos);
		if ( verbose == 2 )
			WDBG(" fw awv aa position 0x%04lx\n", aa_ptr_pos);
		if ( verbose == 2 )
			WDBG(" fw awv xwi position 0x%04lx\n", xwia_ptr_pos);
		current_wtt_pos = out_ftell();
		if ( out_fwrite(ptr_table, ptr_table_size) != 1 )
		{
			WERR("write wmta to file header error!\n");
		wbf_free(buffer);
		wbf_free(mode_table);
		wbf_free(ptr_table);
			if ( vc_table )
			wbf_free(vc_table);
			if ( aa_table )
			wbf_free(aa_table);
			closeStreams();
			return 11;
		}
		wtt_pos = out_ftell();
		if ( verbose == 2 )
			WDBG(" fw wtt position 0x%04lx\n", current_wtt_pos);
		for ( i = 0; i < wbf_mc; ++i )
		{
			if ( wbf_fseek(wmta + 4 * i) )
			{
				WERR("Seek to waveform mode table error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( vc_table )
				wbf_free(vc_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 7;
			}
			if ( getOffset(&tt_offset) )
			{
				WERR("get waveform temperatur table offset error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( vc_table )
				wbf_free(vc_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 92;
			}
			if ( verbose )
				WDBG("WTTA for mode %d is: 0x%08x \n", i, tt_offset);
			for ( j = 0; j < wbf_trc; ++j )
			{
				if ( awv_bits_21 )
				{
					if ( wbf_fseek(4 * (j + wbf_trc) + tt_offset) )
					{
						WERR(" ** Seek to aa pointer table error!\n");
					wbf_free(buffer);
					wbf_free(mode_table);
					wbf_free(ptr_table);
						if ( vc_table )
						wbf_free(vc_table);
						if ( aa_table )
						wbf_free(aa_table);
						closeStreams();
						return 8;
					}
					if ( getData(4, &temp2) )
					{
						WERR("get waveform aa offset error!\n");
					wbf_free(buffer);
					wbf_free(mode_table);
					wbf_free(ptr_table);
						if ( vc_table )
						wbf_free(vc_table);
						if ( aa_table )
						wbf_free(aa_table);
						closeStreams();
						return 93;
					}
					*((uint32_t *)aa_table + wbf_trc * i + j) = temp2;
				}
				if ( wbf_fseek(4 * j + tt_offset) )
				{
					WERR(" ** Seek to waveform temperature table error!\n");
				wbf_free(buffer);
				wbf_free(mode_table);
				wbf_free(ptr_table);
					if ( vc_table )
					wbf_free(vc_table);
					if ( aa_table )
					wbf_free(aa_table);
					closeStreams();
					return 9;
				}
				if ( getOffset(&current_tt_offset) )
				{
					WERR("get waveform temperatur table offset error!\n");
				wbf_free(buffer);
				wbf_free(mode_table);
				wbf_free(ptr_table);
					if ( vc_table )
					wbf_free(vc_table);
					if ( aa_table )
					wbf_free(aa_table);
					closeStreams();
					return 94;
				}
				if ( verbose )
					WDBG("WTA for mode %d, range %d is: 0x%08x \n", i, j, current_tt_offset);
				if ( vc_table )
				{
					vc_ptr = (char *)vc_table + 16 * (wbf_trc * i + j);
					vc_len = 16;
					if ( wbf_fseek(current_tt_offset - 16) )
					{
						WERR(" ** Seek to waveform data error!\n");
					wbf_free(buffer);
					wbf_free(mode_table);
					wbf_free(ptr_table);
						if ( vc_table )
						wbf_free(vc_table);
						if ( aa_table )
						wbf_free(aa_table);
						closeStreams();
						return 12;
					}
					while ( 1 )
					{
						is_more_data = vc_len-- != 0;
						if ( !is_more_data )
							break;
						if ( getData(1, &temp_byte) )
						{
							WERR("get vc data from input stream error!\n");
						wbf_free(buffer);
						wbf_free(mode_table);
						wbf_free(ptr_table);
							if ( vc_table )
							wbf_free(vc_table);
							if ( aa_table )
							wbf_free(aa_table);
							closeStreams();
							return 95;
						}
						*vc_ptr++ = (char)temp_byte;
					}
				}
				if ( wbf_fseek(current_tt_offset) )
				{
					WERR(" ** Seek to waveform data error!\n");
				wbf_free(buffer);
				wbf_free(mode_table);
				wbf_free(ptr_table);
					if ( vc_table )
					wbf_free(vc_table);
					if ( aa_table )
					wbf_free(aa_table);
					closeStreams();
					return 8;
				}
				_flag1 = 1;
				frame_data_size = 0;
				_flag2 = 1;
				_flag3 = 0;
				cindex = wbf_trc * i + j;
				f_temp = 0LL;
				*((uint32_t *)buffer + 3 * cindex) = current_tt_offset;
				ptrptr = (uint8_t *)buffer + 12 * cindex;
				dataptr = out_ftell();
				*(uint32_t *)(ptrptr + 4) = dataptr;
				*(uint32_t *)(ptrptr + 8) = dataptr >> 31;
				if ( o_type > 2 )
				{
					if (! wdff_fopen_w(i, j))
					{
						WERR("Cannot open temp firmware file!\n");
						wbf_free(buffer);
						wbf_free(mode_table);
						wbf_free(ptr_table);
						if ( vc_table )
						wbf_free(vc_table);
						if ( aa_table )
						wbf_free(aa_table);
						closeStreams();
						return 14;
					}
				}
				if ( verbose )
				{
					_unused3 = *((uint32_t *)buffer + 3 * cindex + 2);
					WDBG("luas[%d] : 0x%04llx\n", cindex, *(uint64_t *)((char *)buffer + 12 * cindex));
				}
				if ( i > 0 || j > 0 )
				{
					for ( k = 0; ; ++k )
					{
						if ( k >= cindex )
							goto end_of_table;
						if ( verbose == 2 )
						{
							var12 = (uint8_t *)buffer + 12 * k;
							var13 = *(uint32_t *)(var12 + 8);
							WDBG(
								"wta: 0x%04x, looking up %d : 0x%04x / 0x%04x\n",
								current_tt_offset,
								k,
								*((uint32_t *)buffer + 3 * k),
								*(uint32_t *)(var12 + 4));
						}
						if ( *((uint32_t *)buffer + 3 * k) == current_tt_offset )
							break;
					}
					*((uint32_t *)buffer + 3 * cindex) = current_tt_offset;
					var14 = (uint8_t *)buffer + 12 * cindex;
					var15 = (uint8_t *)buffer + 12 * k;
					var16 = *(uint32_t *)(var15 + 8);
					*(uint32_t *)(var14 + 4) = *(uint32_t *)(var15 + 4);
					*(uint32_t *)(var14 + 8) = var16;
					_flag3 = 1;
				}
end_of_table:
				if ( !_flag3 )
				{
					if ( out_fwrite(&f_temp, 8u) != 1 )
					{
						WERR("writing out dummy frame count error!\n");
						wbf_free(buffer);
						wbf_free(mode_table);
						wbf_free(ptr_table);
						if ( vc_table )
							wbf_free(vc_table);
						if ( aa_table )
							wbf_free(aa_table);
						closeStreams();
						return 15;
					}
					checkSum = 0;
					while ( _flag2 )
					{
						if ( getData(1, &temp_byte) )
						{
							WERR("get byte value from input stream error!\n");
							wbf_free(buffer);
							wbf_free(mode_table);
							wbf_free(ptr_table);
							if ( vc_table )
								wbf_free(vc_table);
							closeStreams();
							if ( aa_table )
								wbf_free(aa_table);
							return 96;
						}
						c = *(uint32_t *)&temp_byte;
						if ( *(uint32_t *)&temp_byte == wbf_eb )
						{
							_flag2 = 0;
						}
						else if ( c == wbf_sb )
						{
							_flag1 = _flag1 == 0;
						}
						else if ( _flag1 )
						{
							if ( getData(1, &temp_byte) )
							{
								WERR("get byte value from input stream error!\n");
								wbf_free(buffer);
								wbf_free(mode_table);
								wbf_free(ptr_table);
								if ( aa_table )
									wbf_free(aa_table);
								if ( vc_table )
									wbf_free(vc_table);
								closeStreams();
								out_fclose();
								return 97;
							}
							temp_counter = *(uint32_t *)&temp_byte + 1;
							while ( 1 )
							{
								is_more_bytes = temp_counter-- != 0;
								if ( !is_more_bytes )
									break;
								if ( luts_bits_10 )
								{
									if ( o_type == 1 )
									{
										out_fputc(c & 0xF);
										out_fputc((uint8_t)c >> 4);
									}
									else
									{
										if ( o_type == 2 )
												out_fputc(c);
											wdff_fputc(c & 0xF);
											wdff_fputc((uint8_t)c >> 4);
									}
									frame_data_size += 2;
								}
								else
								{
									if ( o_type == 1 )
									{
										out_fputc(c & 3);
										out_fputc(((uint32_t)c >> 2) & 3);
										out_fputc(((uint32_t)c >> 4) & 3);
										out_fputc((uint8_t)c >> 6);
									}
									else if ( o_type == 2 )
									{
										out_fputc((c & 3) | (4 * (uint8_t)c & 0x30));
										out_fputc((((uint32_t)c >> 4) & 3) | (((uint32_t)c >> 2) & 0x30));
									}
									else
									{
										wdff_fputc(c & 3);
										wdff_fputc(((uint32_t)c >> 2) & 3);
										wdff_fputc(((uint32_t)c >> 4) & 3);
										wdff_fputc((uint8_t)c >> 6);
									}
									frame_data_size += 4;
								}
							}
						}
						else if ( luts_bits_10 )
						{
							if ( o_type == 1 )
							{
								out_fputc(c & 0xF);
								out_fputc((uint8_t)c >> 4);
							}
							else if ( o_type == 2 )
							{
								out_fputc(c);
							}
							else
							{
								wdff_fputc(c & 0xF);
								wdff_fputc((uint8_t)c >> 4);
							}
							frame_data_size += 2;
						}
						else
						{
							if ( o_type == 1 )
							{
								out_fputc(c & 3);
								out_fputc(((uint32_t)c >> 2) & 3);
								out_fputc(((uint32_t)c >> 4) & 3);
								out_fputc((uint8_t)c >> 6);
							}
							else if ( o_type == 2 )
							{
								out_fputc((c & 3) | (4 * (uint8_t)c & 0x30));
								out_fputc((((uint32_t)c >> 4) & 3) | (((uint32_t)c >> 2) & 0x30));
							}
							else
							{
								wdff_fputc(c & 3);
								wdff_fputc(((uint32_t)c >> 2) & 3);
								wdff_fputc(((uint32_t)c >> 4) & 3);
								wdff_fputc((uint8_t)c >> 6);
							}
							frame_data_size += 4;
						}
					}
					if ( verbose )
						WDBG("Byte count: %d\n", frame_data_size);
					if ( o_type > 2 )
					{
						unused_counter = 0;
						frame_count = frame_data_size / wf_page_size;
						wdff_fclose();
						wdff_fopen_r(i, j);
						wdff_fseek(0);
						page_memory = wbf_malloc(wf_page_size);
						for ( l = 0; l < frame_count; ++l )
						{
							if ( wf_page_size == 1024 )
								temp5 = 32;
							else
								temp5 = 16;
							rec_size1 = temp5;
							nread = wdff_fread(page_memory, wf_page_size);
							if ( nread != 1 )
							{
								WDBG("*** Read temp file error! (%d) ****", l);
							wbf_free(buffer);
							wbf_free(mode_table);
							wbf_free(ptr_table);
								if ( vc_table )
								wbf_free(vc_table);
								if ( aa_table )
								wbf_free(aa_table);
								closeStreams();
								return 6;
							}
							for ( m = 0; ; ++m )
							{
								m_max = wf_page_size == 1024 ? 4 : 1;
								if ( m_max <= m )
									break;
								for ( n = 0; ; n += step2 )
								{
									n_max = wf_page_size == 1024 ? 32 : 16;
									if ( n_max <= n )
										break;
									for ( ii = 0; ; ii += step1 )
									{
										rec_size2 = wf_page_size == 1024 ? 32 : 16;
										if ( rec_size2 <= ii )
											break;
										temp4 = *((uint8_t *)page_memory + ii + rec_size1 * n) & 3;
										if ( n != 30 || ii != 28 || (m != 1 && m != 2) )
										{
											if ( wf_page_size == 1024 )
												aoff = 2;
											else
												aoff = 1;
											temp4 |= 16 * (*((uint8_t *)page_memory + ii + rec_size1 * n + aoff) & 3);
										}
										else if ( m == 1 )
										{
											temp4 |= 16 * (*((uint8_t *)page_memory + 1023) & 3);
										}
										else
										{
											temp4 |= 16 * (*((uint8_t *)page_memory + 957) & 3);
										}
										out_fputc(temp4);
										++unused_counter;
										if ( wf_page_size == 1024 )
											step1 = 4;
										else
											step1 = 2;
									}
									if ( wf_page_size == 1024 )
										step2 = 2;
									else
										step2 = 1;
								}
							}
						}
						wdff_fclose();
						wbf_free(page_memory);
					}
					checkSum = -checkSum;
					if ( getData(1, &temp2) )
					{
						WERR("get waveform data checksum byte from input stream error!\n");
					wbf_free(buffer);
					wbf_free(mode_table);
					wbf_free(ptr_table);
						if ( vc_table )
						wbf_free(vc_table);
						if ( aa_table )
						wbf_free(aa_table);
						closeStreams();
						return 4;
					}
					if ( checkSum )
					{
						WERR("waveform data field has Checksum error!\n");
					wbf_free(buffer);
					wbf_free(mode_table);
					wbf_free(ptr_table);
						if ( vc_table )
						wbf_free(vc_table);
						if ( aa_table )
						wbf_free(aa_table);
						closeStreams();
						return 22;
					}
					pos2 = wbf_ftell();
					if ( pos2 > max_pos )
						max_pos = wbf_ftell();
					f_temp = frame_data_size / wf_page_size;// frame count
					last_pos = out_ftell();
					if ( out_fseek(*(uint32_t *)((uint8_t *)buffer + 12 * cindex + 4)) )
					{
						WERR(" ** rewind error!\n");
					wbf_free(buffer);
					wbf_free(mode_table);
					wbf_free(ptr_table);
						if ( aa_table )
						wbf_free(aa_table);
						if ( vc_table )
						wbf_free(vc_table);
						closeStreams();
						return 24;
					}
					if ( out_fwrite(&f_temp, 8u) != 1 )
					{
						WERR(" ** writing out final frame count error!\n");
					wbf_free(buffer);
					wbf_free(mode_table);
					wbf_free(ptr_table);
						if ( vc_table )
						wbf_free(vc_table);
						if ( aa_table )
						wbf_free(aa_table);
						closeStreams();
						return 25;
					}
					if ( out_fseek(last_pos) )
					{
						WERR(" ** file forward to next wtt error!\n");
					wbf_free(buffer);
					wbf_free(mode_table);
					wbf_free(ptr_table);
						if ( vc_table )
						wbf_free(vc_table);
						if ( aa_table )
						wbf_free(aa_table);
						closeStreams();
						return 26;
					}
				}
			}
		}
		last_pos = out_ftell();
		if ( out_fseek(current_wtt_pos) )
		{
			WERR(" ** Seek to wtt position error!\n");
		wbf_free(buffer);
		wbf_free(mode_table);
		wbf_free(ptr_table);
			if ( vc_table )
			wbf_free(vc_table);
			if ( aa_table )
			wbf_free(aa_table);
			closeStreams();
			return 27;
		}
		for ( i = 0; i < wbf_mc; ++i )
		{
			for ( j = 0; j < wbf_trc; ++j )
			{
				f_temp = *(uint64_t *)((char *)buffer + 12 * (j + wbf_trc * i) + 4) - (uint32_t)pos;
				if ( verbose == 2 )
					WDBG("wtt[%d,%d] = 0x%08llx\n", i, j, f_temp);
				if ( out_fwrite(&f_temp, 8u) != 1 )
				{
					WERR(" ** writing out final wtt error!\n");
				wbf_free(buffer);
				wbf_free(mode_table);
				wbf_free(ptr_table);
					if ( vc_table )
					wbf_free(vc_table);
					if ( aa_table )
					wbf_free(aa_table);
					closeStreams();
					return 28;
				}
			}
		}
		if ( out_fseek(pos) )
		{
			WERR(" ** Seek to wmt position error!\n");
		wbf_free(buffer);
		wbf_free(mode_table);
		wbf_free(ptr_table);
			if ( vc_table )
			wbf_free(vc_table);
			if ( aa_table )
			wbf_free(aa_table);
			closeStreams();
			return 8;
		}
		for ( i = 0; i < wbf_mc; ++i )
		{
			f_temp = current_wtt_pos - pos + 8 * i * wbf_trc;
			if ( verbose == 2 )
				WDBG("wmt[%d] = 0x%08llx\n", i, f_temp);
			if ( out_fwrite(&f_temp, 8u) != 1 )
			{
				WERR(" ** writing out final wmt error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( vc_table )
				wbf_free(vc_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 29;
			}
		}
		if ( verbose == 2 )
			WDBG("Advance waveform : 0x%08llx\n", last_pos);
		if ( vc_table )
		{
			if ( verbose )
				WDBG("VC offset : 0x%08lx\n", vc_ptr_pos);
			if ( out_fseek(vc_ptr_pos) )
			{
				WERR(" ** Seek to VC Data offset position error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( vc_table )
				wbf_free(vc_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 30;
			}
			current_xwi_ptr[0] = (uint32_t)last_pos - (uint32_t)pos;
			if ( out_fwrite(current_xwi_ptr, 8u) != 1 )
			{
				WERR("write vc data pointer error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( vc_table )
				wbf_free(vc_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 31;
			}
			if ( out_fseek(last_pos) )
			{
				WERR(" ** Seek to last position error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( vc_table )
				wbf_free(vc_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 32;
			}
			if ( out_fwrite(vc_table, vc_size) != 1 )
			{
				WERR(" ** writing out VC Data error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( vc_table )
				wbf_free(vc_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 33;
			}
			last_pos = out_ftell();
			if ( verbose )
				WDBG("End of VC data : 0x%08llx\n", last_pos);
		wbf_free(vc_table);
		}
		if ( aa_table )
		{
			if ( out_fseek(aa_ptr_pos) )
			{
				WERR(" ** Seek to aa pointer position error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 34;
			}
			current_xwi_ptr[0] = (uint32_t)last_pos - (uint32_t)pos;
			current_xwi_ptr[1] = max_pos;
			_unused2 = 0;
			if ( verbose == 2 )
			{
				WDBG("aa fw offset : 0x%08llx\n", last_pos);
				WDBG("magic number : 0x%08llx\n", max_pos);
			}
			if ( out_fwrite(&current_xwi_ptr[0], 8u) != 1 || out_fwrite(&current_xwi_ptr[1], 8u) != 1 )
			{
				WERR("write aa data pointer error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 35;
			}
			if ( out_fseek(last_pos) )
			{
				WERR(" ** Seek to last position error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 36;
			}
			if ( out_fwrite(aa_table, aa_table_size) != 1 )
			{
				WERR(" ** writing out aa pointer error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 37;
			}
			last_pos = out_ftell();
		wbf_free(aa_table);
			if ( wbf_fseek(max_pos) )
			{
				WERR(" ** Seek to aa data error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				closeStreams();
				return 38;
			}
			if ( out_fseek(last_pos) )
			{
				WERR(" ** Seek to last fw position error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				closeStreams();
				return 39;
			}
			while ( 1 )
			{
				current_pos = wbf_ftell();
				if ( current_pos >= fw_length )
					break;
				aa_byte = wbf_fgetc();
				if ( out_fputc(aa_byte) == -1 )
				{
					WERR("write aa data error!\n");
				wbf_free(buffer);
				wbf_free(mode_table);
				wbf_free(ptr_table);
					closeStreams();
					return 40;
				}
			}
			last_pos = out_ftell();
			if ( verbose )
				WDBG("End of aa data : 0x%08llx\n", last_pos);
		}
		if ( xwia_exists )
		{
			if ( verbose )
				WDBG("XWIA offset : 0x%08lx\n", xwia_ptr_pos);
			if ( out_fseek(xwia_ptr_pos) )
			{
				WERR(" ** Seek to xwi offset position error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( vc_table )
				wbf_free(vc_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 41;
			}
			current_xwi_ptr[0] = (uint32_t)last_pos - (uint32_t)pos;
			if ( out_fwrite(current_xwi_ptr, 8u) != 1 )
			{
				WERR("write xwi pointer error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( vc_table )
				wbf_free(vc_table);
				if ( aa_table )
				wbf_free(aa_table);
				closeStreams();
				return 42;
			}
			if ( wbf_fseek(xwia) )
			{
				WERR(" ** Seek to xwia data error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				closeStreams();
				return 43;
			}
			xwia_count = wbf_fgetc();
			if ( verbose == 2 )
				WDBG(" xwia count : %d\n", xwia_count);
			if ( verbose == 2 )
				WDBG(" XWI begin : 0x%08llx\n", last_pos);
			if ( out_fseek(last_pos) )
			{
				WERR(" ** Seek to last position error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				if ( vc_table )
				wbf_free(vc_table);
				closeStreams();
				return 44;
			}
			if ( out_fputc(xwia_count) == -1 )
			{
				WERR("write xwi data error!\n");
			wbf_free(buffer);
			wbf_free(mode_table);
			wbf_free(ptr_table);
				closeStreams();
				return 45;
			}
			++xwia_count;
			while ( 1 )
			{
				is_more_xwia = xwia_count-- != 0;
				if ( !is_more_xwia )
					break;
				xwi_byte = wbf_fgetc();
				if ( out_fputc(xwi_byte) == -1 )
				{
					WERR("write xwi data error!");
				wbf_free(buffer);
				wbf_free(mode_table);
				wbf_free(ptr_table);
					closeStreams();
					return 46;
				}
			}
			if ( verbose == 2 )
			{
				end_of_xwi = out_ftell();
				WDBG(" end of XWI : 0x%08lx\n", end_of_xwi);
			}
		}
		wbf_free(buffer);
		wbf_free(mode_table);
		wbf_free(ptr_table);
		out_fclose();
	}
	if ( verbose )
		WDBG("Advance Algorithm Data offset: 0x%08llx\n", max_pos);
	wbf_fclose();

	if (wdff_fopen_r(0, 0))
	{
		wdff_fclose();
		for ( jj = 0; jj < wbf_mc; ++jj )
		{
			for ( kk = 0; kk < wbf_trc; ++kk )
			{
				wdff_remove(jj, kk);
			}
		}
	}
	WDBG("*** Done ***\n");
	return 0;
}

