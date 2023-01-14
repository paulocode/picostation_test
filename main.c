#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "main.pio.h"
#include "pico/multicore.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"

#include "f_util.h"
#include "ff.h"
#include "rtc.h"
#include "hw_config.h"
#include "utils.h"
#include "values.h"
#include "subq.h"
#include "cmd.h"

// globals
mutex_t mechacon_mutex;
volatile uint latched = 0;
volatile uint mechachon_sm_offset;
volatile uint soct_offset;
volatile uint subq_offset;
volatile bool soct = 0;
volatile bool subq_start = 0;
volatile uint sled_move_direction = SLED_MOVE_STOP;
volatile uint count_track = 0;
volatile uint track = 0;
volatile uint original_track = 0;
volatile uint sector = 0;
volatile uint sector_for_track_update = 0;
volatile uint64_t subq_start_time = 0;
volatile uint64_t sled_timer = 0;
volatile int num_logical_tracks = 0;

bool SENS_data[16] = {
	0,0,0,0,0,
	1, // FOK
	0,0,0,0,
	0, // GFS
	0, // COMP
	0, // COUT
	0,0,0
};

uint8_t tracksubq[12];

int *logical_track_to_sector;

void i2s_data_thread() {
	int bytesRead;
	uint32_t *pio_samples[2];
	pio_samples[0] = malloc(CD_SAMPLES_BYTES*2);
	pio_samples[1] = malloc(CD_SAMPLES_BYTES*2);
	memset(pio_samples[0], 0, CD_SAMPLES_BYTES*2);
	memset(pio_samples[1], 0, CD_SAMPLES_BYTES*2);	
	
	ushort *cd_samples = malloc(CD_SAMPLES_BYTES); 
	
	FRESULT fr;
	FIL fil;
	sd_card_t *pSD;
	int bytes;
	char buf[128];
	pSD = sd_get_by_num(0);
	fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
	if (FR_OK != fr) panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
	fr = f_open(&fil, "STREET MUSIC.cue", FA_READ);
	if (FR_OK != fr && FR_EXIST != fr)
		panic("f_open(%s) error: (%d)\n", FRESULT_str(fr), fr);

	f_gets(buf, 128, &fil);

    while (1) {
        f_gets(buf, 128, &fil);
        char * token = strtok(buf, " ");
        if (strcmp("TRACK", token) == 0) {
			num_logical_tracks++;
        }
        if (f_eof(&fil)) {
            break;
        }
    }
	
	f_rewind(&fil);
	printf("%d\n",num_logical_tracks);

	logical_track_to_sector = malloc(sizeof(int)*(num_logical_tracks+2));
	logical_track_to_sector[0] = 0;
	logical_track_to_sector[1] = 4500;

	int logical_track = 0;
	f_gets(buf, 128, &fil);
    while (1) {
        f_gets(buf, 128, &fil);
		
        if (f_eof(&fil)) {
            break;
        }		
        char * token;
        token = strtok(buf, " ");
        //printf("%s\n", token);
        if (strcmp("TRACK", token) == 0) {
			token = strtok(NULL, " ");
            logical_track = atoi(token);
        }
        f_gets(buf, 128, &fil);
		token = strtok(buf, " ");
		token = strtok(NULL, " ");
		token = strtok(NULL, " ");
		
		int mm = atoi(strtok(token, ":"));
		int ss = atoi(strtok(NULL, ":"));
		int ff = atoi(strtok(NULL, ":"));
		if (logical_track != 1) {
			logical_track_to_sector[logical_track] = mm*60*75 + ss*75 + ff + 4650;
		}
		printf("%d %d %d %d\n", logical_track, mm, ss, ff);
    }
	
	f_close(&fil);
	fr = f_open(&fil, "STREET MUSIC.bin", FA_READ);
	if (FR_OK != fr && FR_EXIST != fr)
		panic("f_open(%s) error: (%d)\n", FRESULT_str(fr), fr);

	logical_track_to_sector[num_logical_tracks+1] = f_size(&fil)/2352 + 4650;

	for(int i=0;i<num_logical_tracks+2;i++) {
		printf("%d\n",logical_track_to_sector[i]);
	}
	
	
	int channel = dma_claim_unused_channel(true);
	dma_channel_config c = dma_channel_get_default_config(channel);
	channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
	channel_config_set_dreq(&c, DREQ_PIO0_TX0);             
	dma_channel_configure(channel, &c, &pio0->txf[I2S_DATA_SM], pio_samples[0], CD_SAMPLES*2, false);
	
	int buffer_for_dma = 1;
	int buffer_for_sd_read = 0;

	while (true) {
		if (mutex_try_enter(&mechacon_mutex,0)) {
			while(!pio_sm_is_rx_fifo_empty(pio1, MECHACON_SM)) {
				uint c = reverseBits(pio_sm_get_blocking(pio1, MECHACON_SM),8); 
				latched >>= 8;
				latched |= c << 16;
			}
			gpio_put(SENS, SENS_data[latched >> 20]);
			mutex_exit(&mechacon_mutex);
		}	
		
		if (buffer_for_dma != buffer_for_sd_read) {
			fr = f_read(&fil, cd_samples, CD_SAMPLES_BYTES, &bytesRead);
			uint64_t seek_bytes = (sector-4650)*2352LL;
			if (seek_bytes >= 0) {
				fr = f_lseek(&fil, seek_bytes);
				if (FR_OK != fr) {
					f_rewind(&fil);
				}
			}
			
			if (FR_OK != fr)
				panic("f_read(%s) error: (%d)\n", FRESULT_str(fr), fr);

			for (int i=0; i<CD_SAMPLES*2 ;i++) {
				uint32_t i2s_data = reverseBits(cd_samples[i], 16) << 8;

				if (i2s_data & 0x100) {
					i2s_data |= 0xFF;
				}		
				
				pio_samples[buffer_for_sd_read][i] = i2s_data;
			}

			buffer_for_sd_read = (buffer_for_sd_read + 1) % 2;
		}
		
		if (!dma_channel_is_busy(channel)) {
			buffer_for_dma = (buffer_for_dma + 1) % 2;

			dma_hw->ch[channel].read_addr = pio_samples[buffer_for_dma];
			dma_channel_start(channel);
		}
	}
}

void initialize() {
	set_sys_clock_pll(948000000, 7, 1); // 135428571 Hz, 67714286 Hz PS1 clock
	srand(time(NULL));
	mutex_init(&mechacon_mutex);

	gpio_init(SCEX_DATA);
	gpio_init(SENS);
	gpio_init(LMTSW);
	gpio_init(XLAT);
	gpio_init(DOOR);
	gpio_init(RESET);
	gpio_init(SQCK);
	gpio_init(SQSO);
	gpio_init(CMD_CK);

	gpio_set_dir(SCEX_DATA, GPIO_OUT);
	gpio_put(SCEX_DATA, 1);
	gpio_set_dir(SENS, GPIO_OUT);
	gpio_set_dir(LMTSW, GPIO_OUT);
	gpio_set_dir(XLAT, GPIO_IN);
	gpio_set_dir(DOOR, GPIO_IN);	
	gpio_set_dir(RESET, GPIO_IN);	
	gpio_set_dir(SQCK, GPIO_IN);	
	gpio_set_dir(SQSO, GPIO_OUT);	
	gpio_set_dir(CMD_CK, GPIO_IN);	
	gpio_put(SQSO, 0);
	
	gpio_set_input_hysteresis_enabled(RESET,true); 
	gpio_set_input_hysteresis_enabled(SQCK,true);
	gpio_set_input_hysteresis_enabled(XLAT,true);
	gpio_set_input_hysteresis_enabled(CMD_CK,true);

	uint i2s_pio_offset = pio_add_program(pio0, &i2s_data_program);
	i2s_data_program_init(pio0, I2S_DATA_SM, i2s_pio_offset, DA15);
		
    uint offset2 = pio_add_program(pio0, &i2s_lrck_program);
    i2s_lrck_program_init(pio0, LRCK_DATA_SM, offset2, DA15+2);
	
    mechachon_sm_offset = pio_add_program(pio1, &mechacon_program);
    mechacon_program_init(pio1, MECHACON_SM, mechachon_sm_offset, CMD_DATA);
	
    uint offset3 = pio_add_program(pio0, &cpu_clk_program);
    cpu_clk_program_init(pio0, CPU_CLK_SM, offset3, CLK);

    uint offset5 = pio_add_program(pio1, &scor_program);
	scor_program_init(pio1, SCOR_SM, offset5, SCOR);    

    soct_offset = pio_add_program(pio1, &soct_program);
	
    subq_offset = pio_add_program(pio1, &subq_program);

	uint64_t startTime = time_us_64();

	pio_enable_sm_mask_in_sync(pio0, (1u << CPU_CLK_SM) | (1u << I2S_DATA_SM) | (1u << LRCK_DATA_SM));

	sleep_ms(50);

	gpio_set_dir(RESET, GPIO_OUT);	
	gpio_put(RESET,0);
	sleep_ms(300);
	gpio_set_dir(RESET, GPIO_IN);
	
	while((time_us_64() - startTime) < 30000) {
		if (gpio_get(RESET) == 0) {
			startTime = time_us_64();
		}
	}
	
	while((time_us_64() - startTime) < 30000) {
		if (gpio_get(CMD_CK) == 0) {
			startTime = time_us_64();
		}
	}
	
	printf("ON!\n");
	multicore_launch_core1(i2s_data_thread);
	gpio_set_irq_enabled_with_callback(XLAT, GPIO_IRQ_EDGE_RISE, true, &interrupt_xlat);
	pio_enable_sm_mask_in_sync(pio1, (1u << SCOR_SM) | (1u << MECHACON_SM));
}

int main() {
	sleep_ms(1);
	stdio_init_all();

	sleep_ms(5000);
	
	initialize();
	
	while (true) {
		if (mutex_try_enter(&mechacon_mutex,0)) {
			while(!pio_sm_is_rx_fifo_empty(pio1, MECHACON_SM)) {
				uint c = reverseBits(pio_sm_get_blocking(pio1, MECHACON_SM),8); 
				latched >>= 8;
				latched |= c << 16;
			}
			gpio_put(SENS, SENS_data[latched >> 20]);
			mutex_exit(&mechacon_mutex);
		}
		
			
		if (track < 0 || sector < 0) {
			track = 0;
			sector = 0;
			sector_for_track_update = 0;
		}
		
		if (track > 24000 || sector > 440000) {
			track = 24000;
			sector = track_to_sector(track);
			sector_for_track_update = sector;
		}
		
		if (gpio_get(RESET) == 0) {
			printf("RESET!\n");
			pio_sm_set_enabled(pio1, SUBQ_SM, false);
			pio_sm_set_enabled(pio1, SOCT_SM, false);
			mechacon_program_init(pio1, MECHACON_SM, mechachon_sm_offset, CMD_DATA);
			subq_start = 0;
			soct = 0;
			
			gpio_init(SQSO);
			gpio_set_dir(SQSO, GPIO_OUT);
			gpio_put(SQSO, 0);
			
			uint64_t startTime = time_us_64();
			
			while ((time_us_64() - startTime) < 30000) {
				if (gpio_get(RESET) == 0) {
					startTime = time_us_64();
				}
			}
			
			while ((time_us_64() - startTime) < 30000) {
				if (gpio_get(CMD_CK) == 0) {
					startTime = time_us_64();
				}
			}
			
			pio_sm_set_enabled(pio1, MECHACON_SM, true);
		}
		

		if (soct) {
			uint interrupts = save_and_disable_interrupts();
			subq_start = 0;
			// waiting for RX FIFO entry does not work.
			sleep_us(300);
			soct = 0;		
			pio_sm_set_enabled(pio1, SOCT_SM, false);
			subq_start_time = time_us_64();
			restore_interrupts(interrupts);
		} else if (sled_move_direction == SLED_MOVE_FORWARD) {
			if ((time_us_64() - sled_timer) > TRACK_MOVE_TIME_US) {
				sled_timer = time_us_64();
				track++;
				sector = track_to_sector(track);
				sector_for_track_update = sector;
				if ((track - original_track) >= count_track) {
					original_track = track;
					SENS_data[SENS_COUT] = !SENS_data[SENS_COUT];
				}
			}
		} else if (sled_move_direction == SLED_MOVE_REVERSE) {
			if ((time_us_64() - sled_timer) > TRACK_MOVE_TIME_US) {
				sled_timer = time_us_64();
				track--;
				sector = track_to_sector(track);
				sector_for_track_update = sector;
				if ((original_track - track) >= count_track) {
					original_track = track;
					SENS_data[SENS_COUT] = !SENS_data[SENS_COUT];
				}

			}
		} else if (SENS_data[SENS_GFS]) {
			if (!subq_start) {
				if ((time_us_64() - subq_start_time) > 13333) {
					subq_start_time = time_us_64();
					subq_start = 1;

					start_subq();
				} 
			} else if (gpio_get(SQCK) == 0) {
				wait_end_subq();
				subq_start = 0;
			} else if ((time_us_64() - subq_start_time) > 6667) {
				subq_start = 0;
				sector++;
				pio_sm_set_enabled(pio1, SUBQ_SM, false);
			} 
		} else {
			subq_start = 0;
			pio_sm_set_enabled(pio1, SUBQ_SM, false);
		}
	}

}
