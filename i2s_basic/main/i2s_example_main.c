#include "driver/gpio.h"
#include "driver/i2s.h"
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"
#include "esp_err.h"
#include "esp_intr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/lldesc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/soc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

DMA_ATTR uint8_t      dma_buff[128];
DMA_ATTR lldesc_t     dma_descriptor;

void print_buff( void *p_buffer, uint16_t buffer_size );

#define ARRAY_SIZE(array)  (sizeof(array) / sizeof(array[0]))

#define DATA_BUS_WIDTH          ( 8 )                   // Number of data input streams

// Configure bit & word clocks out using the LEDC module
static const gpio_num_t wc_clk_pin = 37;
static const gpio_num_t bc_clk_pin = 38;
static const gpio_num_t data_bus_pins[DATA_BUS_WIDTH + 2] =
{
  35,        // List data bus pins first (see for loop below)
  4, 5, 6, 7, 8, 9, 10, //, 11,12,
  wc_clk_pin, bc_clk_pin
};

// Assumes one word per channel
#define SERIAL_WORD_BIT_LENGTH    ( 16 )
#define PARALLEL_WORD_BYTE_LENGTH ( ( ( DATA_BUS_WIDTH  - 1 ) / 8 ) + 1 )   // Number of bytes needed to store a single parallel word

void app_main()
{
  ledc_timer_config_t ledc_timer_bclk =
  {
    .duty_resolution = LEDC_TIMER_2_BIT,            // resolution of PWM duty
    .freq_hz = 1 * 1000 * 1000,                     // frequency of PWM signal
    .speed_mode = LEDC_LOW_SPEED_MODE,              // timer mode
    .timer_num = LEDC_TIMER_1,                      // timer index
    .clk_cfg = LEDC_AUTO_CLK,                       // Auto select the source clock
  };

  ledc_timer_config_t ledc_timer_wclk =
  {
    .duty_resolution = LEDC_TIMER_2_BIT,
    .freq_hz = ledc_timer_bclk.freq_hz / ( SERIAL_WORD_BIT_LENGTH * 2 ),  // *2 for left & right channels per word
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_2,
    .clk_cfg = LEDC_AUTO_CLK,
  };

  ledc_channel_config_t ledc_channel_bclk =
  {
    .channel    = LEDC_CHANNEL_0,
    .duty       = 2 << ( ledc_timer_bclk.duty_resolution - LEDC_TIMER_1_BIT - 1 ),  // LEDC channel duty, the range of duty setting is [0, (2**duty_resolution)] */
    .gpio_num   = ( bc_clk_pin ),
    .speed_mode = ledc_timer_bclk.speed_mode,
    .hpoint     = 0,
    .timer_sel  = ledc_timer_bclk.timer_num,
    .intr_type  = LEDC_INTR_DISABLE,
    .flags.output_invert = false,
  };

  ledc_channel_config_t ledc_channel_wclk =
  {
    .channel    = LEDC_CHANNEL_1,
    .duty       = 2 << ( ledc_timer_wclk.duty_resolution - LEDC_TIMER_1_BIT - 1 ),  // LEDC channel duty, the range of duty setting is [0, (2**duty_resolution)] */
    .gpio_num   = ( wc_clk_pin ),
    .speed_mode = ledc_timer_wclk.speed_mode,
    .hpoint     = 0,
    .timer_sel  = ledc_timer_wclk.timer_num,
    .intr_type  = LEDC_INTR_DISABLE,
    .flags.output_invert = false,
  };

  ledc_timer_config( &ledc_timer_bclk );
  ledc_timer_config( &ledc_timer_wclk );
  ledc_channel_config( &ledc_channel_wclk );
  ledc_channel_config( &ledc_channel_bclk );
  periph_module_enable( PERIPH_LEDC_MODULE );      // Get the two clocks more or less in phase

  // Set up our databus and I2S clock signals as inputs
  gpio_config_t conf =
  {
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };

  for ( uint8_t idx = 0; idx < ARRAY_SIZE( data_bus_pins ); idx++ )
  {
    conf.pin_bit_mask = 1LL << data_bus_pins[idx];
    gpio_config( &conf );

    if ( idx < DATA_BUS_WIDTH )
    {
      gpio_matrix_in( data_bus_pins[idx], ( DATA_BUS_WIDTH <= 8 ? I2S0I_DATA_IN8_IDX : I2S0I_DATA_IN0_IDX ) + idx, false );
    }
  }

  // Route the bit & word clocks from the LEDC module to the I2S module
  gpio_matrix_out( bc_clk_pin, LEDC_LS_SIG_OUT0_IDX, false, false );
  gpio_matrix_out( wc_clk_pin, LEDC_LS_SIG_OUT1_IDX, false, false );
  gpio_matrix_in( bc_clk_pin, I2S0I_WS_IN_IDX, false );       // I2S0I_WS_IN_IDX  = Pixel clock in Camera mode
  gpio_matrix_in( wc_clk_pin, I2S0I_V_SYNC_IDX, false );      // I2S0I_V_SYNC_IDX = Word Clock

  // For I2S in parallel camera input mode data is received only when H_SYNC = H_ENABLE = 1
  gpio_matrix_in( 0x38, I2S0I_H_SYNC_IDX, false );            // 0x3C sets signal low (0), 0x38 sets signal high (1)
  gpio_matrix_in( 0x38, I2S0I_H_ENABLE_IDX, false );

  periph_module_enable( PERIPH_I2S0_MODULE );                 // Enable and configure I2S peripheral
  I2S0.conf.rx_reset = 1;
  I2S0.conf.rx_reset = 0;
  I2S0.conf.rx_fifo_reset = 1;
  I2S0.conf.rx_fifo_reset = 0;
  I2S0.lc_conf.in_rst = 1;
  I2S0.lc_conf.in_rst = 0;

  I2S0.conf.rx_slave_mod = 1;                 // Enable slave mode (sampling clock is external)
  I2S0.conf2.lcd_en = 1;                      // Enable parallel mode
  I2S0.conf2.camera_en = 1;                   // Use HSYNC/VSYNC/HREF to control sampling
  I2S0.clkm_conf.clk_sel = 2;                 // 0: No clock  1: APLL_CLK   2: PLL_160M_CLK  3: No clock
  I2S0.clkm_conf.clk_en = 1;                  // Enable clock gate
  // In I2S slave mode, IS20 freq >= 8 * f_bclk.

  // Configure clock divider
  I2S0.clkm_conf.clkm_div_a = 1;              // Fractional clock divider denominator value
  I2S0.clkm_conf.clkm_div_b = 0;              // Fractional clock divider numerator value
  I2S0.clkm_conf.clkm_div_num = 1;            // Integral I2S clock divider value
  I2S0.sample_rate_conf.rx_bck_div_num = 1;   // i2s clk is divided before reaches BCK output

  // FIFO will sink data to DMA
  I2S0.fifo_conf.val = 0;
  I2S0.fifo_conf.dscr_en = 1;                 // Enable I2S DMA mode
  I2S0.fifo_conf.tx_fifo_mod_force_en = 1;    // The bit should always be set to 1, as per i2s_struct.h
  I2S0.fifo_conf.rx_fifo_mod_force_en = 1;    // The bit should always be set to 1, as per i2s_struct.h

  // FIFO configuration
  I2S0.fifo_conf.rx_fifo_mod = 0;                    // Receiver FIFO mode configuration bits, 0: right & left channel, 1: one channel
  I2S0.conf_chan.rx_chan_mod = 0;                    // I2S receiver channel mode configuration bits, not used when rx_dma_equal = 0

  // Clear flags which are used in I2S serial mode
  I2S0.sample_rate_conf.rx_bits_mod = PARALLEL_WORD_BYTE_LENGTH *
                                      8;  // Bit length of I2S receiver channel (word width, must be multiple of 8)
  // Value of 0 = 32-bit.  When = 8, LSB = I2S0I_DATA_IN8_IDX, else LSB = I2S0I_DATA_IN0_IDX
  I2S0.conf.rx_right_first = 0;                     // Receive right channel data first
  I2S0.conf.rx_msb_right = 0;                       // Place right channel data at the MSB in the receive FIFO
  I2S0.conf.rx_msb_shift = 0;                       // Enable receiver in Phillips standard mode
  I2S0.conf.rx_mono = 0;                            // Enable receiver in mono mode
  I2S0.conf.rx_short_sync = 0;                      // Enable receiver in PCM standard mode
  I2S0.conf.rx_dma_equal = 0;
  //I2S0.conf.rx_big_endian = 1;
  I2S0.timing.val = 0;                              // Clear all the bits in I2S0.timing.* union

  memset( dma_buff, 0x5A, sizeof( dma_buff ) );
  printf( "In Buffer:" );
  print_buff( dma_buff, 10 );

  dma_descriptor.length = 0;                            // Number of byte written to the buffer
  dma_descriptor.size = sizeof( dma_buff );           // In bytes, must be in whole number of words
  dma_descriptor.owner = 1;                             // The allowed operator is the DMA controller
  dma_descriptor.sosf = 0;                              // Start of sub-frame. Also likely not used with I2S
  dma_descriptor.buf = ( uint8_t * )dma_buff;
  dma_descriptor.offset = 0;
  dma_descriptor.empty = 0;
  dma_descriptor.eof = 1;                            // indicates the end of the linked list
  dma_descriptor.qe.stqe_next = NULL;                // pointer to the next descriptor

  uint16_t number_samples = 2;
  I2S0.rx_eof_num = number_samples * PARALLEL_WORD_BYTE_LENGTH * SERIAL_WORD_BIT_LENGTH;     // The length of data to be received
  I2S0.in_link.addr = ( uint32_t )&dma_descriptor;
  I2S0.in_link.start = 1;                            // Start the inlink descriptor
  I2S0.int_clr.val = I2S0.int_raw.val;               // Clear interrupt flags
  I2S0.int_ena.val = 0;                              // Disable all interrupts

  I2S0.conf.rx_start = 1;                            // Start I2S & the DMA
  vTaskDelay( 250 / portTICK_RATE_MS );
  I2S0.conf.rx_start = 0;                            // Stop I2S & the DMA

  printf( "Out Buffer:" );
  print_buff( dma_buff, I2S0.rx_eof_num );

  // Turn parallel data into serial data (inefficient/computationally intensive. Should be optimized, maybe with a lookup table):
  uint8_t  *parallel_data_ptr = dma_buff;

  for ( uint8_t sample_idx = 0; sample_idx < number_samples; sample_idx++ )
  {
    uint32_t serial_data[DATA_BUS_WIDTH] = { 0 };

    for ( uint8_t bit_idx = 0; bit_idx < SERIAL_WORD_BIT_LENGTH; bit_idx++ )
    {
      uint32_t parallel_data = *( ( uint32_t * )parallel_data_ptr );

      for ( uint8_t data_bus_idx = 0; data_bus_idx < DATA_BUS_WIDTH; data_bus_idx++ )
      {
        serial_data[data_bus_idx] <<= 1;
        serial_data[data_bus_idx] |= parallel_data & 0x01;
        parallel_data >>= 1;
      }

      parallel_data_ptr += PARALLEL_WORD_BYTE_LENGTH;
    }

    for ( uint8_t data_bus_idx = 0; data_bus_idx < DATA_BUS_WIDTH; data_bus_idx++ )
    {
      printf( "Sample #%i, Data lane %i (%i gpio pin): %i\n", sample_idx, data_bus_idx, data_bus_pins[data_bus_idx],
              serial_data[data_bus_idx] );
    }
  }
}

//-----------------------------------------------------------------------------
void print_buff_custom_format( void *p_buffer, uint16_t buffer_size, uint8_t bytes_per_line, uint32_t base_header_line_cnt,
                               uint32_t base_header_address_offset )
{
  char      line_header[20] = "";
  char      line_text[( bytes_per_line * 3 ) + 1]; // 2 ASCII Characters + space + null
  uint16_t  line_cnt      = 0;
  uint8_t   line_byte_cnt = 0;
  uint8_t   *p_next_in  = ( uint8_t * )p_buffer;
  char      *p_next_out = line_text;

  if ( buffer_size > bytes_per_line )
  {
    printf( "\n" );
  }

  while ( buffer_size )
  {
    if ( line_byte_cnt == 0 )
    {
      sprintf( line_header, "[%04i-0x%04x]: 0x", line_cnt + base_header_line_cnt,
               ( line_cnt * bytes_per_line ) + base_header_address_offset );
      line_cnt++;
      line_byte_cnt = 0;
      p_next_out    = line_text;
    }

    p_next_out += sprintf( p_next_out, "%02x ", *p_next_in );
    p_next_in++;
    buffer_size--;
    line_byte_cnt++;

    if ( ( line_byte_cnt == bytes_per_line ) || ( buffer_size == 0 ) )
    {
      printf( "%s%s\n", line_header, line_text );
      line_byte_cnt = 0;
    }
  }
}

//-----------------------------------------------------------------------------
void print_buff( void *p_buffer, uint16_t buffer_size )
{
  print_buff_custom_format( p_buffer, buffer_size, 16, 0, 0 );
}
