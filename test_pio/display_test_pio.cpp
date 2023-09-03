/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <hardware/structs/rosc.h>
#include "ili9341/ili9341_cmd_regs.h"
#include <stdio.h>
#include <cstdlib>
#include <set>
#include <vector>
#include <malloc.h>

// extern "C" {
#include "pio_display_spi.h"
#include "logic_analyser.h"
// }

#define BINARY_INFO_TAG_HARDWARE_INFO BINARY_INFO_MAKE_TAG('H', 'I')

#define DISPLAY_LED            -1                 // for now I directly use 3.3 v for max baclight led's
#define DISPLAY_RESET           14                // Pin that hold hardware reset signal of LCD
#define DISPLAY_DCRS            5                 // Pin for lcd register select
#define DISPLAY_SPI_CS1         DISPLAY_DCRS + 1  // Pin for spi slave select
#define DISPLAY_SPI_CS2         DISPLAY_DCRS + 2  // Pin for spi slave select
#define DISPLAY_SPI_CS3         DISPLAY_DCRS + 3  // Pin for spi slave select
#define DISPLAY_SPI_CS4         DISPLAY_DCRS + 4  // Pin for spi slave select

#define DEFAULT_PIO_DCRSL       0xE01E // This command corresponds to set pins CSn to high and DCRS to low
#define DEFAULT_PIO_DCRSH       0xE01F // This command corresponds to set pins CSn to high and DCRS to high
#define DEFAULT_PIO_CS4_MASK    0xFFEF // This mask multiplied on DCRSL or DCRSH provides comand to set CS4 to low. (chose 4_th slave device)
#define DEFAULT_PIO_CS3_MASK    0xFFF7 // This mask multiplied on DCRSL or DCRSH provides comand to set CS3 to low. (chose 3_rd slave device)
#define DEFAULT_PIO_CS2_MASK    0xFFFB // This mask multiplied on DCRSL or DCRSH provides comand to set CS2 to low. (chose 2_nd slave device)
#define DEFAULT_PIO_CS1_MASK    0xFFFD // This mask multiplied on DCRSL or DCRSH provides comand to set CS1 to low. (chose 1_st slave device)

pio_spi_inst_t pio_spi_default_instance = {
            .pio = pio0,
            .sm = 0,
};

pio_spi_inst_t pio_spi_analyser_instance = {
            .pio = pio1,
            .sm = 0,
};

#define default_pio_spi pio_spi_default_instance
#define default_pio_analyser pio_spi_analyser_instance


uint32_t getTotalHeap(void) {
   extern char __StackLimit, __bss_end__;
   
   return &__StackLimit  - &__bss_end__;
}

uint32_t getFreeHeap(void) {
   struct mallinfo m = mallinfo();

   return getTotalHeap() - m.uordblks;
}


/*! \brief Struct that works with lcd cmds
 *  \ingroup lcd_control
*/
class lcd_cmds_t{
public:
    lcd_cmds_t(pio_spi_inst_t *spi): spi(spi), buf() {}

    inline void serialize_command(std::initializer_list<uint8_t> data){
        // First value will allways be recognized as cmd
        if(data.size() == 0) return;
        uint8_t cmd = *(data.begin());
        add_com(cmd);
        // Check wheter met software reset or sleep out
        if ( cmd == ILI9341_SLPOUT || cmd == ILI9341_SWRESET ){
            printf("Met cmd that require sleep on the bus\n");
            execute();
            sleep_ms(150);
        }
        for (auto it = data.begin() + 1 ; it != data.end(); it++){
            add_data(*it);
        };

        if(buf.size() >= 4096) { // limit each lcd_cmd buf to 4 kB +- number of arguments
            // printf("Current free memory is %d bytes\n", getFreeHeap());
            execute();
        }
    };

    inline void add_com(const uint8_t& cmd, const uint16_t& CSn_mask = DEFAULT_PIO_CS1_MASK){
        // Code bellow is used to generate PIO cmd that will be executed from OSR
        uint16_t pio_set_cmd = generate_pio_set_cmd(DEFAULT_PIO_DCRSL, CSn_mask);
        buf.push_back( pio_set_cmd >> 8 );
        buf.push_back( pio_set_cmd );
        // Add one of the commands to display as defined in ILI9341 Datasheet 
        buf.push_back( cmd );
    }

    inline void add_data(const uint8_t& data, const uint16_t& CSn_mask = DEFAULT_PIO_CS1_MASK){
        // Code bellow is used to generate PIO cmd that will be executed from OSR
        uint16_t pio_set_cmd = generate_pio_set_cmd(DEFAULT_PIO_DCRSH, CSn_mask);
        buf.push_back( pio_set_cmd >> 8 );
        buf.push_back( pio_set_cmd );
        // Add one of the command parameters to displlay as defined in ILI9341 Datasheet 
        buf.push_back( data );

        if(buf.size() >= 4096) { // limit each lcd_cmd buf to 4 kB +- number of arguments
            // printf("Current free memory is %d bytes\n", getFreeHeap());
        execute();
        }
    }

    void execute(){
        if(buf.size() == 0) return;
        std::vector< uint8_t > exec_buf;
        exec_buf.swap(buf); 
        printf("Executing cmd buf...\n");
        pio_spi_write8_blocking(spi, exec_buf.data(), exec_buf.size());
    }

private:
    uint16_t generate_pio_set_cmd(uint16_t pio_dcrs_cmd, uint16_t pio_csn_mask){
        return (pio_dcrs_cmd & pio_csn_mask);
    }

private:
    pio_spi_inst_t *spi;
    std::vector< uint8_t > buf;
};

void Address_set(lcd_cmds_t& cmds,
    uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
    // cmds.add_com(0x2a);
    cmds.serialize_command({
        uint8_t(ILI9341_CASET), 
        uint8_t(x1>>8), 
        uint8_t(x1),
        uint8_t(x2>>8),
        uint8_t(x2)
    });

    cmds.serialize_command({
        uint8_t(ILI9341_PASET), 
        uint8_t(y1>>8), 
        uint8_t(y1),
        uint8_t(y2>>8),
        uint8_t(y2)
    });

    cmds.serialize_command({uint8_t(ILI9341_RAMWR)});
    cmds.execute();						 
}

void Lcd_Init(void)
{
    printf("Start LED initialization sequence\n");
    gpio_put(DISPLAY_RESET,1);
    sleep_ms(5); 
    gpio_put(DISPLAY_RESET,0);
    sleep_ms(15);
    gpio_put(DISPLAY_RESET,1);
    sleep_ms(15);

    printf("Preparing LCD initialization buffer...\n");
    lcd_cmds_t init_cmds(&pio_spi_default_instance);
    init_cmds.serialize_command({uint8_t(ILI9341_SWRESET)});
    init_cmds.serialize_command({uint8_t(ILI9341_DISPOFF)});

    // init_cmds.add_com(0xCB); //Power Control A [39 2C 00 34 02]
    init_cmds.serialize_command({
        uint8_t(ILI9341_Power_control_A), 
        uint8_t(0x39), 
        uint8_t(0x2C),
        uint8_t(0x00),
        uint8_t(0x34),
        uint8_t(0x02),
    });

    // init_cmds.add_com(0xCF); //Power Control B [00 81 30]
    init_cmds.serialize_command({
        uint8_t(ILI9341_Power_control_B), 
        uint8_t(0x00), 
        uint8_t(0xC1),
        uint8_t(0x30)
    });

    // init_cmds.add_com(0xE8); //Driver Timing A [04 11 7A]
    init_cmds.serialize_command({
        uint8_t(ILI9341_Driver_timing_control_A), 
        uint8_t(0x85), 
        uint8_t(0x00),
        uint8_t(0x78)
    });

    // init_cmds.add_com(0xEA);  // Driver Timing B [66 00]
    init_cmds.serialize_command({
        uint8_t(ILI9341_Driver_timing_control_B), 
        uint8_t(0x00), 
        uint8_t(0x00)
    });

    // init_cmds.add_com(0xED);  // ILI POWERONSEQ
    init_cmds.serialize_command({
        uint8_t(ILI9341_Power_on_sequence_control), 
        uint8_t(0x64), 
        uint8_t(0x03),
        uint8_t(0X12),
        uint8_t(0X81)
    });

    // init_cmds.add_com(0xF7); // PUMPRATIO
    init_cmds.serialize_command({
        uint8_t(ILI9341_Pump_ratio_control), 
        uint8_t(0x20)
    });

    // init_cmds.add_com(0xC0);  //Power control
    init_cmds.serialize_command({
        uint8_t(ILI9341_PWCTRL_1), 
        uint8_t(0x23) //VRH[5:0]  
    });

    // init_cmds.add_com(0xC1);  //Power control
    init_cmds.serialize_command({
        uint8_t(ILI9341_PWCTRL_2), 
        uint8_t(0x10) //SAP[2:0];BT[3:0]  
    });

    // init_cmds.add_com(0xC5);  //VCM control
    init_cmds.serialize_command({
        uint8_t(ILI9341_VMCTRL1), 
        uint8_t(0x3e), //Contrast
        uint8_t(0x28)
    });

    // init_cmds.add_com(0xC7);  //VCM control2
    init_cmds.serialize_command({
        uint8_t(ILI9341_VMCTRL2), 
        uint8_t(0x86) //--
    });

    init_cmds.serialize_command({
        uint8_t(ILI9341_MADCTL), // Memory Access Control
        uint8_t(0x48) //--
    });

    // init_cmds.add_com(0x3A);  //Pixel read=565, write=565.
    init_cmds.serialize_command({
        uint8_t(ILI9341_PIXSET), 
        uint8_t(0x55)
    });

    // init_cmds.add_com(0xB1);  // Frame Control [00 1B]
    init_cmds.serialize_command({
        uint8_t(ILI9341_FRMCTR1), 
        uint8_t(0x00),
        uint8_t(0x18)
    });

    // init_cmds.add_com(0xB6);  // Display Function Control
    init_cmds.serialize_command({
        uint8_t(ILI9341_DISCTRL), 
        uint8_t(0x08),
        uint8_t(0x82),
        uint8_t(0x27),
    });

    init_cmds.serialize_command({uint8_t(ILI9341_SLPOUT)}); //Exit Sleep
    init_cmds.serialize_command({uint8_t(ILI9341_DISPON)}); //Display on
    init_cmds.serialize_command({uint8_t(ILI9341_RAMWR)});
    printf("Buffer prepared!\n");

    printf("Execute cmd buffer\n");
    init_cmds.execute();
    printf("Finish LED initialization sequence\n");
}

void H_line(lcd_cmds_t& cmds,
    uint16_t x, uint16_t y, uint16_t l, uint16_t c)                   
{	
  uint16_t i,j;
  cmds.serialize_command({uint8_t(ILI9341_RAMWR)});
  //cmds.add_com(0x02c); //write_memory_start
  l=l+x;
  Address_set(cmds,x,y,l,y);
  j=l*2;
  for(i=1;i<=j;i++)
  {
    cmds.add_data(c>>8);
    cmds.add_data(c);
  }
  cmds.execute();
}

void V_line(lcd_cmds_t& cmds,
    uint16_t x, uint16_t y, uint16_t l, uint16_t c)
{	
  uint16_t i,j;
  cmds.serialize_command({uint8_t(ILI9341_RAMWR)}); //write_memory_start
  l=l+y;
  Address_set(cmds,x,y,x,l);
  j=l*2;
  for(i=1;i<=j;i++)
  { 
    cmds.add_data(c>>8);
    cmds.add_data(c);
  }
  cmds.execute();
}

void Rect(lcd_cmds_t& cmds,
    uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c)
{
  H_line(cmds, x  , y  , w, c);
  H_line(cmds, x  , y+h, w, c);
  V_line(cmds, x  , y  , h, c);
  V_line(cmds, x+w, y  , h, c);
}

void Rectf(lcd_cmds_t& cmds,
    uint16_t x,uint16_t y,uint16_t w,uint16_t h,uint16_t c)
{
  uint16_t i;
  for(i=0;i<h;i++)
  {
    H_line(cmds, x  , y  , w, c);
    H_line(cmds, x  , y+i, w, c);
  }
}

uint32_t RGB(uint8_t r, uint8_t g, uint8_t b)
{
  return r << 16 | g << 8 | b;
}

void LCD_Clear(lcd_cmds_t& cmds, uint16_t j)                   
{	
  uint16_t i,m;
  Address_set(cmds, 0,0,320,240);
  for(i=0;i<240;i++)
    for(m=0;m<320;m++)
    {
      cmds.add_data(j>>8);
      cmds.add_data(j);
    }
  cmds.execute();
}


void setup_display_spi(bool is_8_bit = true, bool cpha = false, bool cpol = false)
{
    // Trying to initialize SPI
#if !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning pio/spi/spi_flash example requires a board with SPI pins
    puts("Default SPI pins were not defined");
#else
    printf("Initialize PIO module for extended SPI interface\n");

    uint offset = 0;
    if( is_8_bit ){
        offset = cpha ? pio_add_program(pio_spi_default_instance.pio, &spi_8_cpha1_mcs_program) : pio_add_program(pio_spi_default_instance.pio, &spi_8_cpha0_mcs_program);
    } else {
        offset = cpha ? pio_add_program(pio_spi_default_instance.pio, &spi_16_cpha1_mcs_program) : pio_add_program(pio_spi_default_instance.pio, &spi_16_cpha0_mcs_program);
    }
    printf("Loaded program at %d\n", offset);

    pio_display_spi_init(pio_spi_default_instance.pio, pio_spi_default_instance.sm, offset,
                 is_8_bit,       // 8 bits per SPI frame (false 16) <- 16 bit not ready
                 2.016f, // 62 MHz @ 125 clk_sys
                 cpha,   // CPHA = 0
                 cpol,   // CPOL = 0
                 PICO_DEFAULT_SPI_SCK_PIN,
                 PICO_DEFAULT_SPI_TX_PIN,
                 PICO_DEFAULT_SPI_RX_PIN,
                 DISPLAY_DCRS

    );
    // Make the 'SPI' pins available to picotool
    bi_decl(bi_program_description("Program to control 2.8inch SPI Module ILI9341 SKU:MSP2807 Touch Display"));
    // bi_decl(bi_program_feature_group_with_flags(BINARY_INFO_TAG_HARDWARE_INFO, 0, "hardware", BI_NAMED_GROUP_SHOW_IF_EMPTY));
    // bi_decl(bi_string(BINARY_INFO_TAG_HARDWARE_INFO, 0, "joystick: No name china potentiometer joystick"));
    // bi_decl(bi_string(BINARY_INFO_TAG_HARDWARE_INFO, 0, "display: 2.8inch SPI Module ILI9341 SKU:MSP2807 - Touch Display"));

    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_RX_PIN, "Display SPI RX")); 
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_TX_PIN, "Display SPI TX")); 
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_SCK_PIN, "Display SPI SCK")); 
    bi_decl(bi_1pin_with_name(DISPLAY_DCRS, "Display DC/RS"));
    bi_decl(bi_1pin_with_name(DISPLAY_SPI_CS1, "Display SPI CS1"));
    bi_decl(bi_1pin_with_name(DISPLAY_SPI_CS2, "Display SPI CS2"));
    bi_decl(bi_1pin_with_name(DISPLAY_SPI_CS3, "Display SPI CS3"));
    bi_decl(bi_1pin_with_name(DISPLAY_SPI_CS4, "Display SPI CS4"));
    bi_decl(bi_1pin_with_name(DISPLAY_RESET, "Display Hardware Reset"));


#endif
    // Here SPI initialization is finished

    // Initialize additional pins that would drive lcd
    gpio_init(DISPLAY_RESET);
    gpio_set_dir(DISPLAY_RESET, GPIO_OUT);
    gpio_put(DISPLAY_RESET, 1);
    printf("%s\n", gpio_get(DISPLAY_RESET) ? "HIGH" : "LOW");

    Lcd_Init();
}


void printbuf(uint8_t buf[], size_t len) {
    int i;
    for (i = 0; i < len; ++i) {
        if (i % 16 == 15)
            printf("%02x\n", buf[i]);
        else
            printf("%02x ", buf[i]);
    }

    // append trailing newline if there isn't one
    if (i % 16) {
        putchar('\n');
    }
}

void printbuf(uint16_t buf[], size_t len) {
    int i;
    for (i = 0; i < len; ++i) {
        if (i % 8 == 7)
            printf("%04x\n", buf[i]);
        else
            printf("%04x ", buf[i]);
    }

    // append trailing newline if there isn't one
    if (i % 8) {
        putchar('\n');
    }
}

bool get_random_bit() {
    return rosc_hw->randombit;
}

template<typename T> 
T random(T limit){
    T result = 0;
    for (uint8_t i = 0; i < sizeof(T)*8; ++i){
        result += (get_random_bit() << i);
    }
    return result % (limit + 1);
}

template<typename T>
T random(T min, T max){
    T result = 0;
    T rand = 0;
    for (uint8_t i = 0; i < sizeof(T)*8; ++i){
        rand += (get_random_bit() << i);
    }
    result = rand % (max - min + 1) + min;
    return result;
}


int main() {
    stdio_init_all(); // Enable USB debug on /dev/ttyACM*
    sleep_ms(5*1000);
    printf("Start the program\n");
    
    // Initialize all needed connections
    setup_display_spi();

    
    // // We're going to capture into a u32 buffer, for best DMA efficiency. Need
    // // to be careful of rounding in case the number of pins being sampled
    // // isn't a power of 2.
    // uint total_sample_bits = CAPTURE_N_SAMPLES * CAPTURE_PIN_COUNT;
    // total_sample_bits += bits_packed_per_word(CAPTURE_PIN_COUNT) - 1;
    // uint buf_size_words = total_sample_bits / bits_packed_per_word(CAPTURE_PIN_COUNT);
    // uint32_t *capture_buf = (uint32_t *) malloc(buf_size_words * sizeof(uint32_t));
    // hard_assert(capture_buf);

    // // Grant high bus priority to the DMA, so it can shove the processors out
    // // of the way. This should only be needed if you are pushing things up to
    // // >16bits/clk here, i.e. if you need to saturate the bus completely.
    // bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    // logic_analyser_init(default_pio_analyser.pio, default_pio_analyser.sm, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, 1.f);

    // printf("Arming trigger\n");
    // logic_analyser_arm(default_pio_analyser.pio, default_pio_analyser.sm, 0, capture_buf, buf_size_words, CAPTURE_PIN_BASE, true);


    printf("Prepare cmd buffer for drawing\n");
    lcd_cmds_t draw_cmds(&pio_spi_default_instance);
    printf("Enter main loop\n");
    uint16_t x = 0, y = 0, w = 0, h = 0;
    for (size_t i = 0; i < 1; ++i) {
        printf("Flush Screen Blue\n");
        LCD_Clear(draw_cmds, 0xf800);
        sleep_ms(2000);
        printf("Flush Screen Green\n");
        LCD_Clear(draw_cmds, 0x07E0);
        sleep_ms(2000);
        printf("Flush Screen Red\n");
        LCD_Clear(draw_cmds, 0x001F);
        sleep_ms(2000);
        printf("Flush Screen Black\n");
        LCD_Clear(draw_cmds, 0x0);
        
        sleep_ms(2000);
        // for(int i=0;i<500;i++) {
        //     x = random<uint16_t>(20, 150);
        //     w = random<uint16_t>(150, 300) - x;
        //     y = random<uint16_t>(20, 120);
        //     h = random<uint16_t>(120, 220) - y;
        //     printf("Drawing rect {{%d, %d, %d, %d}}\n", x, y, w, h);
        //     Rect(draw_cmds, 
        //         x,
        //         y,
        //         w,
        //         h,
        //         random<uint16_t>(65535)); // rectangle at x, y, with, hight, color
        //     draw_cmds.execute();

        // }
    }

    // The logic analyser should have started capturing as soon as it saw the
    // first transition. Wait until the last sample comes in from the DMA.
    // dma_channel_wait_for_finish_blocking(0);

    // print_capture_buf(capture_buf, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, CAPTURE_N_SAMPLES);
    
    return 0;
}
