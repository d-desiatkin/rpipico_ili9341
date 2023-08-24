/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include "hardware/structs/rosc.h"
#include <hardware/spi.h>
#include <stdio.h>
#include <cstdlib>
#include <set>
#include <vector>
#include <malloc.h>

#define BUF_LEN         0x100
#define LED            -1  // for now I directly use 3.3 v  
#define RESET           14 // Pin that hold reset signal of LCD
#define RS              15 // Pin for lcd register select


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
class lcd_cmds{
public:
    lcd_cmds(spi_inst_t *spi): spi(spi), buf() {}

    inline void add_com(uint8_t cmd){
        add(cmd, 0);
    }

    inline void add_data(uint8_t data){
        add(data, 1);
    }

    inline void add(uint8_t value, bool reg_num) {
        buf.push_back({value, reg_num});
        if(buf.size() >= 512) { // limit each lcd_cmd buf to 1 kB
            // printf("Current free memory is %d bytes\n", getFreeHeap());
            execute();
        }
    };


    void execute(){
        if(buf.size() == 0) return;
        bool rs_reg = buf[0].second;
        gpio_put(RS, rs_reg);
        std::vector<uint8_t> sub_buf_to_exec;
        sub_buf_to_exec.reserve(20);
        for(auto it=buf.begin(); it != buf.end(); it++){
            if(it->second != rs_reg){
                while(spi_is_busy(spi));
                spi_write_blocking(spi, sub_buf_to_exec.data(), sub_buf_to_exec.size());
                sub_buf_to_exec.clear();
                rs_reg = it->second;
                gpio_put(RS, rs_reg);
            }
            sub_buf_to_exec.push_back(it->first);
        }
        while(spi_is_busy(spi));
        spi_write_blocking(spi, sub_buf_to_exec.data(), sub_buf_to_exec.size());
        buf.clear();
    }

private:
    spi_inst_t *spi;
    std::vector< std::pair<uint8_t, bool>> buf;
};

void Address_set(lcd_cmds& cmds,
    uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
    cmds.add_com(0x2a);
    cmds.add_data(x1>>8);
    cmds.add_data(x1);
    cmds.add_data(x2>>8);
    cmds.add_data(x2);

    cmds.add_com(0x2b);
	cmds.add_data(y1>>8);
	cmds.add_data(y1);
	cmds.add_data(y2>>8);
	cmds.add_data(y2);
	cmds.add_com(0x2c);
    cmds.execute();						 
}

void Lcd_Init(void)
{
    printf("Start LED initialization sequence\n");
    gpio_put(RESET,1);
    sleep_ms(5); 
    gpio_put(RESET,0);
    sleep_ms(15);
    gpio_put(RESET,1);
    sleep_ms(15);

    lcd_cmds init_cmds(spi_default);
    init_cmds.add_com(0xCB);
    init_cmds.add_data(0x39);
    init_cmds.add_data(0x2C);
    init_cmds.add_data(0x00);
    init_cmds.add_data(0x34);
    init_cmds.add_data(0x02);

    init_cmds.add_com(0xCF);
    init_cmds.add_data(0x00);
    init_cmds.add_data(0XC1);
    init_cmds.add_data(0X30);

    init_cmds.add_com(0xE8);
    init_cmds.add_data(0x85);
    init_cmds.add_data(0x00);
    init_cmds.add_data(0x78);

    init_cmds.add_com(0xEA);
    init_cmds.add_data(0x00);
    init_cmds.add_data(0x00);

    init_cmds.add_com(0xED);
    init_cmds.add_data(0x64);
    init_cmds.add_data(0x03);
    init_cmds.add_data(0X12);
    init_cmds.add_data(0X81);

    init_cmds.add_com(0xF7);
    init_cmds.add_data(0x20);

    init_cmds.add_com(0xC0);  //Power control 
    init_cmds.add_data(0x23); //VRH[5:0]  

    init_cmds.add_com(0xC1);  //Power control 
    init_cmds.add_data(0x10); //SAP[2:0];BT[3:0]  

    init_cmds.add_com(0xC5);  //VCM control 
    init_cmds.add_data(0x3e); //Contrast
    init_cmds.add_data(0x28);

    init_cmds.add_com(0xC7);  //VCM control2 
    init_cmds.add_data(0x86); //--

    init_cmds.add_com(0x36);  // Memory Access Control
    init_cmds.add_data(0x48);

    init_cmds.add_com(0x3A);
    init_cmds.add_data(0x55);

    init_cmds.add_com(0xB1); 
    init_cmds.add_data(0x00); 
    init_cmds.add_data(0x18);

    init_cmds.add_com(0xB6);  // Display Function Control 
    init_cmds.add_data(0x08);
    init_cmds.add_data(0x82);
    init_cmds.add_data(0x27);

    init_cmds.add_com(0x11);  //Exit Sleep   
    sleep_ms(120); 

    init_cmds.add_com(0x29);  //Display on 
    init_cmds.add_com(0x2c); 		

    init_cmds.execute();
    printf("Finish LED initialization sequence\n");
}

void H_line(lcd_cmds& cmds,
    uint16_t x, uint16_t y, uint16_t l, uint16_t c)                   
{	
  uint16_t i,j;
  cmds.add_com(0x02c); //write_memory_start
  //digitalWrite(RS,HIGH);
  l=l+x;
  Address_set(cmds,x,y,l,y);
  j=l*2;
  for(i=1;i<=j;i++)
  {
    cmds.add_data(c);
  }
  cmds.execute();
}

void V_line(lcd_cmds& cmds,
    uint16_t x, uint16_t y, uint16_t l, uint16_t c)
{	
  uint16_t i,j;
  cmds.add_com(0x02c); //write_memory_start
  //digitalWrite(RS,HIGH);
  l=l+y;
  Address_set(cmds,x,y,x,l);
  j=l*2;
  for(i=1;i<=j;i++)
  { 
    cmds.add_data(c);
  }
  cmds.execute();
}

void Rect(lcd_cmds& cmds,
    uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c)
{
  H_line(cmds, x  , y  , w, c);
  H_line(cmds, x  , y+h, w, c);
  V_line(cmds, x  , y  , h, c);
  V_line(cmds, x+w, y  , h, c);
}

void Rectf(lcd_cmds& cmds,
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

void LCD_Clear(lcd_cmds& cmds, uint16_t j)                   
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

void setup()
{
    // Trying to initialize SPI
#if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning spi/spi_master example requires a board with SPI pins
    puts("Default SPI pins were not defined");
#else
    printf("SPI master example\n");

    // Enable SPI 0 at 62.5 MHz and connect to GPIOs
    spi_init( spi_default , 62.5 * 1000 * 1000);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_4pins_with_func(
            PICO_DEFAULT_SPI_RX_PIN, 
            PICO_DEFAULT_SPI_TX_PIN, 
            PICO_DEFAULT_SPI_SCK_PIN, 
            PICO_DEFAULT_SPI_CSN_PIN, 
            GPIO_FUNC_SPI));
#endif
    // Here SPI initialization is finished

    // Initialize additional pins that would drive lcd
    gpio_init(RESET);
    gpio_set_dir(RESET, GPIO_OUT);
    gpio_init(RS);
    gpio_set_dir(RS, GPIO_OUT);

    gpio_put(RESET, 1);
    gpio_put(RS, 1);

    printf("%s\n", gpio_get(RESET) ? "HIGH" : "LOW");
    printf("%s\n", gpio_get(RS) ? "HIGH" : "LOW");

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

bool get_random_bit() {
    return rosc_hw->randombit;
}

template<typename T>
T random(T limit){
    T result = 0;
    for (uint8_t i = 0; i < sizeof(T)*8; ++i){
        result += (get_random_bit() << i);
    }
    return result % limit;
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
    setup();

    // // Prepare buffers for SPI communications
    // uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];
    // // Initialize output buffer
    // for (size_t i = 0; i < BUF_LEN; ++i) {
    //     out_buf[i] = i;
    // }

    // sleep_ms(2 * 1000);
    // printf("SPI master says: The following buffer will be written to MOSI endlessly:\n");
    // printbuf(out_buf, BUF_LEN);

    // for (size_t i = 0; ; ++i) {
    //     // Write the output buffer to MOSI, and at the same time read from MISO.
    //     spi_write_read_blocking(spi_default, out_buf, in_buf, BUF_LEN);

    //     // Write to stdio whatever came in on the MISO line.
    //     printf("SPI master says: read page %d from the MISO line:\n", i);
    //     printbuf(in_buf, BUF_LEN);

    //     // Sleep for ten seconds so you get a chance to read the output.
    //     sleep_ms(10 * 1000);
    // }

    printf("Prepare cmd buffer for drawing\n");
    lcd_cmds draw_cmds(spi_default);
    printf("Enter main loop\n");
    uint16_t x = 0, y = 0, w = 0, h = 0;
    for (size_t i = 0; ; ++i) {
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
        for(int i=0;i<500;i++) {
            x = random<uint16_t>(20, 150);
            w = random<uint16_t>(150, 300) - x;
            y = random<uint16_t>(20, 120);
            h = random<uint16_t>(120, 220) - y;
            printf("Drawing rect {{%d, %d, %d, %d}}\n", x, y, w, h);
            Rect(draw_cmds, 
                x,
                y,
                w,
                h,
                random<uint16_t>(65535)); // rectangle at x, y, with, hight, color
            draw_cmds.execute();

        }
    }
    
    return 0;
}
