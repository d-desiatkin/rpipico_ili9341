/* Copyright 2023 Desiatkin Dmitrii
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


/*! \file ili9341_cmd_regs.h
 *  \author Desiatkin Dmitrii
 *  \brief Command register defenitions of ILI9341 LCD driver
 *  \defgroup ili9341 ili9341
 *  
 * All information was written by hand followiing manufacturer's manual
 * ILI9341_DS_v1.09_20110315
 * 
 * Pin Functions (I, I/O, O directions):
 * RESX I - This signal will reset the device and must be applied to properly initialize the chip. Signal is active low.
 * 
 * EXTC I - Extended command set enable. Low: extended command set is discarded. High: extended command set is accepted.
 * 
 * CSX  I - Chip select input pin (“Low” enable). This pin can be permanently fixed “Low” in MPU interface mode on.
 * 
 * D/CX (SCL) I - This pin is used to select “Data or Command” in the parallel interface or 4-wire 8-bit serial data interface.
 *                When DCX = ’1’, data is selected. When DCX = ’0’, command is selected. 
 *                This pin is used serial interface clock in 3-wire 9-bit / 4-wire 8-bit serial data interface.
 *                If not used, this pin should be connected to VDDI or VSS.
 * 
 * RDX I -  8080-I/8080-II system (RDX): Serves as a read signal and MCU read data at the rising edge. 
 *          Fix to VDDI level when not used
 * 
 * WRX(D/CX) I - 8080-I/8080-II system (WRX): Serves as a write signal and writes data at the rising edge.
 *               4-line system (D/CX): Serves as command or parameter select.
 *               Fix to VDDI level when not used
 * 
 * D[17:0] I/O - 18-bit parallel bi-directional data bus for MCU system and RGB interface mode
 *               Fix to VSS level when not in use
 * 
 * SDI/SDA I/O - When IM[3] : Low, Serial in/out signal. When IM[3] : High, Serial input signal.
 *               The data is applied on the rising edge of the SCL signal.
 *               If not used, fix this pin at VDDI or VSS.
 * 
 * SDO O - Serial output signal. The data is outputted on the falling edge of the SCL signal.
 *         If not used, open this pin
 * 
 * TE  O - Tearing effect output pin to synchronize MPU to frame writing, activated by S/W command. 
 *         When this pin is not activated, this pin is low.
 *         If not used, open this pin.
 * 
 * DOTCLK I - Dot clock signal for RGB interface operation.
 *            Fix to VDDI or VSS level when not in use.
 * 
 * VSYNC  I - Frame synchronizing signal for RGB interface operation.
 *            Fix to VDDI or VSS level when not in use.
 * 
 * HSYNC  I - Line synchronizing signal for RGB interface operation.
 *            Fix to VDDI or VSS level when not in use.
 * 
 * DE     I - Data enable signal for RGB interface operation.
 *            Fix to VDDI or VSS level when not in use.
 * 
*/

#ifndef _ILI9341_CMD_REGS_H
#define _ILI9341_CMD_REGS_H
 
 
/******************************
 *   Regulative Command Set   *
 ******************************/
//! \defgroup ili9341_regulative_cmd_set ili9341_regulative_cmd_set

/*! \def ILI9341_NOP
 *  \brief No Operation
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 * 
 * This command is an empty command; it does not have any effect on the display module.
 * However it can be used to terminate Frame Memory Write or Read as described
 * in RAMWR (Memory Write) and RAMRD (Memory Read) Commands. X = Don’t care.
 * 
 * |   00h     | NOP (No Operation) |||||||||||||
 * |-----------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * |           |  D/CX   |  RDX  |  WRX  | D17-8 |  D7  |  D6  |  D5  |  D4  |  D3  |  D2  |  D1  |  D0  |  HEX  |
 * |-----------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * |  Command  |    0    |   1   |   ↑   |  XX   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  00h  |
 * |-----------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | Parameter | No Parameter. |||||||||||||
 * 
 */
#define ILI9341_NOP           0x00





/*! \def ILI9341_SWRESET
 *  \brief Software Reset.
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 * 
 * When the Software Reset command is written, it causes a software reset.
 * It resets the commands and parameters to their S/W Reset default values.
 * (See default tables in each command description.) X = Don’t care.
 * 
 * |   01h     | SWRESET |||||||||||||
 * |-----------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * |           |  D/CX   |  RDX  |  WRX  | D17-8 |  D7  |  D6  |  D5  |  D4  |  D3  |  D2  |  D1  |  D0  |  HEX  |
 * |-----------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * |  Command  |    0    |   1   |   ↑   |  XX   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  1   |  01h  |
 * |-----------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | Parameter | No Parameter. |||||||||||||
 * 
 * \note
 * The Frame Memory contents are unaffected by this command. 
 * It will be necessary to wait 5msec before sending new command following software reset.
 * The display module loads all display supplier factory default values to the registers during this 5msec.
 * If Software Reset is applied during Sleep Out mode, it will be necessary to wait 120msec before sending Sleep out command.
 * Software Reset Command cannot be sent during Sleep Out sequence.
 */
#define ILI9341_SWRESET       0x01





/*! \def ILI9341_RDDIDIF
 *  \brief Read Display Identification Information
 *  \return 4 bytes of data with meaningfull 24 bits see detailed description
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 * 
 * This read byte returns 24 bits display identification information.
 * The 1-st parameter is dummy data.
 * The 2-nd parameter (ID1 [7:0]): LCD module’s manufacturer ID.
 * The 3-rd parameter (ID2 [7:0]): LCD module/driver version ID.
 * The 4-th parameter (ID3 [7:0]): LCD module/driver ID.
 * 
 * |      04h       |                  RDDIDIF (Read Display Identification Information)                  |||||||||||||
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * |                |  D/CX   |  RDX  |  WRX  | D17-8 |  D7  |  D6  |  D5  |  D4  |  D3  |  D2  |  D1  |  D0  |  HEX  |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * |    Command     |    0    |   1   |   ↑   |  XX   |  0   |  0   |  0   |  0   |  0   |  1   |  0   |  0   |  04h  |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | 1-st Parameter |    1    |   ↑   |   1   |  XX   |  X   |  X   |  X   |  X   |  X   |  X   |  X   |  X   |   X   |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | 2-nd Parameter |    1    |   ↑   |   1   |  XX   |                    ID1[7:0]                    ||||||||  XX   |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | 3-rd Parameter |    1    |   ↑   |   1   |  XX   |                    ID2[7:0]                    ||||||||  XX   |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | 4-th Parameter |    1    |   ↑   |   1   |  XX   |                    ID3[7:0]                    ||||||||  XX   |
 */
#define ILI9341_RDDIDIF       0x04





/*! \def ILI9341_RDDST
 *  \brief Read Display Status
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 * 
 * This command indicates the current status of the display as described in the table below:
 * D[31:25]: D31 - Booster voltage status (0 Booster OFF || 1 Booster ON)
 *           D30 - Row address order ( 0 Top to Bottom (When MADCTL B7=’0’) || 1 Bottom to Top (When MADCTL B7=’1’) )
 *           D29 - Column address order ( 0 Left to Right (When MADCTL B6=’0’) || 1 Right to Left (When MADCTL B6=’1’) )
 *           D28 - Row/column exchange ( 0 Normal Mode (When MADCTL B5=’0’) || 1 Reverse Mode (When MADCTL B5=’1’) )
 *           D27 - Vertical refresh ( 0 LCD Refresh Top to Bottom (When MADCTL B4=’0’) || 1 LCD Refresh Bottom to Top (When MADCTL B4=’1’) )
 *           D26 - RGB/BGR order ( 0 RGB (When MADCTL B3=’0’) || 1 BGR (When MADCTL B3=’1’) )
 *           D25 - Horizontal refresh order ( 0 LCD Refresh Left to Right (When MADCTL B2=’0’) || 1 LCD Refresh Right to Left (When MADCTL B2=’1’) )
 * D[24:23]: Not used (Allways Zero)
 * D[22:20]: Interface color pixel format definition ( 101 16-bit/pixel || 110 18-bit/pixel )
 * D[19:16]: D19 - Idle mode ON/OFF           ( 1/0 )
 *           D18 - Partial mode ON/OFF        ( 1/0 )
 *           D17 - Sleep IN/OUT               ( 0/1 ) <- inverse behaviour here !!!
 *           D16 - Display normal mode ON/OFF ( 1/0 )
 * D[15:11]: D15 - Scroll Off (Allways zero) // Others have description but it seems that they unused in that MCU
 * D[10:8] : D10 - Display ON/OFF             ( 1/0 )
 *           D9  - Tearing effect line ON/OFF ( 1/0 )
 * D[8:6]  : Gamma curve selection ( 000 = GC0 || 001 = --- || 010 = --- || 011 = --- || other == Not defined )
 * D[5]    : Tearing effect line mode ( '0': "Mode 1, V-Blanking only" || '1': "Mode 2, both H-Blanking and V-Blanking" )
 * D[4:0]  : Not used (Allways Zero)
 * 
 * |      09h       | RDDST (Read Display Status) |||||||||||||
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * |                |  D/CX   |  RDX  |  WRX  | D17-8 |  D7  |  D6  |  D5  |  D4  |  D3  |  D2  |  D1  |  D0  |  HEX  |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * |    Command     |    0    |   1   |   ↑   |  XX   |  0   |  0   |  0   |  0   |  0   |  1   |  0   |  0   |  09h  |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | 1-st Parameter |    1    |   ↑   |   1   |  XX   |  X   |  X   |  X   |  X   |  X   |  X   |  X   |  X   |   X   |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | 2-nd Parameter |    1    |   ↑   |   1   |  XX   |                    D[31:25]              |||||||  0   |  00   |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | 3-rd Parameter |    1    |   ↑   |   1   |  XX   |  0   |      D[22:20]    |||          D[19:16]      ||||  61   |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | 4-th Parameter |    1    |   ↑   |   1   |  XX   |  0   |  0   |  0   |  0   |  0   |      D[10:8]     |||  00   |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | 5-th Parameter |    1    |   ↑   |   1   |  XX   |       D[7:5]     |||  0   |  0   |  0   |  0   |  0   |  00   |
 */
#define ILI9341_RDDST         0x09





/*! \def ILI9341_RDDPM
 *  \brief Read Display Power Mode
 */
#define ILI9341_RDDPM         0x0A





/*! \def ILI9341_RDDMADCTL
 *  \brief Read Display MADCTL
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDDMADCTL     0x0B





/*! \def ILI9341_RDDCOLMOD
 *  \brief Read Display Pixel Format
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDDCOLMOD     0x0C





/*! \def ILI9341_RDDIM
 *  \brief Read Display Image Mode
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDDIM         0x0D





/*! \def ILI9341_RDDSM 
 *  \brief Read Display Signal Mode
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDDSM         0x0E





/*! \def ILI9341_RDDSDR
 *  \brief Read Display Self Diagnostic Result
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDDSDR        0x0F





/*! \def ILI9341_SLPIN
 *  \brief Enter Sleep Mode
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_SLPIN         0x10





/*! \def ILI9341_SLPOUT
 *  \brief Sleep OUT
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_SLPOUT        0x11





/*! \def ILI9341_PTLON
 *  \brief Partial Mode ON
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_PTLON         0x12





/*! \def ILI9341_NORON
 *  \brief Normal Display Mode ON
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_NORON         0x13





/*! \def ILI9341_DINVOFF
 *  \brief Display Inversion OFF
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_DINVOFF       0x20





/*! \def ILI9341_DINVON
 *  \brief Display Inversion ON
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_DINVON        0x21





/*! \def ILI9341_GAMSET
 *  \brief Display Gamma Set
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_GAMSET        0x26





/*! \def ILI9341_DISPOFF
 *  \brief Display OFF
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_DISPOFF       0x28





/*! \def ILI9341_DISPON
 *  \brief Display ON
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_DISPON        0x29





/*! \def ILI9341_CASET
 *  \brief Column Address Set
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_CASET         0x2A





/*! \def ILI9341_PASET
 *  \brief Page Address Set
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_PASET         0x2B





/*! \def ILI9341_RAMWR
 *  \brief Memory Write
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RAMWR         0x2C





/*! \def ILI9341_RGBSET
 *  \brief Color Set
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RGBSET        0x2D





/*! \def ILI9341_RAMRD
 *  \brief Memory Read
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RAMRD         0x2E





/*! \def ILI9341_PLTAR
 *  \brief Partial Area
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_PLTAR         0x30





/*! \def ILI9341_VSCRDEF
 *  \brief Vertical Scrolling Definition
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_VSCRDEF       0x33





/*! \def ILI9341_TEOFF
 *  \brief Tearing Effect Line OFF
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_TEOFF         0x34





/*! \def ILI9341_TELON
 *  \brief Tearing Effect Line ON
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_TEON          0x35





/*! \def ILI9341_MADCTL
 *  \brief Memory Access Control
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_MADCTL        0x36





/*! \def ILI9341_VSCRSADD
 *  \brief Vertical Scrolling Start Address
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_VSCRSADD      0x37





/*! \def ILI9341_IDMOFF
 *  \brief Idle Mode OFF
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_IDMOFF        0x38





/*! \def ILI9341_IDMON
 *  \brief Idle Mode ON
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_IDMON         0x39





/*! \def ILI9341_PIXSET
 *  \brief Pixel Format Set
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_PIXSET        0x3A





/*! \def ILI9341_Write_Memory_Continue
 *  \brief Write Memory Continue
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_Write_Memory_Continue   0x3C





/*! \def ILI9341_Read_Memory_Continue
 *  \brief Read Memory Continue
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_Read_Memory_Continue    0x3E





/*! \def ILI9341_Set_Tear_Scanline
 *  \brief Set Tear Scanline
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_Set_Tear_Scanline       0x44





/*! \def ILI9341_Get_Scanline
 *  \brief Get Scanline
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_Get_Scanline            0x45





/*! \def ILI9341_WRDISBV
 *  \brief Write Display Brightness
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_WRDISBV       0x51





/*! \def ILI9341_RDB
 *  \brief Read Display Brightness Value
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDDISBV       0x52





/*! \def ILI9341_WRCTRLD
 *  \brief Write Control Display
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_WRCTRLD       0x53





/*! \def ILI9341_RDCTRLD
 *  \brief Read Control Display
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDCTRLD       0x54





/*! \def ILI9341_WRCABC
 *  \brief Write Content Adaptive Brightness Control
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_WRCABC        0x55





/*! \def ILI9341_RDCABC
 *  \brief Read Content Adaptive Brightness Control
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDCABC        0x56





/*! \def ILI9341_Write_Backlight_Control_1
 *  \brief Write CABC Minimum Brightness
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_Write_Backlight_Control_1    0x5E





/*! \def ILI9341_Read_Backlight_Control_1
 *  \brief Read CABC Minimum Brightness
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_Read_Backlight_Control_1     0x5F





/*! \def ILI9341_RDID1
 *  \brief Read ID1 Module’s Manufacture
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDID1         0xDA





/*! \def ILI9341_RDID2
 *  \brief Read ID2 LCD Module / Driver Version
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDID2         0xDB





/*! \def ILI9341_RDID3
 *  \brief Read ID3 LCD Module / Driver ID
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDID3         0xDC










/******************************
 *    Extended Command Set    *
 ******************************/
//! \defgroup ili9341_extended_cmd_set ili9341_extended_cmd_set

/*! \def ILI9341_IFMODE
 *  \brief Interface Mode Control
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_IFMODE        0xB0





/*! \def ILI9341_FRMCTR1
 *  \brief Frame Rate Control (In Normal Mode / Full colors)
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_FRMCTR1       0xB1





/*! \def ILI9341_FRMCTR2
 *  \brief Frame Rate Control (In Idle Mode / 8l colors)
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_FRMCTR2       0xB2





/*! \def ILI9341_FRMCTR3
 *  \brief Frame Rate Control (In Partial Mode / Full colors)
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_FRMCTR3       0xB3





/*! \def ILI9341_INVTR
 *  \brief Display Inversion Control
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_INVTR         0xB4





/*! \def ILI9341_PRCTR
 *  \brief Blanking Porch Control
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_PRCTR         0xB5





/*! \def ILI9341_DISCTRL
 *  \brief Display Function Control
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_DISCTRL       0xB6





/*! \def ILI9341_ETMOD
 *  \brief Entry Mode Set
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_ETMOD         0xB7





/*! \def ILI9341_Backlight_Control_1
 *  \brief Backlight Control 1
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_Backlight_Control_1     0xB8





/*! \def ILI9341_Backlight_Control_2
 *  \brief Backlight Control 2
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_Backlight_Control_2     0xB9





/*! \def ILI9341_Backlight_Control_3
 *  \brief Backlight Control 3
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_Backlight_Control_3     0xBA





/*! \def ILI9341_Backlight_Control_4
 *  \brief Backlight Control 4
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_Backlight_Control_4     0xBB





/*! \def ILI9341_Backlight_Control_5
 *  \brief Backlight Control 5
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_Backlight_Control_5     0xBC





/*! \def ILI9341_Backlight_Control_7
 *  \brief Backlight Control 7
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_Backlight_Control_7     0xBE





/*! \def ILI9341_Backlight_Control_8
 *  \brief Backlight Control 8
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_Backlight_Control_8     0xBF





/*! \def ILI9341_PWCTRL_1
 *  \brief Power Control 1
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_PWCTRL_1      0xC0





/*! \def ILI9341_PWCTRL_2
 *  \brief Power Control 2
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_PWCTRL_2      0xC1





/*! \def ILI9341_VMCTRL1
 *  \brief VCOM Control 1
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_VMCTRL1       0xC5





/*! \def ILI9341_VMCTRL2
 *  \brief VCOM Control 2
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_VMCTRL2      0xC7





/*! \def ILI9341_NVMWR
 *  \brief NV Memory Write
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_NVMWR         0xD0





/*! \def ILI9341_NVMPKEY
 *  \brief NV Memory Protection Key
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_NVMPKEY       0xD1





/*! \def ILI9341_RDNVM
 *  \brief NV Memory Status Read
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_RDNVM         0xD2





/*! \def ILI9341_RDID4
 *  \brief Read ID4
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_RDID4         0xD3





/*! \def ILI9341_PGAMCTRL
 *  \brief Positive Gamma Control
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_PGAMCTRL      0xE0





/*! \def ILI9341_NGAMCTRL
 *  \brief Negative Gamma Correction
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_NGAMCTRL      0xE1





/*! \def ILI9341_DGAMCTRL1
 *  \brief Digital Gamma Control 1
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_DGAMCTRL1     0xE2





/*! \def ILI9341_DGAMCTRL2
 *  \brief Digital Gamma Control 2
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_DGAMCTRL2     0xE3





/*! \def ILI9341_IFCTL
 *  \brief 16bits Data Format Selection
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_IFCTL         0xF6





/*! \def ILI9341_Power_control_A
 *  \brief Power control A
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_Power_control_A         0xCB





/*! \def ILI9341_Power_control_B
 *  \brief Power control B
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_Power_control_B         0xCF





/*! \def ILI9341_Driver_timing_control_A
 *  \brief Driver timing control A
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_Driver_timing_control_A           0xE8





/*! \def ILI9341_Driver_timing_control_B
 *  \brief Driver timing control B
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_Driver_timing_control_B           0xEA





/*! \def ILI9341_Power_on_sequence_control
 *  \brief Power on sequence control
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_Power_on_sequence_control         0xED





/*! \def ILI9341_Enable_3G
 *  \brief Enable 3G
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_Enable_3G     0xF2





/*! \def ILI9341_Pump_ratio_control
 *  \brief Pump ratio control
 *  \ingroup ili9341
 *  \ingroup ili9341_extended_cmd_set
 */
#define ILI9341_Pump_ratio_control      0xF7

#endif // _ILI9341_CMD_REGS_H