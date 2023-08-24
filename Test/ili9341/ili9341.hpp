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


/*! \file ili9341.hpp
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
#define ILI9341_NOP    0x00





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
#define ILI9341_SWRESET    0x01





/*! \def ILI9341_RDDIDIF
 *  \brief Read Display Identification Information
 *  \return first_byte is dummy data.
 *  \return second_byte (ID1 [7:0]): LCD module’s manufacturer ID.
 *  \return third_byte  (ID2 [7:0]): LCD module/driver version ID.
 *  \return fourth_byte (ID3 [7:0]): LCD module/driver ID.
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 * 
 * This read byte returns 24 bits display identification information.
 * The 1-st parameter is dummy data.
 * The 2-nd parameter (ID1 [7:0]): LCD module’s manufacturer ID.
 * The 3-rd parameter (ID2 [7:0]): LCD module/driver version ID.
 * The 4-th parameter (ID3 [7:0]): LCD module/driver ID.
 * 
 * |      04h       | RDDIDIF (Read Display Identification Information) |||||||||||||
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * |                |  D/CX   |  RDX  |  WRX  | D17-8 |  D7  |  D6  |  D5  |  D4  |  D3  |  D2  |  D1  |  D0  |  HEX  |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * |    Command     |    0    |   1   |   ↑   |  XX   |  0   |  0   |  0   |  0   |  0   |  1   |  0   |  0   |  04h  |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | 1-st Parameter |    1    |   1   |   ↑   |  XX   |  X   |  X   |  X   |  X   |  X   |  X   |  X   |  X   |   X   |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | 2-nd Parameter |    1    |   1   |   ↑   |  XX   |                    ID1[7:0]                    ||||||||  XX   |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | 3-rd Parameter |    1    |   1   |   ↑   |  XX   |                    ID2[7:0]                    ||||||||  XX   |
 * |----------------|---------|-------|-------|-------|------|------|------|------|------|------|------|------|-------|
 * | 4-th Parameter |    1    |   1   |   ↑   |  XX   |                    ID3[7:0]                    ||||||||  XX   |
 */
#define ILI9341_RDDIDIF    0x04





/*! \def ILI9341_RDDST
 *  \brief Read Display Status
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDDST    0x09





/*! \def ILI9341_RDDPM
 *  \brief Read Display Power Mode
 */
#define ILI9341_RDDPM    0x0A





/*! \def ILI9341_RDDMADCTL
 *  \brief Read Display MADCTL
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDDMADCTL    0x0B





/*! \def ILI9341_RDPF
 *  \brief Read Display Pixel Format
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDPF    0x0C





/*! \def ILI9341_RDIF
 *  \brief Read Display Image Format
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDIF    0x0D





/*! \def ILI9341_RDSM 
 *  \brief Read Display Signal Mode
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDSM    0x0E





/*! \def ILI9341_RDSDR
 *  \brief Read Display Self Diagnostic Result
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDSDR    0x0F





/*! \def ILI9341_ESM
 *  \brief Enter Sleep Mode
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_ESM    0x10





/*! \def ILI9341_SO
 *  \brief Sleep OUT
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_SO    0x11





/*! \def ILI9341_PMON
 *  \brief Partial Mode ON
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_PMON    0x12





/*! \def ILI9341_NDMON
 *  \brief Normal Display Mode ON
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_NDMON    0x13





/*! \def ILI9341_DIOFF
 *  \brief Display Inversion OFF
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_DIOFF    0x20





/*! \def ILI9341_DION
 *  \brief Display Inversion ON
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_DION    0x21





/*! \def ILI9341_GS
 *  \brief Display Gamma Set
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_GS    0x26





/*! \def ILI9341_DOFF
 *  \brief Display OFF
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_DOFF    0x28





/*! \def ILI9341_DON
 *  \brief Display ON
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_DON    0x29





/*! \def ILI9341_CAS
 *  \brief Column Address Set
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_CAS    0x2A





/*! \def ILI9341_PAS
 *  \brief Page Address Set
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_PAS    0x2B





/*! \def ILI9341_MW
 *  \brief Memory Write
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_MW    0x2C





/*! \def ILI9341_CS
 *  \brief Color Set
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_CS    0x2D





/*! \def ILI9341_MR
 *  \brief Memory Read
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_MR    0x2E





/*! \def ILI9341_PA
 *  \brief Partial Area
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_PA    0x30





/*! \def ILI9341_VSD
 *  \brief Vertical Scrolling Definition
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_VSD    0x33





/*! \def ILI9341_TELOFF
 *  \brief Tearing Effect Line OFF
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_TELOFF    0x34





/*! \def ILI9341_TELON
 *  \brief Tearing Effect Line ON
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_TELON   0x35





/*! \def ILI9341_MAS
 *  \brief Memory Access Control
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_MAS   0x36





/*! \def ILI9341_VSSA
 *  \brief Vertical Scrolling Start Address
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_VSSA  0x37





/*! \def ILI9341_IMOFF
 *  \brief Idle Mode OFF
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_IMOFF  0x38





/*! \def ILI9341_IMON
 *  \brief Idle Mode ON
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_IMON  0x39





/*! \def ILI9341_PFS
 *  \brief Pixel Format Set
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_PFS  0x3A





/*! \def ILI9341_WMC
 *  \brief Write Memory Continue
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_WMC  0x3C





/*! \def ILI9341_RMC
 *  \brief Read Memory Continue
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RMC  0x3E





/*! \def ILI9341_STS
 *  \brief Set Tear Scanline
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_STS  0x44





/*! \def ILI9341_GTSC
 *  \brief Get Scanline
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_GTSC  0x45





/*! \def ILI9341_WDB
 *  \brief Write Display Brightness
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_WDB  0x51





/*! \def ILI9341_RDB
 *  \brief Read Display Brightness
 *  \ingroup ili9341
 *  \ingroup ili9341_regulative_cmd_set
 */
#define ILI9341_RDB  0x52