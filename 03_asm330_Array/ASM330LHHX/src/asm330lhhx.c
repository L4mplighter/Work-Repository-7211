#include "asm330lhhx.h"
#include "ATSpi.h"
#include "wk_system.h"

static void asm330lhhx_byte_read(spi_type* spi_x, uint8_t reg, uint8_t *data);
static void asm330lhhx_byte_write(spi_type* spi_x, uint8_t reg, uint8_t data);
static void asm330lhhx_read(spi_type* spi_x, uint8_t reg, uint8_t *data, uint8_t sensorNum);
static void asm330lhhx_write(spi_type* spi_x, uint8_t reg, uint8_t data, uint8_t sensorNum);
static void asm330lhhx_mutiRead(spi_type* spi_x, uint8_t reg, uint8_t *data, uint8_t dataSize, uint8_t sensorNum);
static int asm330lhhx_id(void);

ASM_StatusTypeDef asm330lhhx_init(void) {
    P_AOUT(1) = 1;P_AOUT(2) = 1;P_AOUT(3) = 1;P_AOUT(4) = 1;
    if (asm330lhhx_id()) return ASM_ERROR;
    asm330lhhx_write(SPI1, CTRL3_C, 0B00000001, ASM330_SENSOR_1);/* RESET SENSOR 1 */
    asm330lhhx_write(SPI1, CTRL3_C, 0B00000001, ASM330_SENSOR_2);/* RESET SENSOR 2 */
    asm330lhhx_write(SPI1, CTRL3_C, 0B00000001, ASM330_SENSOR_3);/* RESET SENSOR 3 */
    asm330lhhx_write(SPI1, CTRL3_C, 0B00000001, ASM330_SENSOR_4);/* RESET SENSOR 4 */
    wk_delay_ms(10);
    for (int i = 1; i <= SENSOR_NUM; i++) {
        asm330lhhx_write(SPI1, CTRL3_C, 0B01000100, i);/* SET REGISTER ADDRESS AUTOMATICALLY INC; SET BDU: 1*/
        asm330lhhx_write(SPI1, CTRL1_XL, 0B01010100, i);/* ACC ODR: 208hz; ACC RANGE: ±16g; NO LPF2 */
        asm330lhhx_write(SPI1, CTRL2_G, 0B01010100, i);/* GYRO ODR: 208hz; GYRO RANGE: ±500dps */
        asm330lhhx_write(SPI1, CTRL4_C, 0B00000010, i);/* SET LPF1 FOR GYRO */
        asm330lhhx_write(SPI1, CTRL5_C, 0B00000000, i);/* NO WRAPAROUND; NO SELF-TEST */
        asm330lhhx_write(SPI1, CTRL6_C, 0B00000000, i);/* SET ACC HIGH-PERFORMANCE; SET GYRO LPF1 BANDWIDTH: 67hz */
        asm330lhhx_write(SPI1, CTRL7_G, 0B00000000, i);/* SET GYRO HIGH-PERFORMANCE; NO HPF */
        asm330lhhx_write(SPI1, CTRL8_XL, 0B00000000, i);/* NO LPF AND HPF; LOW PASS ON ODR/2 */ 
    }
    return ASM_OK;
}

__INLINE static void asm330lhhx_byte_read(spi_type* spi_x, uint8_t reg, uint8_t *data) {
    uint8_t DUMMY = 0xff;
    at_spi_transmit_receive8(spi_x, reg);
    *data = at_spi_transmit_receive8(spi_x, DUMMY);
 }

__INLINE static void asm330lhhx_byte_write(spi_type* spi_x, uint8_t reg, uint8_t data) {
    at_spi_transmit_receive8(spi_x, reg);
    at_spi_transmit_receive8(spi_x, data);
}

static void asm330lhhx_read(spi_type* spi_x, uint8_t reg, uint8_t *data, uint8_t sensorNum) {
    uint8_t DUMMY = 0xff;
    uint8_t trash = 0;
    switch (sensorNum) {
        case ASM330_SENSOR_1:
            ASM330CS1_L;
            asm330lhhx_byte_read(spi_x, ASM330READ(reg), data);
            ASM330CS1_H;
            break;
        case ASM330_SENSOR_2:
            ASM330CS2_L;
            asm330lhhx_byte_read(spi_x, ASM330READ(reg), data);
            ASM330CS2_H;
            break;
        case ASM330_SENSOR_3:
            ASM330CS3_L;
            asm330lhhx_byte_read(spi_x, ASM330READ(reg), data);
            ASM330CS3_H;
            break;
        case ASM330_SENSOR_4:
            ASM330CS4_L;
            asm330lhhx_byte_read(spi_x, ASM330READ(reg), data);
            ASM330CS4_H;
            break;
        default:
            break;
    }
}

static void asm330lhhx_write(spi_type* spi_x, uint8_t reg, uint8_t data, uint8_t sensorNum) {
    switch (sensorNum) {
        case ASM330_SENSOR_1:
            ASM330CS1_L;
            asm330lhhx_byte_write(spi_x, ASM330WRITE(reg), data);
            ASM330CS1_H; 
            break;
        case ASM330_SENSOR_2:
            ASM330CS2_L;
            asm330lhhx_byte_write(spi_x, ASM330WRITE(reg), data);
            ASM330CS2_H; 
            break;
        case ASM330_SENSOR_3:
            ASM330CS3_L;
            asm330lhhx_byte_write(spi_x, ASM330WRITE(reg), data);
            ASM330CS3_H; 
            break;
        case ASM330_SENSOR_4:
            ASM330CS4_L;
            asm330lhhx_byte_write(spi_x, ASM330WRITE(reg), data);
            ASM330CS4_H; 
            break;
        default:
            break;
     }
    
}

static void asm330lhhx_mutiRead(spi_type* spi_x, uint8_t reg, uint8_t *data, uint8_t dataSize, uint8_t sensorNum) {
    uint8_t DUMMY = 0xff;
    switch (sensorNum) {
        case ASM330_SENSOR_1:
            ASM330CS1_L;
            at_spi_transmit_receive8(spi_x, ASM330READ(reg));
            for (int i = 0; i < dataSize; i++) {
                data[i] = at_spi_transmit_receive8(spi_x, DUMMY);
            }
            ASM330CS1_H; 
            break;
        case ASM330_SENSOR_2:
            ASM330CS2_L;
            at_spi_transmit_receive8(spi_x, ASM330READ(reg));
            for (int i = 0; i < dataSize; i++) {
                data[i] = at_spi_transmit_receive8(spi_x, DUMMY);
            }
            ASM330CS2_H; 
            break;
        case ASM330_SENSOR_3:
            ASM330CS3_L;
            at_spi_transmit_receive8(spi_x, ASM330READ(reg));
            for (int i = 0; i < dataSize; i++) {
                data[i] = at_spi_transmit_receive8(spi_x, DUMMY);
            }
            ASM330CS3_H; 
            break;
        case ASM330_SENSOR_4:
            ASM330CS4_L;
            at_spi_transmit_receive8(spi_x, ASM330READ(reg));
            for (int i = 0; i < dataSize; i++) {
                data[i] = at_spi_transmit_receive8(spi_x, DUMMY);
            }
            ASM330CS4_H; 
            break;
        default:
            break;
    }
}

static int asm330lhhx_id(void) {
    for(int i = 1; i <= SENSOR_NUM; i++) {
        uint8_t id = 0;
        asm330lhhx_read(ASM330PORT, WHO_AM_I, &id, i);
        if (id != ASM330_ID) {
            return i;
        }
        wk_delay_ms(5);
    }
    return 0;
}

void asm330lhhx_data(spi_type* spi_x, ASM330DATAOP *asmdata, uint8_t sensorNum) {
    ASM330DATARAW rawData;
    asm330lhhx_mutiRead(spi_x, OUT_TEMP_L, (uint8_t *)&rawData, sizeof(rawData), sensorNum);
    asmdata->temp = (float)((int16_t)(rawData.out_temp_h << 8 | rawData.out_temp_l) / 256.0f) + 25.0f;
    asmdata->gyro[0] = (float)((int16_t)(rawData.outx_h_g << 8 | rawData.outx_l_g) * 17.5f) / 1000.0f;
    asmdata->gyro[1] = (float)((int16_t)(rawData.outy_h_g << 8 | rawData.outy_l_g) * 17.5f) / 1000.0f;
    asmdata->gyro[2] = (float)((int16_t)(rawData.outz_h_g << 8 | rawData.outz_l_g) * 17.5f) / 1000.0f;
    asmdata->acc[0] = (float)((int16_t)(rawData.outx_h_a << 8 | rawData.outx_l_a) * 0.488f) / 1000.0f;
    asmdata->acc[1] = (float)((int16_t)(rawData.outy_h_a << 8 | rawData.outy_l_a) * 0.488f) / 1000.0f;
    asmdata->acc[2] = (float)((int16_t)(rawData.outz_h_a << 8 | rawData.outz_l_a) * 0.488f) / 1000.0f;
}


