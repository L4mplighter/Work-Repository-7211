#include "SCH16T.h"

static const uint8_t SCH16T_DUMMY_CMD[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 

SCH16T_HandleTypeDef sch16t_handle = {
    .SPIhandler = &hspi1,             
    .CS_GPIOport = GPIOA,             
    .CS_GPIOpin = GPIO_PIN_4,
    .RST_GPIOport  = GPIOB,
    .RST_GPIOpin = GPIO_PIN_0
};

uint64_t SCH16T_SendFrame(SCH16T_HandleTypeDef * SCH_Handle, uint64_t Request)
{
    uint64_t ReceivedData = 0;
    uint8_t txBuffer[6];
    uint8_t rxBuffer[6];
    uint8_t index;
    uint8_t size = 6; 
    
	for (index = 0; index < size; index++)
    {
        txBuffer[size - index - 1] = (Request >> (index << 3)) & 0xFF;
    }
    
    HAL_GPIO_WritePin(SCH_Handle->CS_GPIOport, SCH_Handle->CS_GPIOpin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(SCH_Handle->SPIhandler, txBuffer, rxBuffer, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SCH_Handle->CS_GPIOport, SCH_Handle->CS_GPIOpin, GPIO_PIN_SET);   

    for (index = 0; index < size; index++)
    {
    	ReceivedData |= (uint64_t)rxBuffer[index] << ((size - index - 1) << 3);
    }
    return ReceivedData;
}

uint64_t SCH16T_sendRequest(uint64_t request)
{
    // return SCH16T_SendFrame_316Bit(id, request);
    return SCH16T_SendFrame(&sch16t_handle, request);
}

void SCH16T_Reset(void)
{
    HAL_GPIO_WritePin(sch16t_handle.RST_GPIOport, sch16t_handle.RST_GPIOpin, GPIO_PIN_RESET);
    HAL_Delay(5);   
    HAL_GPIO_WritePin(sch16t_handle.RST_GPIOport, sch16t_handle.RST_GPIOpin, GPIO_PIN_SET);    
    HAL_Delay(10); 
}



/**
 * @brief isValid各种
 *
 * @param 
 * 
 * @return 
 *         
 */
bool SCH16T_isValidFilterFreq(uint32_t Freq)
{   
    if (Freq == 13 || Freq == 30 || Freq == 68 || Freq == 235 || Freq == 280 || Freq == 370 || Freq == SCH1_FILTER_BYPASS) 
        return true;
    else    
        return false;
}

bool SCH16T_isValidRateSens(uint32_t Sens)
{   
    if (Sens == 1600 || Sens == 3200 || Sens == 6400) 
        return true;
    else    
        return false;
}

bool SCH16T_isValidAccSens(uint32_t Sens)
{   
    if (Sens == 3200 || Sens == 6400 || Sens == 12800 || Sens == 25600) 
        return true;
    else    
        return false;
}

bool SCH16T_isValidDecimation(uint32_t Decimation)
{   
    if (Decimation == 2 || Decimation == 4 || Decimation == 8 || Decimation == 16 || Decimation == 32) 
        return true;
    else    
        return false;
}

bool SCH16T_isValidSampleRate(uint32_t Freq)
{   
    if ((Freq >= 1) && (Freq <= 10000))
        return true;

    return false;
}

bool SCH16T_check_48bit_frame_error(uint64_t *data, int size)
{
    for (int i = 0; i < size; i++) 
    {
        uint64_t value = data[i];
        if (value & ERROR_FIELD_MASK)
            return true;
    }
    
    return false;
}



/**
 * @brief Convert各种
 *
 * @param 
 * 
 * @return 
 *         
 */
uint32_t SCH16T_convertFilterToBitfield(uint32_t Freq)
{
    switch (Freq)
    {
        case 13:
            return 0x092;   // 010 010 010
        case 30:
            return 0x049;   // 001 001 001
        case 68:
            return 0x000;   // 000 000 000        
        case 235:
            return 0x16D;   // 101 101 101
        case 280:
            return 0x0DB;   // 011 011 011
        case 370:
            return 0x124;   // 100 100 100
        case SCH1_FILTER_BYPASS:
            return 0x1FF;   // 111 111 111, filter bypass mode
        default:
            return 0x000;
    }
}

uint32_t SCH16T_convertBitfieldToRateSens(uint32_t bitfield)
{
    switch (bitfield)
    {
        case 0x02:          // 010
            return 1600;
        case 0x03:          // 011
            return 3200;      
        case 0x04:          // 100
            return 6400;
        default:
            return 0x00;
    }
}

uint32_t SCH16T_convertRateSensToBitfield(uint32_t Sens)
{
    switch (Sens)
    {
        case 1600:
            return 0x02;   // 010
        case 3200:
            return 0x03;   // 011      
        case 6400:
            return 0x04;   // 100
        default:
            return 0x01;       
    }
}

uint32_t SCH16T_convertAccSensToBitfield(uint32_t Sens)
{
    switch (Sens)
    {
        case 3200:
            return 0x01;   // 001
        case 6400:
            return 0x02;   // 010
        case 12800:
            return 0x03;   // 011      
        case 25600:
            return 0x04;   // 100
        default:
            return 0x00;       
    }
}

uint32_t SCH16T_convertBitfieldToAccSens(uint32_t bitfield)
{
    switch (bitfield)
    {
        case 0x01:          // 001
            return 3200;
        case 0x02:          // 010
            return 6400;
        case 0x03:          // 011
            return 12800;      
        case 0x04:          // 100
            return 25600;
        default:
            return 0x00;       
    }
}


uint32_t SCH16T_convertDecimationToBitfield(uint32_t Decimation)
{
    switch (Decimation)
    {
        case 2:
            return 0x00;   // 001
        case 4:
            return 0x01;   // 010
        case 8:
            return 0x02;   // 011      
        case 16:
            return 0x03;   // 100
        case 32:
            return 0x04;   // 100
        default:
            return 0x00;       
    }
}

uint32_t SCH16T_convertBitfieldToDecimation(uint32_t bitfield)
{
    switch (bitfield)
    {
        case 0x00:      // 001
            return 2;
        case 0x01:      // 010
            return 4;
        case 0x02:      // 011
            return 8;      
        case 0x03:      // 100
            return 16;
        case 0x04:      // 100
            return 32;
        default:
            return 0x00;       
    }
}


/**
 * @brief Set各种
 *
 * @param 
 * 
 * @return 
 *         
 */
int SCH16T_setFilters(uint32_t Freq_Rate12, uint32_t Freq_Acc12, uint32_t Freq_Acc3)
{
    uint32_t dataField;
    uint64_t requestFrame_Rate12;
    uint64_t responseFrame_Rate12;
    uint64_t requestFrame_Acc12;
    uint64_t responseFrame_Acc12;
    uint64_t requestFrame_Acc3;
    uint64_t responseFrame_Acc3;
    uint8_t  CRCvalue;

    if (SCH16T_isValidFilterFreq(Freq_Rate12) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    if (SCH16T_isValidFilterFreq(Freq_Acc12) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    if (SCH16T_isValidFilterFreq(Freq_Acc3) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    
    // Set filters for Rate_XYZ1 (interpolated) and Rate_XYZ2 (decimated) outputs.
    requestFrame_Rate12 = REQ_SET_FILT_RATE;
    dataField = SCH16T_convertFilterToBitfield(Freq_Rate12);
    requestFrame_Rate12 |= dataField;
    requestFrame_Rate12 <<= 8;
    CRCvalue = CRC8(requestFrame_Rate12);
    requestFrame_Rate12 |= CRCvalue;
    SCH16T_sendRequest(requestFrame_Rate12);

    // Set filters for Acc_XYZ1 (interpolated) and Acc_XYZ2 (decimated) outputs.
    requestFrame_Acc12 = REQ_SET_FILT_ACC12;
    dataField = SCH16T_convertFilterToBitfield(Freq_Acc12);
    requestFrame_Acc12 |= dataField;
    requestFrame_Acc12 <<= 8;
    CRCvalue = CRC8(requestFrame_Acc12);
    requestFrame_Acc12 |= CRCvalue;
    SCH16T_sendRequest(requestFrame_Acc12);

    // Set filters for Acc_XYZ3 (interpolated) output.
    requestFrame_Acc3 = REQ_SET_FILT_ACC3;
    dataField = SCH16T_convertFilterToBitfield(Freq_Acc3);
    requestFrame_Acc3 |= dataField;
    requestFrame_Acc3 <<= 8;
    CRCvalue = CRC8(requestFrame_Acc3);
    requestFrame_Acc3 |= CRCvalue;
    SCH16T_sendRequest(requestFrame_Acc3);
    
    // Read back filter register contents.
    SCH16T_sendRequest(REQ_READ_FILT_RATE);        
    responseFrame_Rate12 = SCH16T_sendRequest(REQ_READ_FILT_ACC12);
    responseFrame_Acc12 = SCH16T_sendRequest(REQ_READ_FILT_ACC3);
    responseFrame_Acc3 = SCH16T_sendRequest(REQ_READ_FILT_ACC3);
    
    // Check that return frame is not blank.
    if ((responseFrame_Rate12 == 0xFFFFFFFFFFFF) || (responseFrame_Rate12 == 0x00))
        return SCH1_ERR_OTHER;
    if ((responseFrame_Acc12 == 0xFFFFFFFFFFFF) || (responseFrame_Acc12 == 0x00))
        return SCH1_ERR_OTHER;
    if ((responseFrame_Acc3 == 0xFFFFFFFFFFFF) || (responseFrame_Acc3 == 0x00))
        return SCH1_ERR_OTHER;
    
    // Check that Source Address matches Target Address.
    if (((requestFrame_Rate12 & TA_FIELD_MASK) >> 38) != ((responseFrame_Rate12 & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    if (((requestFrame_Acc12 & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc12 & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    if (((requestFrame_Acc3 & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc3 & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    
    // Check that read and written data match.
    if ((requestFrame_Rate12 & DATA_FIELD_MASK) != (responseFrame_Rate12 & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    if ((requestFrame_Acc12 & DATA_FIELD_MASK) != (responseFrame_Acc12 & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    if ((requestFrame_Acc3 & DATA_FIELD_MASK) != (responseFrame_Acc3 & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    
    return SCH1_OK;
}


int SCH16T_setRateSensDec(uint16_t Sens_Rate1, uint16_t Sens_Rate2, uint16_t Dec_Rate2)
{
    uint32_t dataField;
    uint32_t bitField;
    uint64_t requestFrame_Rate_Ctrl;
    uint64_t responseFrame_Rate_Ctrl;
    uint8_t  CRCvalue;
 
    if (SCH16T_isValidRateSens(Sens_Rate1) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    if (SCH16T_isValidRateSens(Sens_Rate2) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    if (SCH16T_isValidDecimation(Dec_Rate2) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }

    // Set sensitivities for Rate_XYZ1 (interpolated) and Rate_XYZ2 (decimated) outputs.
    // Also set decimation for Rate_XYZ2.
    requestFrame_Rate_Ctrl = REQ_SET_RATE_CTRL;
    dataField = SCH16T_convertRateSensToBitfield(Sens_Rate1);
    dataField <<= 3;
    bitField = SCH16T_convertRateSensToBitfield(Sens_Rate2);
    dataField |= bitField;
    dataField <<= 3;
    bitField = SCH16T_convertDecimationToBitfield(Dec_Rate2);
    dataField |= bitField;
    dataField <<= 3;
    dataField |= bitField;
    dataField <<= 3;
    dataField |= bitField;
    
    requestFrame_Rate_Ctrl |= dataField;
    requestFrame_Rate_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_Rate_Ctrl);
    requestFrame_Rate_Ctrl |= CRCvalue;
    SCH16T_sendRequest(requestFrame_Rate_Ctrl);

    // Read back rate control register contents.
    SCH16T_sendRequest(REQ_READ_RATE_CTRL);
    responseFrame_Rate_Ctrl = SCH16T_sendRequest(REQ_READ_RATE_CTRL);

    // Check that return frame is not blank.
    if ((responseFrame_Rate_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Rate_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((requestFrame_Rate_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_Rate_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    
    // Check that read and written data match.
    if ((requestFrame_Rate_Ctrl & DATA_FIELD_MASK) != (responseFrame_Rate_Ctrl & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;

    return SCH1_OK;
}


int SCH16T_setAccSensDec(uint16_t Sens_Acc1, uint16_t Sens_Acc2, uint16_t Sens_Acc3, uint16_t Dec_Acc2)
{
    uint32_t dataField;
    uint32_t bitField;
    uint64_t requestFrame_Acc12_Ctrl;
    uint64_t responseFrame_Acc12_Ctrl;
    uint64_t requestFrame_Acc3_Ctrl;
    uint64_t responseFrame_Acc3_Ctrl;
    uint8_t  CRCvalue;
 
    if (SCH16T_isValidAccSens(Sens_Acc1) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    if (SCH16T_isValidAccSens(Sens_Acc2) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    if (SCH16T_isValidAccSens(Sens_Acc3) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }
    if (SCH16T_isValidDecimation(Dec_Acc2) == false) {
        return SCH1_ERR_INVALID_PARAM;
    }

    // Set sensitivities for Acc_XYZ1 (interpolated) and Acc_XYZ2 (decimated) outputs.
    // Also set decimation for Acc_XYZ2.
    requestFrame_Acc12_Ctrl = REQ_SET_ACC12_CTRL;
    dataField = SCH16T_convertAccSensToBitfield(Sens_Acc1);
    dataField <<= 3;
    bitField = SCH16T_convertAccSensToBitfield(Sens_Acc2);
    dataField |= bitField;
    dataField <<= 3;
    bitField = SCH16T_convertDecimationToBitfield(Dec_Acc2);
    dataField |= bitField;
    dataField <<= 3;
    dataField |= bitField;
    dataField <<= 3;
    dataField |= bitField;
    
    requestFrame_Acc12_Ctrl |= dataField;
    requestFrame_Acc12_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_Acc12_Ctrl);
    requestFrame_Acc12_Ctrl |= CRCvalue;
    SCH16T_sendRequest(requestFrame_Acc12_Ctrl);

    // Set sensitivity for Acc_XYZ3 (interpolated) output.
    requestFrame_Acc3_Ctrl = REQ_SET_ACC3_CTRL;
    dataField = SCH16T_convertAccSensToBitfield(Sens_Acc3);
    requestFrame_Acc3_Ctrl |= dataField;
    requestFrame_Acc3_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_Acc3_Ctrl);
    requestFrame_Acc3_Ctrl |= CRCvalue;
    SCH16T_sendRequest(requestFrame_Acc3_Ctrl);

    // Read back sensitivity control register contents.
    SCH16T_sendRequest(REQ_READ_ACC12_CTRL);
    responseFrame_Acc12_Ctrl = SCH16T_sendRequest(REQ_READ_ACC3_CTRL);
    responseFrame_Acc3_Ctrl = SCH16T_sendRequest(REQ_READ_ACC3_CTRL);

    // Check that return frame is not blank.
    if ((responseFrame_Acc12_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Acc12_Ctrl == 0x00))
        return SCH1_ERR_OTHER;
    if ((responseFrame_Acc3_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Acc3_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((requestFrame_Acc12_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc12_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    if (((requestFrame_Acc3_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc3_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    
    // Check that read and written data match.
    if ((requestFrame_Acc12_Ctrl & DATA_FIELD_MASK) != (responseFrame_Acc12_Ctrl & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    if ((requestFrame_Acc3_Ctrl & DATA_FIELD_MASK) != (responseFrame_Acc3_Ctrl & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;

    return SCH1_OK;
}


int SCH16T_setDRY(int8_t polarity, bool enable)
{
    uint64_t requestFrame_User_If_Ctrl;
    uint64_t responseFrame_User_If_Ctrl;
    uint64_t dataContent;
    uint8_t  CRCvalue;

    if ((polarity < -1) || (polarity > 1))
        return SCH1_ERR_INVALID_PARAM;
    
    // Read USER_IF_CTRL -register content
    SCH16T_sendRequest(REQ_READ_USER_IF_CTRL);
    responseFrame_User_If_Ctrl = SCH16T_sendRequest(REQ_READ_USER_IF_CTRL);
    dataContent = (responseFrame_User_If_Ctrl & DATA_FIELD_MASK) >> 8;
    
    if (polarity == 0)
        dataContent &= (uint16_t)~0x40;   // Set DRY active high (0b01000000)
    else if (polarity == 1)
        dataContent |= 0x40;              // Set DRY active low
    
    if (enable)
        dataContent |= 0x20;              // Set DRY enabled (0b00100000)
    else
        dataContent &= (uint16_t)~0x20;   // Set DRY disabled
        
    requestFrame_User_If_Ctrl = REQ_SET_USER_IF_CTRL;
    requestFrame_User_If_Ctrl |= dataContent;
    requestFrame_User_If_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_User_If_Ctrl);
    requestFrame_User_If_Ctrl |= CRCvalue;
    SCH16T_sendRequest(requestFrame_User_If_Ctrl);

    // Read back sensitivity control register contents.
    SCH16T_sendRequest(REQ_READ_USER_IF_CTRL);
    responseFrame_User_If_Ctrl = SCH16T_sendRequest(REQ_READ_USER_IF_CTRL);

    // Check that return frame is not blank.
    if ((responseFrame_User_If_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_User_If_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((requestFrame_User_If_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_User_If_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;

    // Check that read and written data match.
    if ((requestFrame_User_If_Ctrl & DATA_FIELD_MASK) != (responseFrame_User_If_Ctrl & DATA_FIELD_MASK))
        return SCH1_ERR_OTHER;
    
    return SCH1_OK;
}


int SCH16T_enableMeas(bool enableSensor, bool setEOI)
{
    uint64_t requestFrame_Mode_Ctrl;
    uint64_t responseFrame_Mode_Ctrl;
    uint8_t  CRCvalue;

    requestFrame_Mode_Ctrl = REQ_SET_MODE_CTRL;

    // Handle EN_SENSOR -bit
    if (enableSensor)
        requestFrame_Mode_Ctrl |= 0x01;

    // Handle EOI_CTRL -bit
    if (setEOI)
        requestFrame_Mode_Ctrl |= 0x02;

    requestFrame_Mode_Ctrl <<= 8;
    CRCvalue = CRC8(requestFrame_Mode_Ctrl);
    requestFrame_Mode_Ctrl |= CRCvalue;
    SCH16T_sendRequest(requestFrame_Mode_Ctrl);

    // Read back sensitivity control register contents.
    SCH16T_sendRequest(REQ_READ_MODE_CTRL);
    responseFrame_Mode_Ctrl = SCH16T_sendRequest(REQ_READ_MODE_CTRL);

    // Check that return frame is not blank.
    if ((responseFrame_Mode_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Mode_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((requestFrame_Mode_Ctrl & TA_FIELD_MASK) >> 38) != ((responseFrame_Mode_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    
    return SCH1_OK;
}


/**
 * @brief Get各种
 *
 * @param 
 * 
 * @return 
 *         
 */
int SCH1_getAccSensDec(uint16_t *Sens_Acc1, uint16_t *Sens_Acc2, uint16_t *Sens_Acc3, uint16_t *Dec_Acc2)
{
    uint32_t dataField;
    uint64_t responseFrame_Acc12_Ctrl;
    uint64_t responseFrame_Acc3_Ctrl;

    // Read Acc12 and Acc3 control register contents.
    SCH16T_sendRequest(REQ_READ_ACC12_CTRL);
    responseFrame_Acc12_Ctrl = SCH16T_sendRequest(REQ_READ_ACC3_CTRL);
    responseFrame_Acc3_Ctrl = SCH16T_sendRequest(REQ_READ_ACC3_CTRL);


    // Check that return frame is not blank.
    if ((responseFrame_Acc12_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Acc12_Ctrl == 0x00))
        return SCH1_ERR_OTHER;
    if ((responseFrame_Acc3_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Acc3_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((REQ_READ_ACC12_CTRL & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc12_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;
    if (((REQ_READ_ACC3_CTRL & TA_FIELD_MASK) >> 38) != ((responseFrame_Acc3_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;

    
    // Get sensitivities for Acc_XYZ1 (interpolated) and Acc_XYZ2 (decimated) outputs.
    // Also get decimation for Acc_XYZ2.
    
    // Firat get the Acc_XYZ2 decimation
    dataField = (uint16_t)(responseFrame_Acc12_Ctrl >> 8) & 0x07;
    *Dec_Acc2 = (uint16_t)SCH16T_convertBitfieldToDecimation(dataField);

    // Acc_XYZ2 sensitivity
    dataField = (uint16_t)(responseFrame_Acc12_Ctrl >> 17) & 0x07;
    *Sens_Acc2 = (uint16_t)SCH16T_convertBitfieldToAccSens(dataField);

    // Acc_XYZ1 sensitivity
    dataField = (uint16_t)(responseFrame_Acc12_Ctrl >> 20) & 0x07;
    *Sens_Acc1 = (uint16_t)SCH16T_convertBitfieldToAccSens(dataField);

    // Acc_XYZ3 sensitivity
    dataField = (uint16_t)(responseFrame_Acc3_Ctrl >> 8) & 0x07;
    *Sens_Acc3 = (uint16_t)SCH16T_convertBitfieldToAccSens(dataField);

    return SCH1_OK;
}


int SCH16T_getRateSensDec(uint16_t *Sens_Rate1, uint16_t *Sens_Rate2, uint16_t *Dec_Rate2)
{
    uint32_t dataField;
    uint64_t responseFrame_Rate_Ctrl;

    // Read Rate control register contents.
    SCH16T_sendRequest(REQ_READ_RATE_CTRL);
    responseFrame_Rate_Ctrl = SCH16T_sendRequest(REQ_READ_RATE_CTRL);

    // Check that return frame is not blank.
    if ((responseFrame_Rate_Ctrl == 0xFFFFFFFFFFFF) || (responseFrame_Rate_Ctrl == 0x00))
        return SCH1_ERR_OTHER;

    // Check that Source Address matches Target Address.
    if (((REQ_READ_RATE_CTRL & TA_FIELD_MASK) >> 38) != ((responseFrame_Rate_Ctrl & SA_FIELD_MASK) >> 37))
        return SCH1_ERR_OTHER;

    
    // Get sensitivities for Rate_XYZ1 (interpolated) and Rate_XYZ2 (decimated) outputs.
    // Also get decimation for Rate_XYZ2.
    
    // First get the Rate_XYZ2 decimation
    dataField = (uint16_t)(responseFrame_Rate_Ctrl >> 8) & 0x07;
    *Dec_Rate2 = (uint16_t)SCH16T_convertBitfieldToDecimation(dataField);

    // Rate_XYZ2 sensitivity
    dataField = (uint16_t)(responseFrame_Rate_Ctrl >> 17) & 0x07;
    *Sens_Rate2 = (uint16_t)SCH16T_convertBitfieldToRateSens(dataField);

    // Rate_XYZ1 sensitivity
    dataField = (uint16_t)(responseFrame_Rate_Ctrl >> 20) & 0x07;
    *Sens_Rate1 = (uint16_t)SCH16T_convertBitfieldToRateSens(dataField);

    return SCH1_OK;
}


int SCH16T_getStatus(SCH1_status *Status)
{
    if (Status == NULL) {
        return SCH1_ERR_NULL_POINTER;
    }
    
    SCH16T_sendRequest(REQ_READ_STAT_SUM);
    Status->Summary     = SPI48_DATA_UINT16(SCH16T_sendRequest(REQ_READ_STAT_SUM_SAT));
    Status->Summary_Sat = SPI48_DATA_UINT16(SCH16T_sendRequest(REQ_READ_STAT_COM));
    Status->Common      = SPI48_DATA_UINT16(SCH16T_sendRequest(REQ_READ_STAT_RATE_COM));
    Status->Rate_Common = SPI48_DATA_UINT16(SCH16T_sendRequest(REQ_READ_STAT_RATE_X));
    Status->Rate_X      = SPI48_DATA_UINT16(SCH16T_sendRequest(REQ_READ_STAT_RATE_Y));
    Status->Rate_Y      = SPI48_DATA_UINT16(SCH16T_sendRequest(REQ_READ_STAT_RATE_Z));
    Status->Rate_Z      = SPI48_DATA_UINT16(SCH16T_sendRequest(REQ_READ_STAT_ACC_X));    
    Status->Acc_X       = SPI48_DATA_UINT16(SCH16T_sendRequest(REQ_READ_STAT_ACC_Y));
    Status->Acc_Y       = SPI48_DATA_UINT16(SCH16T_sendRequest(REQ_READ_STAT_ACC_Z));
    Status->Acc_Z       = SPI48_DATA_UINT16(SCH16T_sendRequest(REQ_READ_STAT_ACC_Z));

    return SCH1_OK;
}

bool SCH16T_verifyStatus(SCH1_status *Status)
{
    if (Status == NULL) {
        return SCH1_ERR_NULL_POINTER;
    }

    if (Status->Summary != 0xffff)
        return false;
    if (Status->Summary_Sat != 0xffff)
        return false;
    if (Status->Common != 0xffff)
        return false;
    if (Status->Rate_Common != 0xffff)
        return false;
    if (Status->Rate_X != 0xffff)
        return false;
    if (Status->Rate_Y != 0xffff)
        return false;
    if (Status->Rate_Z != 0xffff)
        return false;
    if (Status->Acc_X != 0xffff)
        return false;
    if (Status->Acc_Y != 0xffff)
        return false;
    if (Status->Acc_Z != 0xffff)
        return false;
    
    return true;
}


char* SCH16T_getSnbr(void)
{
//    uint16_t sn_id1;
//    uint16_t sn_id2;
//    uint16_t sn_id3;
//    static char strBuffer[15];

//    SCH16T_sendRequest(REQ_READ_SN_ID1);
//    sn_id1 = SPI48_DATA_UINT16(SCH16T_sendRequest(REQ_READ_SN_ID2));
//    sn_id2 = SPI48_DATA_UINT16(SCH16T_sendRequest(REQ_READ_SN_ID3));
//    sn_id3 = SPI48_DATA_UINT16(SCH16T_sendRequest(REQ_READ_SN_ID3));

//    // Build serial number string 
//	snprintf(strBuffer, 14, "%05d%01X%04X", sn_id2, sn_id1 & 0x000F, sn_id3);
//    
//    return strBuffer;
}


/**
 * @brief CRC相关函数
 *
 * @param 
 * 
 * @return 
 *         
 */
uint8_t CRC8(uint64_t SPIframe)
{
    uint64_t data = SPIframe & 0xFFFFFFFFFF00LL;
    uint8_t crc = 0xFF;

    for (int i = 47; i >= 0; i--)
    {
        uint8_t data_bit = (data >> i) & 0x01;
        crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ 0x2F) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
    }
    
    return crc;
}

bool SCH1_checkCRC8(uint64_t SPIframe)
{
    if((uint8_t)(SPIframe & 0xff) == CRC8(SPIframe))
        return true;
    else
        return false;
}

uint8_t CRC3(uint32_t SPIframe)
{
    uint32_t data = SPIframe & 0xFFFFFFF8;
    uint8_t crc = 0x05;
 
    for (int i = 31; i >= 0; i--)
    {
        uint8_t data_bit = (data >> i) & 0x01;
        crc = crc & 0x4 ? (uint8_t)((crc << 1) ^ 0x3) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
        crc &= 0x07;
    }
 
    return crc;
}

bool SCH16T_checkCRC3(uint32_t SPIframe)
{
    if((uint8_t)(SPIframe & 0x07) == CRC3(SPIframe))
        return true;
    else
        return false;
}


/**
 * @brief Init SCH
 *
 * @param 
 * 
 * @return 
 *         
 */
int SCH16T_init(SCH1_filter sFilter, SCH1_sensitivity sSensitivity, SCH1_decimation sDecimation, bool enableDRY) 
{

    int ret = SCH1_OK;
    uint8_t startup_attempt = 0;
    bool SCH1status = false;
    SCH1_status SCH1statusAll;
        
    // SCH1 startup sequence specified in section "5 Component Operation,
    // Reset and Power Up" in the data sheet.

    SCH16T_Reset();

    for (startup_attempt = 0; startup_attempt < 2; startup_attempt++) {
                                    
        // Wait 32 ms for the non-volatile memory (NVM) Read
        HAL_Delay(32);       
        
        // Set user controls
        SCH16T_setFilters(sFilter.Rate12, sFilter.Acc12, sFilter.Acc3);
        SCH16T_setRateSensDec(sSensitivity.Rate1, sSensitivity.Rate2, sDecimation.Rate2);
        SCH16T_setAccSensDec(sSensitivity.Acc1, sSensitivity.Acc2, sSensitivity.Acc3, sDecimation.Acc2);
        if (enableDRY)
            SCH16T_setDRY(0, true);   // 0 = DRY active high
        else
            SCH16T_setDRY(0, false);

        // Write EN_SENSOR = 1
        SCH16T_enableMeas(true, false);

        // Wait 215 ms
        HAL_Delay(215);
    
        // Read all status registers once. No critization
        SCH16T_getStatus(&SCH1statusAll);

        // Write EOI = 1 (End of Initialization command)
        SCH16T_enableMeas(true, true);
        
        // Wait 3 ms
        HAL_Delay(3);
        
        // Read all status registers twice.
        SCH16T_getStatus(&SCH1statusAll);
        SCH16T_getStatus(&SCH1statusAll);

        // Read all user control registers and verify content - Add verification here if needed for FuSa.

        // Check that all status registers have OK status.
        if (!SCH16T_verifyStatus(&SCH1statusAll)) {
            SCH1status = false;            
            SCH16T_Reset();    // Sensor failed, reset and retry.
        }
        else {
            SCH1status = true;           
            break;
        }
        
    } // for (startup_attempt = 0; startup_attempt < 2; startup_attempt++) 

    if (SCH1status != true)
        ret = SCH1_ERR_SENSOR_INIT;
             
    return ret;
}

int8_t SCH16T_SetFilter_and_Init(bool isEnableDYR)
{
    SCH1_filter         Filter;
    SCH1_sensitivity    Sensitivity;
    SCH1_decimation     Decimation;

    Filter.Rate12 = FILTER_RATE;
	Filter.Acc12 = FILTER_ACC12;
	Filter.Acc3 = FILTER_ACC3;

    Sensitivity.Rate1 = SENSITIVITY_RATE1;
    Sensitivity.Rate2 = SENSITIVITY_RATE2;
    Sensitivity.Acc1  = SENSITIVITY_ACC1;
    Sensitivity.Acc2  = SENSITIVITY_ACC2;
    Sensitivity.Acc3  = SENSITIVITY_ACC3;    

    Decimation.Rate2 = DECIMATION_RATE;
    Decimation.Acc2  = DECIMATION_ACC;

    return SCH16T_init(Filter, Sensitivity, Decimation, isEnableDYR);
}

/**
 * @brief Get_data
 *
 * @param 
 * 
 * @return 
 *         
 */

void SCH16T_getData(SCH1_raw_data *data)
{
   SCH16T_sendRequest(REQ_READ_RATE_X2);
   uint64_t rate_x_raw = SCH16T_sendRequest(REQ_READ_RATE_Y2);
   uint64_t rate_y_raw = SCH16T_sendRequest(REQ_READ_RATE_Z2);
   uint64_t rate_z_raw = SCH16T_sendRequest(REQ_READ_ACC_X2);
   uint64_t acc_x_raw  = SCH16T_sendRequest(REQ_READ_ACC_Y2);
   uint64_t acc_y_raw  = SCH16T_sendRequest(REQ_READ_ACC_Z2);
   uint64_t acc_z_raw  = SCH16T_sendRequest(REQ_READ_TEMP);
   uint64_t temp_raw   = SCH16T_sendRequest(REQ_READ_TEMP);

   // Get possible frame errors
   uint64_t miso_words[] = {rate_x_raw, rate_y_raw, rate_z_raw, acc_x_raw, acc_y_raw, acc_z_raw, temp_raw};       
   data->frame_error = SCH16T_check_48bit_frame_error(miso_words, (sizeof(miso_words) / sizeof(uint64_t)));
   
   // Parse MISO data to structure
   data->Rate2_raw[AXIS_X] = SPI48_DATA_INT32(rate_x_raw);
   data->Rate2_raw[AXIS_Y] = SPI48_DATA_INT32(rate_y_raw);
   data->Rate2_raw[AXIS_Z] = SPI48_DATA_INT32(rate_z_raw);
   data->Acc2_raw[AXIS_X]  = SPI48_DATA_INT32(acc_x_raw);
   data->Acc2_raw[AXIS_Y]  = SPI48_DATA_INT32(acc_y_raw);
   data->Acc2_raw[AXIS_Z]  = SPI48_DATA_INT32(acc_z_raw);

   // Temperature data is always 16 bits wide. Drop 4 LSBs as they are not used.
   data->Temp_raw = SPI48_DATA_INT32(temp_raw) >> 4;
}

/**
 * @brief Convert_raw_data
 *
 * @param 
 * 
 * @return                     
 */
void SCH16T_convert_data(SCH1_raw_data *data_in, SCH1_result *data_out)
{
    // Convert from raw counts to sensitivity and calculate averages here for faster execution
    data_out->Rate2[AXIS_X] = (float)data_in->Rate2_raw[AXIS_X] / (SENSITIVITY_RATE1 * (float)AVG_FACTOR);
    data_out->Rate2[AXIS_Y] = (float)data_in->Rate2_raw[AXIS_Y] / (SENSITIVITY_RATE1 * (float)AVG_FACTOR);
    data_out->Rate2[AXIS_Z] = (float)data_in->Rate2_raw[AXIS_Z] / (SENSITIVITY_RATE1 * (float)AVG_FACTOR);
    data_out->Acc2[AXIS_X]  = (float)data_in->Acc2_raw[AXIS_X] / (SENSITIVITY_ACC1 * (float)AVG_FACTOR);
    data_out->Acc2[AXIS_Y]  = (float)data_in->Acc2_raw[AXIS_Y] / (SENSITIVITY_ACC1 * (float)AVG_FACTOR);
    data_out->Acc2[AXIS_Z]  = (float)data_in->Acc2_raw[AXIS_Z] / (SENSITIVITY_ACC1 * (float)AVG_FACTOR);

    // Convert temperature and calculate average
    data_out->Temp = GET_TEMPERATURE((float)data_in->Temp_raw / (float)AVG_FACTOR);
}

/**
 * @brief Get_data!
 *
 * @param 
 * 
 * @return                     
 */
void SCH16T_GetSingleData(SCH1_result *SCH16T_Data)
{
    SCH1_raw_data raw;
    SCH16T_getData(&raw);
    SCH16T_convert_data(&raw, SCH16T_Data);
}
