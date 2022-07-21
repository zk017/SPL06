
#include "nrf_log.h"
#include "app_mpu6500.h"
#include "drv_mpu_xxxx.h"
#include "nrf_delay.h"
#include "mytype.h"

//=============================================================================


//=============================================================================


//---------------------------------------------------------------------------


//-----------------------------------------------------------------------------









//==========================================================================================================
//SPL06 driver
//==========================================================================================================

SPL06_T  g_spl06_1, g_spl06_2, g_spl06_3;
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1
//========================================================================
void set_spl06_4_api(SPL06_T  *pSpl06)
{
	pSpl06->pFuncRead = bmp280_iic_2_read;
	pSpl06->pFuncWrite = bmp280_iic_2_write;
	pSpl06->dev_addr  = 0x76;
	pSpl06->delay_msec_func = nrf_delay_ms;
}

void set_spl06_3_api(SPL06_T  *pSpl06)
{
	pSpl06->pFuncRead = bmp280_iic_2_read;
	pSpl06->pFuncWrite = bmp280_iic_2_write;
	pSpl06->dev_addr  = BMP280_DEV_ADDR;
	pSpl06->delay_msec_func = nrf_delay_ms;
}

//=============================================================================
void set_spl06_2_api(SPL06_T *pSpl06)
{
	pSpl06->pFuncRead = bmp280_iic_read;
	pSpl06->pFuncWrite = bmp280_iic_write;
	pSpl06->dev_addr  = 0x76;
	pSpl06->delay_msec_func = nrf_delay_ms;
}
//----------------------------------------------------------------------------


void set_spl06_1_api(SPL06_T *pSpl06)
{
	pSpl06->pFuncRead = bmp280_iic_read;
	pSpl06->pFuncWrite = bmp280_iic_write;
	pSpl06->dev_addr  = BMP280_DEV_ADDR;//0x77
	pSpl06->delay_msec_func = nrf_delay_ms;
}
//---------------------------------------------------------------------------
int8_t spl06_get_calib_param(SPL06_T *pSpl06)
{
	uint32_t h;
    uint32_t m;
    uint32_t l;
	
	uint8_t tmp_val;

	pSpl06->pFuncRead(pSpl06->dev_addr, 0x10, &tmp_val, 1);
	h = tmp_val;
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x11, &tmp_val, 1);
	l = tmp_val;
    pSpl06->calib_param.c0 = (int32_t)h<<4 | l>>4;
    pSpl06->calib_param.c0 = (pSpl06->calib_param.c0&0x0800)?(0xF000|pSpl06->calib_param.c0):pSpl06->calib_param.c0;
	
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x11, &tmp_val, 1);
	h = tmp_val;
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x12, &tmp_val, 1);
	l = tmp_val;
    pSpl06->calib_param.c1 = (int32_t)(h&0x0F)<<8 | l;
    pSpl06->calib_param.c1 = (pSpl06->calib_param.c1&0x0800)?(0xF000|pSpl06->calib_param.c1):pSpl06->calib_param.c1;
	
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x13, &tmp_val, 1);
	h = tmp_val;
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x14, &tmp_val, 1);
	m = tmp_val;
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x15, &tmp_val, 1);
	l = tmp_val;
    pSpl06->calib_param.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
    pSpl06->calib_param.c00 = (pSpl06->calib_param.c00&0x080000)?(0xFFF00000|pSpl06->calib_param.c00):pSpl06->calib_param.c00;
	
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x15, &tmp_val, 1);
	h = tmp_val;
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x16, &tmp_val, 1);
	m = tmp_val;
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x17, &tmp_val, 1);
	l = tmp_val;
    pSpl06->calib_param.c10 = (int32_t)h<<16 | (int32_t)m<<8 | l;
    pSpl06->calib_param.c10 = (pSpl06->calib_param.c10&0x080000)?(0xFFF00000|pSpl06->calib_param.c10):pSpl06->calib_param.c10;
	
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x18, &tmp_val, 1);
	h = tmp_val;
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x19, &tmp_val, 1);
	l = tmp_val;
    pSpl06->calib_param.c01 = (int32_t)h<<8 | l;
	
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x1A, &tmp_val, 1);
	h = tmp_val;
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x1B, &tmp_val, 1);
	l = tmp_val;
    pSpl06->calib_param.c11 = (int32_t)h<<8 | l;
	
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x1C, &tmp_val, 1);
	h = tmp_val;
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x1D, &tmp_val, 1);
	l = tmp_val;
    pSpl06->calib_param.c20 = (int32_t)h<<8 | l;
	
	
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x1E, &tmp_val, 1);
	h = tmp_val;
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x1F, &tmp_val, 1);
	l = tmp_val;
    pSpl06->calib_param.c21 = (int32_t)h<<8 | l;

	pSpl06->pFuncRead(pSpl06->dev_addr, 0x20, &tmp_val, 1);
	h = tmp_val;
	pSpl06->pFuncRead(pSpl06->dev_addr, 0x21, &tmp_val, 1);
	l = tmp_val;
    pSpl06->calib_param.c30 = (int32_t)h<<8 | l;
	return 0;
}
//---------------------------------------------------------------------------
int8_t init_one_spl06(SPL06_T *pSpl06)
{
	/* variable used to return communication result*/
	int8_t com_rslt = ERROR;
	u8 v_data_u8 = 0; //0
	u8 v_chip_id_read_count = 5; //5
 
     
	while (v_chip_id_read_count > 0) 
	{
		/* read chip id */
		com_rslt = pSpl06->pFuncRead(pSpl06->dev_addr, 0x0D, &v_data_u8, 1);
		/* Check for the correct chip id */
		NRF_LOG_PRINTF("devaddr=0x%2X, obj_addr=0x%2X, chip_id=0x%2X\n", pSpl06->dev_addr, pSpl06, v_data_u8);
		if (v_data_u8 == SPL06_CHIP_ID)
			break;
		v_chip_id_read_count--;
		/* Delay added concerning the low speed of power up system to
		facilitate the proper reading of the chip ID */
		pSpl06->delay_msec_func(30);
	}

	/*assign chip ID to the global structure*/
	pSpl06->chip_id = v_data_u8;
	/*com_rslt status of chip ID read*/
	com_rslt = (v_chip_id_read_count == 0) ?SPL06_CHIP_ID_READ_FAIL : SPL06_CHIP_ID_READ_SUCCESS;

	if (com_rslt == SPL06_CHIP_ID_READ_SUCCESS) 
	{
		/* readout bmp280 calibration parameter structure */
		spl06_get_calib_param(pSpl06);
	}
	return com_rslt;
}

//-----------------------------------------------------------------------------
void spl0601_rateset(SPL06_T *pSnr, uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
    uint8_t reg = 0;
    int32_t i32kPkT = 0;
    switch(u8SmplRate)
    {
        case 2:
            reg |= (1<<4);
            break;
        case 4:
            reg |= (2<<4);
            break;
        case 8:
            reg |= (3<<4);
            break;
        case 16:
            reg |= (4<<4);
            break;
        case 32:
            reg |= (5<<4);
            break;
        case 64:
            reg |= (6<<4);
            break;
        case 128:
            reg |= (7<<4);
            break;
        case 1:
        default:
            break;
    }
    switch(u8OverSmpl)
    {
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    if(iSensor == PRESSURE_SENSOR)
    {
		pSnr->i32kP = i32kPkT;
		pSnr->pFuncWrite(pSnr->dev_addr, 0x06, &reg, 1);
        if(u8OverSmpl > 8)
        {
            //reg = spl0601_read(HW_ADR, 0x09);
			pSnr->pFuncRead(pSnr->dev_addr, 0x09, &reg, 1);
            //spl0601_write(HW_ADR, 0x09, reg | 0x04);
			reg|=0x04;
			pSnr->pFuncWrite(pSnr->dev_addr, 0x09, &reg, 1);
        }
        else
        {
            //reg = spl0601_read(HW_ADR, 0x09);
			pSnr->pFuncRead(pSnr->dev_addr, 0x09, &reg, 1);
            //spl0601_write(HW_ADR, 0x09, reg & (~0x04));
			reg = reg&(~0x04);
			pSnr->pFuncWrite(pSnr->dev_addr, 0x09, &reg, 1);
        }
    }
    if(iSensor == TEMPERATURE_SENSOR)
    {
        pSnr->i32kT = i32kPkT;
        //spl0601_write(HW_ADR, 0x07, reg|0x80);  //Using mems temperature
		reg|=0x80;
		pSnr->pFuncWrite(pSnr->dev_addr, 0x07, &reg, 1);
        if(u8OverSmpl > 8)
        {
            //reg = spl0601_read(HW_ADR, 0x09);
			pSnr->pFuncRead(pSnr->dev_addr, 0x09, &reg, 1);
            //spl0601_write(HW_ADR, 0x09, reg | 0x08);
			reg |=0x08;
			pSnr->pFuncWrite(pSnr->dev_addr, 0x09, &reg, 1);
        }
        else
        {
            //reg = spl0601_read(HW_ADR, 0x09);
			pSnr->pFuncRead(pSnr->dev_addr, 0x09, &reg, 1);
            //spl0601_write(HW_ADR, 0x09, reg & (~0x08));
			reg =  reg & (~0x08);
			pSnr->pFuncWrite(pSnr->dev_addr, 0x09, &reg, 1);
        }
    }
}
//---------------------------------------------------------------------------
void spl0601_start_temperature(SPL06_T *p1)
{
    //spl0601_write(HW_ADR, 0x08, 0x02);
	uint8_t a = 0x02;
	p1->pFuncWrite(p1->dev_addr, 0x08, &a, 1);
}

void spl0601_start_pressure(SPL06_T *p1)
{
    //spl0601_write(HW_ADR, 0x08, 0x01);
	uint8_t a = 0x01;
	p1->pFuncWrite(p1->dev_addr, 0x08, &a, 1);
}

void spl0601_start_continuous(SPL06_T *p1, uint8_t mode)
{
    //spl0601_write(HW_ADR, 0x08, mode+4);
	uint8_t a = mode+4;
	p1->pFuncWrite(p1->dev_addr, 0x08, &a, 1);
}

void spl0601_get_raw_temp(SPL06_T *p)
{
    uint8_t h[3] = {0};
    
	//	h[0] = spl0601_read(HW_ADR, 0x03);
	//	h[1] = spl0601_read(HW_ADR, 0x04);
	//	h[2] = spl0601_read(HW_ADR, 0x05);
	p->pFuncRead(p->dev_addr, 0x03, &h[0], 1);
	p->pFuncRead(p->dev_addr, 0x04, &h[1], 1);
	p->pFuncRead(p->dev_addr, 0x05, &h[2], 1);

    p->i32rawTemperature = (int32_t)h[0]<<16 | (int32_t)h[1]<<8 | (int32_t)h[2];
    p->i32rawTemperature= (p->i32rawTemperature&0x800000) ? (0xFF000000|p->i32rawTemperature) : p->i32rawTemperature;
}

void spl0601_get_raw_pressure(SPL06_T *p)
{
    uint8_t h[3];
    
	//	h[0] = spl0601_read(HW_ADR, 0x00);
	//	h[1] = spl0601_read(HW_ADR, 0x01);
	//	h[2] = spl0601_read(HW_ADR, 0x02);
	p->pFuncRead(p->dev_addr, 0x00, &h[0], 1);
	p->pFuncRead(p->dev_addr, 0x01, &h[1], 1);
	p->pFuncRead(p->dev_addr, 0x02, &h[2], 1);
    
    p->i32rawPressure = (int32_t)h[0]<<16 | (int32_t)h[1]<<8 | (int32_t)h[2];
    p->i32rawPressure= (p->i32rawPressure&0x800000) ? (0xFF000000|p->i32rawPressure) : p->i32rawPressure;
}

float spl0601_get_temperature(SPL06_T *p)
{
    float fTCompensate;
    float fTsc;

    fTsc = p->i32rawTemperature / (float)p->i32kT;
    fTCompensate =  p->calib_param.c0 * 0.5 + p->calib_param.c1 * fTsc;
    return fTCompensate;
}
//---------------------------------------------------------------------------------
float spl0601_get_pressure(SPL06_T *p)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = p->i32rawTemperature / (float)p->i32kT;
    fPsc = p->i32rawPressure / (float)p->i32kP;
    qua2 = p->calib_param.c10 + fPsc * (p->calib_param.c20 + fPsc* p->calib_param.c30);
    qua3 = fTsc * fPsc * (p->calib_param.c11 + fPsc * p->calib_param.c21);
		//qua3 = 0.9f *fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);
	
    fPCompensate = p->calib_param.c00 + fPsc * qua2 + fTsc * p->calib_param.c01 + qua3;
		//fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + 0.9f *fTsc  * p_spl0601->calib_param.c01 + qua3;
    return fPCompensate;
}
//------------------------------------------------------------------------------------
uint32_t get_one_sensor_pressure(SPL06_T *p)
{
	float pressure = 0;
	uint32_t v = 0;
	spl0601_get_raw_temp(p);
	spl0601_get_temperature(p);

	spl0601_get_raw_pressure(p);
	pressure = spl0601_get_pressure(p);
	v = (uint32_t)pressure;
	//NRF_LOG_INFO("F_pressure:" NRF_LOG_FLOAT_MARKER "Pa\n", NRF_LOG_FLOAT(pressure));
	//NRF_LOG_PRINTF("int_pressure=%d\n", v);
	return v;
}
//-----------------------------------------------------------------------------
#define CONTINUOUS_P_AND_T      3
float temp_pressure = 0, temp_temperature;
int8_t Init_all_spl06(SPL06_T *p1, SPL06_T *p2, SPL06_T *p3)//第一个传感器出错返回1，第二个出错返回2，第三个返回4
{
	uint8_t retValue = 0;
	set_spl06_1_api(p1);//SCL、SDA= --7,9--
	set_spl06_2_api(p2); //--7,9--
	set_spl06_3_api(p3); //-28,26--
	
	if(init_one_spl06(p3))
	{
		retValue |= 0x04;
	}
	
	spl0601_rateset(p3, PRESSURE_SENSOR,32, 8);   

    spl0601_rateset(p3, TEMPERATURE_SENSOR,32, 8);
	spl0601_start_continuous(p3, CONTINUOUS_P_AND_T);
	
	if(init_one_spl06(p1))
	{
		retValue |= 0x01;
	}
	if(init_one_spl06(p2))
	{
		retValue |= 0x02;
	}

	
	spl0601_get_raw_temp(p3);
	temp_temperature = spl0601_get_temperature(p3);

	spl0601_get_raw_pressure(p3);
	temp_pressure = spl0601_get_pressure(p3)/1000;

	NRF_LOG_INFO("P3: temp:" NRF_LOG_FLOAT_MARKER ",", NRF_LOG_FLOAT(temp_temperature));
	NRF_LOG_INFO("pressure:" NRF_LOG_FLOAT_MARKER "kPa\n", NRF_LOG_FLOAT(temp_pressure));

	spl0601_rateset(p2, PRESSURE_SENSOR,32, 8);   

    spl0601_rateset(p2, TEMPERATURE_SENSOR,32, 8);
	spl0601_start_continuous(p2, CONTINUOUS_P_AND_T);
	
	spl0601_get_raw_temp(p2);
	temp_temperature = spl0601_get_temperature(p2);

	spl0601_get_raw_pressure(p2);
	temp_pressure = spl0601_get_pressure(p2)/1000;

	NRF_LOG_INFO("P2: temp:" NRF_LOG_FLOAT_MARKER ",", NRF_LOG_FLOAT(temp_temperature));
	NRF_LOG_INFO("pressure:" NRF_LOG_FLOAT_MARKER "kPa\n", NRF_LOG_FLOAT(temp_pressure));
	
	//----------------------------------------
	spl0601_rateset(p1, PRESSURE_SENSOR,32, 8);   

    spl0601_rateset(p1, TEMPERATURE_SENSOR,32, 8);
	spl0601_start_continuous(p1, CONTINUOUS_P_AND_T);
	
	spl0601_get_raw_temp(p1);
	temp_temperature = spl0601_get_temperature(p1);

	spl0601_get_raw_pressure(p1);
	temp_pressure = spl0601_get_pressure(p1)/1000;

	NRF_LOG_INFO("P1: temp:" NRF_LOG_FLOAT_MARKER ",", NRF_LOG_FLOAT(temp_temperature));
	NRF_LOG_INFO("pressure:" NRF_LOG_FLOAT_MARKER "kPa.\r\n", NRF_LOG_FLOAT(temp_pressure));
	
	return retValue;
}

//---------------------------------------------------------------------------
void loop_detect(SPL06_T *p3, SPL06_T *p2, SPL06_T *p1)
{
	float temp_pressure3=0, temp_pressure2=0, temp_pressure1=0;
	//spl0601_get_raw_temp(p3);
	//temp_temperature3 = spl0601_get_temperature(p3);

	//spl0601_get_raw_pressure(p3);
	temp_pressure3 = spl0601_get_pressure(p3)/1000;

	//NRF_LOG_INFO("P3: temp:" NRF_LOG_FLOAT_MARKER ",", NRF_LOG_FLOAT(temp_temperature));
	NRF_LOG_INFO("pressure3:" NRF_LOG_FLOAT_MARKER "kPa", NRF_LOG_FLOAT(temp_pressure3));
	
	//spl0601_get_raw_temp(p2);
	//temp_temperature2 = spl0601_get_temperature(p2);

	//spl0601_get_raw_pressure(p2);
	temp_pressure2 = spl0601_get_pressure(p2)/1000;

	//NRF_LOG_INFO("P3: temp:" NRF_LOG_FLOAT_MARKER ",", NRF_LOG_FLOAT(temp_temperature));
	NRF_LOG_INFO("|pressure2:" NRF_LOG_FLOAT_MARKER "kPa", NRF_LOG_FLOAT(temp_pressure2));
	
	temp_pressure1 = spl0601_get_pressure(p1)/1000;

	//NRF_LOG_INFO("P3: temp:" NRF_LOG_FLOAT_MARKER ",", NRF_LOG_FLOAT(temp_temperature));
	NRF_LOG_INFO("|pressure1:" NRF_LOG_FLOAT_MARKER "kPa\n", NRF_LOG_FLOAT(temp_pressure1));
}



