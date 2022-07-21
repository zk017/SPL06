uint32_t bmp280_iic_read(uint8_t dev, uint8_t reg, uint8_t *pdata, uint32_t len)

{
	
	return nrf_drv_iic_read(dev, reg, len, pdata);

}

//---------------------------------------------------------------------------------

uint32_t bmp280_iic_write(uint8_t dev, uint8_t reg, uint8_t *pdata, uint32_t len)

{
	
	return nrf_drv_iic_write(dev, reg, len ,pdata);

}