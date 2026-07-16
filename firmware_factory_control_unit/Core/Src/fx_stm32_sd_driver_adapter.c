#include "fx_stm32_sd_driver.h"

TX_SEMAPHORE sd_tx_semaphore;
TX_SEMAPHORE sd_rx_semaphore;

extern SD_HandleTypeDef hsd1;

/**
* @brief Check the SD IP status.
* @param UINT instance SD instance to check
* @retval 0 when ready 1 when busy
*/
INT fx_stm32_sd_get_status(UINT instance)
{
  UNUSED(instance);
  return (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)?1:0;
}

/**
* @brief Read Data from the SD device into a buffer.
* @param UINT instance SD IP instance to read from.
* @param UINT *buffer buffer into which the data is to be read.
* @param UINT start_block the first block to start reading from.
* @param UINT total_blocks total number of blocks to read.
* @retval 0 on success error code otherwise
*/
INT fx_stm32_sd_read_blocks(UINT instance, UINT *buffer, UINT start_block, UINT total_blocks)
{
    UNUSED(instance);
    return HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)buffer, start_block, total_blocks) != HAL_OK?1:0;
}

/**
* @brief Write data buffer into the SD device.
* @param UINT instance SD IP instance to write into.
* @param UINT *buffer buffer to write into the SD device.
* @param UINT start_block the first block to start writing into.
* @param UINT total_blocks total number of blocks to write.
* @retval 0 on success error code otherwise
*/
INT fx_stm32_sd_write_blocks(UINT instance, UINT *buffer, UINT start_block, UINT total_blocks)
{
	UNUSED(instance);
    return HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)buffer, start_block, total_blocks) != HAL_OK? 1:0;
}

/**
* @brief SD DMA Tx Transfer completed callbacks
* @param Instance the sd instance
* @retval None
*/
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  tx_semaphore_put(&sd_tx_semaphore);
}

/**
* @brief SD DMA Rx Transfer completed callbacks
* @param Instance the sd instance
* @retval None
*/
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  tx_semaphore_put(&sd_rx_semaphore);
}