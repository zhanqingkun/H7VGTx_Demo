#ifndef __CRC_H__
#define __CRC_H__

/* 系统库 */
#include <stdint.h>

// CRC8
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength, uint8_t CRC8);
uint32_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);

// CRC16
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t CRC16);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

//CRC_CCITT
uint16_t Get_CRC_CCITT_Check_Sum(uint16_t crc, uint8_t const *buffer, int len);

#endif
