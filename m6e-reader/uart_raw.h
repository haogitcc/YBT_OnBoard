#ifndef __UART_RAW_H__
#define __UART_RAW_H__
#include "reader.h"

/**@brief   设置串口参数：波特率，数据位，停止位和效验位
 * @param[in]  fd         类型  int      打开的串口文件句柄
 * @param[in]  nSpeed     类型  int     波特率
 * @param[in]  nBits     类型  int     数据位   取值 为 7 或者8
 * @param[in]  nParity     类型  int     停止位   取值为 1 或者2
 * @param[in]  nStop      类型  int      效验类型 取值为N,E,O,,S
 * @return     返回设置结果
 * - 0         设置成功
 * - -1     设置失败
 */
int setOpt(int *fdSerial, int nSpeed, int nBits, int nParity, int nStop);


/**@brief 串口读取函数
 * @param[in]  fd         打开的串口文件句柄
 * @param[in]  *rcv_buf 接收缓存指针
 * @param[in]  data_len    要读取数据长度
 * @param[in]  timeout     接收等待超时时间，单位ms
 * @return     返回设置结果
 * - >0      设置成功
 * - 其他      读取超时或错误
 */
int UART_Recv(int *fdSerial, char *rcv_buf, int data_len, int timeout);

/**@brief 串口发送函数
 * @param[in]  fd            打开的串口文件句柄
 * @param[in]  *send_buf     发送数据指针
 * @param[in]  data_len     发送数据长度
 * @return     返回结果
 * - data_len    成功
 * - -1            失败
 */
int UART_Send(int *fdSerial, char *send_buf, int data_len);

int UART_Open(int *fdSerial, char *dev);

void UART_Set(int *fdSerial);

void UART_Close(int *fdSerial);

int UART_serial_sendBytes(int fd, uint8_t* message, uint32_t length);

int UART_write2port(int fd, const char *data, int size_data);

int UART_read_from_port(int fd, char *data, int size_data);

#endif//__UART_RAW_H__
