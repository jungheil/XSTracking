#ifndef XSTRACKING_USART_H
#define XSTRACKING_USART_H
#pragma once

#include <fcntl.h>  //文件控制定义
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>  //终端控制定义
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <vector>

class Usart {
  public:
    Usart() = default;
    explicit Usart(char *device, uint32_t baudrate = 9600,
                   uint8_t len = 0);  //串口初始化
    void Init();
    bool Write(uint8_t *buff, int len);
    bool Read(uint8_t *buff, int len);
    bool isOpen() const { return is_open_; };

    virtual bool Send(void *msg) { return false; };
    virtual bool Recv(void *msg) { return false; };

  private:
    bool     empty_   = true;
    bool     is_open_ = false;
    speed_t  baudr_;
    char *   device_;
    int      serial_fd_;  //设备名称
    uint8_t  len_ = 0;
    uint16_t GetCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength,
                              uint16_t wCRC);
    uint32_t VerifyCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);
    void     AppendCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);
};

#endif  // XSTRACKING_USART_H