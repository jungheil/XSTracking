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

class CRC16 {
  public:
    /*
    ** Descriptions: CRC16 Verify function
    ** Input: Data to Verify,Stream length = Data + checksum
    ** Output: True or False (CRC Verify Result)
    */
    bool VerifyCRC16CheckSum(const uint8_t *pchMessage,
                             const uint32_t dwLength) const;
    /*
    ** Descriptions: append CRC16 to the end of data
    ** Input: Data to CRC and append,Stream length = Data + checksum
    ** Output: True or False (CRC Verify Result)
    */
    bool AppendCRC16CheckSum(uint8_t       *pchMessage,
                             const uint32_t dwLength) const;

  protected:
    /*
    ** Descriptions: CRC16 checksum function
    ** Input: Data to check,Stream length, initialized checksum
    ** Output: CRC checksum
    */
    virtual uint16_t GetCRC16CheckSum(const uint8_t *pchMessage,
                                      uint32_t       dwLength) const = 0;

    template <typename T>
    static T Inverse(const T &buff) {
        T t;
        for (int i = 0; i < sizeof(T) * 8; ++i) {
            t = t << 1;
            t = t | ((buff >> i) & 0x01);
        }
        return std::move(t);
    }
};

class CRC16Factory {
  public:
    enum CRC16_NAME {
        CRC16_NAME_IBM        = 0,
        CRC16_NAME_MAXIM      = 1,
        CRC16_NAME_USB        = 2,
        CRC16_NAME_MODBUS     = 3,
        CRC16_NAME_CCITT      = 4,
        CRC16_NAME_CCITTFLASE = 5,
        CRC16_NAME_X25        = 6,
        CRC16_NAME_XMODEM     = 7
    };
    static CRC16 *Get(CRC16_NAME name);
};

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
    bool    empty_   = true;
    bool    is_open_ = false;
    speed_t baudr_;
    char   *device_;
    int     serial_fd_;  //设备名称
    uint8_t len_ = 0;
    CRC16  *crc16;
};

#endif  // XSTRACKING_USART_H
