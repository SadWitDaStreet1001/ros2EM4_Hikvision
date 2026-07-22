#pragma once
using namespace std;
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include<vector>
#include<math.h>
#include<time.h>

#define VIEW_H 1040
#define ENC_IDENT 0
#define ENC_NEAR  1
#define ENC_DIFF  2
#define ENC_ORG   3

namespace robosense
{
namespace lidar
{
class CompressAlgo
{
public:
static void RLenc_unpack_optimize(const uint16_t* data, uint16_t* punpack_data, uint16_t data_len, uint8_t dec_width)
{
    uint8_t type = (data[0] >> 14) & 0x3;
    bool endflag = false;
    uint16_t block_len = (data[0] & 0x3fff) - 1;
    if (block_len >= VIEW_H)
    {
        return;
    }
    uint16_t base = data[1];
    uint16_t vec_idx = 0;
    punpack_data[vec_idx++] = base;
    int i = (block_len != 0) ? 2 : 1;//
    uint8_t width = 0;
    if ((data_len & 0x1) == 0)//ensure data_len is 32bit * n
    {
        while (!endflag)
        {
            if (block_len != 0)
            {
                uint16_t org;
                switch (type)
                {
                case ENC_IDENT:

                    org = base;
                    punpack_data[vec_idx++] = org;
                    block_len--;
                    break;
                case ENC_NEAR://4bit

                    if (block_len >= 4)
                    {
                        for (uint8_t k = 0; k < 4; k++)
                        {
                            width = 4 * k;
                            int8_t flag = (data[i] >> width) & 0x8;
                            int8_t dataTmp = 0;
                            if (flag != 0)
                            {
                                dataTmp = -(16 - ((data[i] >> width) & 0xf));
                            }
                            else
                            {
                                dataTmp = ((data[i] >> width) & 0xf);
                            }

                            org = dataTmp + base;
                            base = org;
                            punpack_data[vec_idx++] = org;
                            block_len--;
                        }
                    }
                    else
                    {
                        uint16_t templen = block_len;
                        for (uint8_t k = 0; k < templen; k++)
                        {
                            width = 4 * k;
                            int8_t flag = (data[i] >> width) & 0x8;
                            int8_t dataTmp = 0;
                            if (flag != 0)
                            {
                                dataTmp = -(16 - ((data[i] >> width) & 0xf));
                            }
                            else
                            {
                                dataTmp = ((data[i] >> width) & 0xf);
                            }
                            org = dataTmp + base;
                            base = org;
                            punpack_data[vec_idx++] = org;
                            block_len--;
                        }
                    }
                    break;
                case ENC_DIFF://8bit 
                    if (block_len >= 2)
                    {
                        for (uint8_t k = 0; k < 2; k++)
                        {
                            width = 8 * k;
                            org = (int8_t)((data[i] >> width) & 0xff) + base;
                            base = org;
                            punpack_data[vec_idx++] = org;
                            block_len--;
                        }
                    }
                    else
                    {
                        uint16_t templen = block_len;
                        for (uint8_t k = 0; k < templen; k++)
                        {
                            width = 8 * k;
                            org = (int8_t)((data[i] >> width) & 0xff) + base;
                            base = org;
                            punpack_data[vec_idx++] = org;
                            block_len--;
                        }
                    }
                    break;
                case ENC_ORG://取原始值
                    org = data[i];
                    punpack_data[vec_idx++] = org;
                    block_len--;
                    break;
                }
            }

            if (block_len == 0) {
                // 处理块结束逻辑
                uint8_t flag = 1;
                while (block_len == 0 || flag)
                {
                    if (i >= data_len || vec_idx >= VIEW_H) {
                        endflag = 1;
                    }
                    if (type == ENC_IDENT) {
                        i--;
                    }
                    // 读取下一个块头
                    if (i + 2 < data_len)
                    {
                        type = (data[i + 1] >> 14) & 0x3;

                        block_len = (data[i + 1] & 0x3fff) - 1;
                        if (block_len >= VIEW_H)
                        {
                            return;
                        }
                        base = data[i + 2];
                        i = i + 2;

                        if (type == ENC_IDENT)
                        {
                            i++;
                        }
                        punpack_data[vec_idx++] = base;
                        flag = 0;
                    }
                    else
                    {
                        endflag = 1;
                        break;
                    }
                }
            }
            if (type != ENC_IDENT)
            {
                i++;
            }
        }
    }
}
};
}  // namespace lidar
}  // namespace robosense