// gcc -Wall -pthread -o bbSPIx_test bbSPIx_test.c -lpigpio
// sudo ./bbSPIx_test

#ifndef MCP3008_H
#define MCP3008_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "pigpio.h"
#include "utils.h"

#define CE0 8
#define CE1 7
#define MISO 9
#define MOSI 10
#define SCLK 11

namespace kruiser {
    class MCP3008 {
        public:
            MCP3008(uint8_t ceo, uint8_t miso, uint8_t mosi, uint8_t clk) {
                clk_ = clk;
                miso_ = miso;
                mosi_ = mosi;
                ceo_ = ceo;
                
                 bbSPIOpen(ceo_, miso_, mosi_, clk_, 20000, 3); // MCP3008 ADC
            }
     
            virtual ~MCP3008() {
                 bbSPIClose(ceo_);
            }
            
            int ir_to_cm(int value) {
                return map(value,7,880,30,4);
            }
        
            int readMulti(int * values, int numValues) {
                for(int i=0; i < numValues; i++) {
                    values[i] = read(i);
                }
            }
            
            int read(int adc_number) {
                int count, read_val;
                unsigned char inBuf[3];
                
                int cmd = 0b11 << 6;
                cmd |= (adc_number & 0x07) << 3;
                
                unsigned char outBuf[3] = {cmd,0x0,0x0};
                 
                count = bbSPIXfer(CE0, (char*) outBuf, (char *)inBuf, 3); // < ADC
                
                if(count == 3) {
                     read_val = (inBuf[0] & 0x01) << 9;
                     read_val |= (inBuf[1] & 0xFF) << 1;
                     read_val |= (inBuf[2] & 0x80) >> 7;
                     read_val &= 0x3FF;
                     
                }
                
                return ir_to_cm(read_val);
            }
            
        private:
            uint8_t ceo_, miso_, mosi_, clk_;
    };
}
#endif