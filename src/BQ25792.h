#pragma once
#ifndef bq25792_h_
#define bq25792_h_
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

namespace EmbeddedDevices
{
    
    template <int CELL>
    class BQ25792  
    {       
      private:
        const uint8_t I2C_ADDR = 0x6B;
        enum class REG
        {
            VSYSMIN        = 0x00,
            VREG           = 0x01,
            ICHG           = 0x03,
            VINDPM         = 0x05,
            IINDPM         = 0x06,
            PRE_CHARGE     = 0x08,
            TERM_REG       = 0x09,
            RECHG          = 0x0A,
            VOTG           = 0x0B,
            IOTG           = 0x0D,
            EN_TCR         = 0x0E,
            EN_CCR0        = 0x0F,
            EN_CCR1        = 0x10,
            EN_CCR2        = 0x11,
            EN_CCR3        = 0x12,
            EN_CCR4        = 0x13,
            EN_CCR5        = 0x14,
            RSRV_REG       = 0x15,
            TEMP_REG       = 0x16,
            NTC_REG0       = 0x17,
            NTC_REG1       = 0x18,
            ICO_CLR        = 0x19,
            CHG_STATUS0    = 0x1B,
            CHG_STATUS1    = 0x1C,
            CHG_STATUS2    = 0x1D,
            CHG_STATUS3    = 0x1E,
            CHG_STATUS4    = 0x1F,
            FAULT_STATUS0  = 0x20,
            FAULT_STATUS1  = 0x21,
            CH_FLAGREG0    = 0x22,
            CH_FLAGREG1    = 0x23,
            CH_FLAGREG2    = 0x24,
            CH_FLAGREG3    = 0x25,
            FAULT_FLAGREG0 = 0x26,
            FAULT_FLAGREG1 = 0x27,
            CH_MASKREG0    = 0x28,
            CH_MASKREG1    = 0x29,
            CH_MASKREG2    = 0x2A,
            CH_MASKREG3    = 0x2B,
            FAULT_MASKREG0 = 0x2C,
            FAULT_MASKREG1 = 0x2D,
            ADC_CTRL       = 0x2E,
            ADC_DISABLE0   = 0x2F,
            ADC_DISABLE1   = 0x30,
            IBUS_ADC       = 0x31,
            IBAT_ADC       = 0x33,
            VBUS_ADC       = 0x35,
            VAC1_ADC       = 0x37,
            VAC2_ADC       = 0x39,
            VBAT_ADC       = 0x3B,
            VBAT_ADC2      = 0x3C,
            VSYS_ADC       = 0x3D,
            TS_ADC         = 0X3F,
            TDIE_ADC       = 0x41,
            DP_ADC         = 0x43,
            DN_ADC         = 0x45,
            DPDM_ADC       = 0x47,
            PART_INFO      = 0x48
        };
        void printBinary(byte inByte)
        {
          for (int b = 7; b >= 0; b--)
          {
            Serial.print(bitRead(inByte, b));
          }
        }

        void write(const REG reg, const bool stop = true)
        {
            wire->beginTransmission(I2C_ADDR);
            wire->write((uint8_t)reg);
            wire->endTransmission(stop);
        }

        void write_(const REG reg, const uint8_t data, const bool stop = true)
        {
            wire->beginTransmission(I2C_ADDR);
            wire->write((uint8_t)reg);
            wire->write(data);
            wire->endTransmission(stop);
        }

        byte read(const REG reg)                                             // for reading single byte
        {
            byte data = 0;
            write(reg, false);
            wire->requestFrom((uint8_t)I2C_ADDR, (uint8_t)1);
            if(wire->available()==1)
            {  
                data = wire->read();
            }
            return data;
        }
        
        int readW(const REG reg)                                         // Read multiple bytes for 16-bit ADC
        {
            int16_t value = 0;
            byte buffer1[2];
            write(reg, false);
            wire->requestFrom((uint8_t)I2C_ADDR, (uint8_t)2);
            if(wire->available())
            {
                Serial.println("Two Bytes available");
                buffer1[0] = wire->read();
                buffer1[1] = wire->read();
            }
            else
            {
              Serial.println("Not available");
            }
            value = (buffer1[1]<<8) | buffer1[0];
            Serial.print("Value is: ");
            Serial.println(value);
            return value;
        }
        
        TwoWire* wire;
        float VBUS, VSYS, VBAT, IBAT, IBUS;
        
        void setADC_enabled(void)
        { 
            byte data = read(REG::ADC_CTRL);
            data |= (1UL << (7));       // start A/D convertion
            write_(REG::ADC_CTRL, data); 
        }
        void takeVBUSData(void)
        {
            int16_t data = readW(REG::VBUS_ADC);
            VBUS = 0;
            VBUS  += (float) data *0.01f;
        }
        
        void takeVSYSData(void)
        {
            int16_t data = readW(REG::VSYS_ADC);
            VSYS = 0;
            VSYS  += (float) data *0.01f;
        }

        void takeVBATData()
        {
            int data = readW(REG::VBAT_ADC);
            VBAT = 0;
            VBAT  += (float)data *0.01f;         
        }

        void takeIBUSData(void)
        {
            int16_t data = readW(REG::IBUS_ADC);
            IBUS = 0;
            IBUS  += (float) data *0.01f;
        }
        
        void takeIBATData(void)
        {
            int16_t data = readW(REG::IBAT_ADC);
            IBAT = 0;
            IBAT  += (float) data *0.01f;
        } 

    public:
        BQ25792(TwoWire& w) : wire(&w){};
        void begin(void) {setADC_enabled();}
        void properties(void)
        { 
            setADC_enabled();
            takeIBUSData();
            takeIBATData();
            takeVBUSData();
            takeVBATData();
            takeVSYSData();
        } 
        float getIBUS(void) {return IBUS;}
        float getIBAT(void) {return IBAT;}
        float getVBUS(void) {return VBUS;}
        float getVBAT(void) {return VBAT;}
        float getVSYS(void) {return VSYS;}
    };
}
using BQ25792 = EmbeddedDevices::BQ25792<1>;
#endif


