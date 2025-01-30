/*
 * Copyright (c) PeeWee Labs All rights reserved.
 * Licensed under the MIT License.
 * See LICENSE file in the project root for full license information.
 */

#include <pwl_rfm9X.h>

// #define PWL_RFM9X_DEBUG

uint8_t PWL_RFM9X::_read_byte(uint8_t addr)
{
    uint8_t data;
    if (_read_func((uint8_t) addr, &data, 1) == 0)
        return data;
    else
        return 0xFF;
}


int PWL_RFM9X::_write_byte(uint8_t addr, uint8_t data)
{
    int rval = _write_func(addr | 0x80, &data, 1);
    return rval;
}


int PWL_RFM9X::set_center_frequency(uint32_t Hz)
{
    float freq_units = ((float)Hz) / (PWL_RFM9X_BASE_CLOCK_FREQENCY / 524288.0);
    uint32_t fr = (uint32_t)freq_units;
    _write_byte(RFM9X_REG_FrfMsb, (fr >> 16) & 0xFF);
    _write_byte(RFM9X_REG_FrfMid, (fr >>  8) & 0xFF);
    _write_byte(RFM9X_REG_FrfLsb, (fr >>  0) & 0xFF);
    _freq = Hz;
    return PWL_RFM9X_STATUS_GOOD;
}


int PWL_RFM9X::set_power_amp(int dBm)
{
    if (dBm < 5 || dBm > 20) return PWL_RFM9X_STATUS_BAD;
    _write_byte(RFM9X_REG_PaConfig, 0x80 | (dBm - 5));
    _write_byte(RFM9X_REG_PaDac, 0x87);
    return PWL_RFM9X_STATUS_GOOD;
}


int PWL_RFM9X::set_mode(lora_mode_t mode)
{
    uint8_t mode_byte;

    if (mode == RFM9X_LORA_MODE_RX)
        _rx_valid = false;

    _mode = mode;

    mode_byte = (uint8_t) mode;
    if (mode == RFM9X_LORA_MODE_SLEEP)
        mode_byte |= 0x80;

    _write_byte(RFM9X_REG_OpMode, mode_byte);

    // The radio can take some time to switch into STDBY or SLEEP modes
    // during which time it will not respond to other commands.  So give
    // it some time here.
    if (_mode == RFM9X_LORA_MODE_SLEEP || _mode == RFM9X_LORA_MODE_STDBY)
        _delay_func(25);

    return PWL_RFM9X_STATUS_GOOD;
}


uint8_t PWL_RFM9X::get_mode()
{
    _mode = _read_byte(RFM9X_REG_OpMode) & 0x07;
    return _mode;
}


int PWL_RFM9X::init(uint32_t Hz,
                    int dBm,
                    lora_bw_t bw,
                    lora_cr_t cr,
                    lora_sf_t sf)
{
    uint8_t config;

    // Force a short delay in case this is POR
    _delay_func(20);

    // Must go into sleep mode first to set the LORA bit
    set_mode(RFM9X_LORA_MODE_SLEEP);
    set_mode(RFM9X_LORA_MODE_STDBY);

    // If the radio is there, then we should be able to read mode as standby
    if (get_mode() != (RFM9X_LORA_MODE_STDBY & 0x7)) return PWL_RFM9X_STATUS_BAD;

    // Clear any pending interrupts and set masks
    _write_byte(RFM9X_REG_IrqFlagsMask, ~PWL_RFM9X_INTERRUPT_STATUS_MASK);
    _write_byte(RFM9X_REG_IrqFlags, 0b11111111);

    _write_byte(RFM9X_REG_FifoTxBaseAddr, 0);
    _write_byte(RFM9X_REG_FifoRxBaseAddr, 0);

    config  = (uint8_t) bw << RFM9X_LORA_BW_BitPos;
    config |= (uint8_t) cr << RFM9X_LORA_CR_BitPos;
    _write_byte(RFM9X_REG_ModemConfig1, config);
    config  = (uint8_t) sf << RFM9X_LORA_SF_BitPos;
    config |= 0b00000100;  // Enable the TX CRC
    _write_byte(RFM9X_REG_ModemConfig2, config);
    _write_byte(RFM9X_REG_ModemConfig3, 0x04); // Alway set AgcAutoOn = 1

    set_center_frequency(Hz);

    set_power_amp(dBm);

    return PWL_RFM9X_STATUS_GOOD;
}


int PWL_RFM9X::poll( )
{
    uint8_t  iflags;
    uint8_t  caddr;
    int8_t   snr;
    int32_t  i32;

    iflags = _read_byte(RFM9X_REG_IrqFlags);
    _write_byte(RFM9X_REG_IrqFlags, iflags);
#ifdef PWL_RFM9X_DEBUG
    Serial.print("Poll: ");
    Serial.print(iflags, HEX);
    Serial.print(" ");
    Serial.print(_mode, HEX);
    Serial.print(" ");
    Serial.println(_read_byte(RFM9X_REG_OpMode) & 0x07, HEX);
    delay(100);
#endif
    iflags &= PWL_RFM9X_INTERRUPT_STATUS_MASK;
    if (!iflags) return PWL_RFM9X_POLL_NO_STATUS;

    if (_mode == RFM9X_LORA_MODE_RX_CONTINUOUS)
    {
        if (!(iflags & PWL_RFM9X_RX_DONE_INTERRUPT_FLAG))
        {
            // It is an error condition to be in this if statement
            // since while in the RX, any interrupt that gets us here
            // should include the RX Done flag.  So, to recover,
            // simply return to Standby and Restart the RX
            set_mode(RFM9X_LORA_MODE_STDBY);
            set_mode(RFM9X_LORA_MODE_RX_CONTINUOUS);
            return PWL_RFM9X_POLL_RX_ERROR;
        }

        // RX is done... check that we have a valid CRC
        // If not, then restart the RX process and wait for another packet
        if (((_read_byte(RFM9X_REG_HopChannel) & 0b01000000) == 0)
            || (iflags & PWL_RFM9X_RX_CRC_ERROR_FLAG) )
        {
            set_mode(RFM9X_LORA_MODE_STDBY);
            set_mode(RFM9X_LORA_MODE_RX_CONTINUOUS);
            return PWL_RFM9X_POLL_RX_ERROR;
        }

        _rxlength = _read_byte(RFM9X_REG_RxNbBytes);
        caddr = _read_byte(RFM9X_REG_FifoRxCurrentAddr);
        _write_byte(RFM9X_REG_FifoAddrPtr, caddr);
        _read_func(RFM9X_REG_Fifo, _buffer, _rxlength);

        set_mode(RFM9X_LORA_MODE_STDBY);

        // Calculate the RSSI
        i32 = (int32_t) _read_byte(RFM9X_REG_PktRssiValue);
        snr = ((int8_t) _read_byte(RFM9X_REG_PktSnrValue)) >> 2;
        if (snr >= 0)
            i32 = (i32 << 4) / 15;
        else
            i32 += snr;
        if (_freq <= 525000000) // LF output
            _rssi = (int16_t)(i32 - 164);
        else
            _rssi = (int16_t)(i32 - 157);

        _rx_valid = true;

        return PWL_RFM9X_POLL_RX_READY;
    }
    else if (_mode == RFM9X_LORA_MODE_TX)
    {
        if ((iflags & 0b00001000) == 0b00001000)
        {
            set_mode(RFM9X_LORA_MODE_STDBY);
            return PWL_RFM9X_POLL_TX_DONE;
        }
    }

    return PWL_RFM9X_POLL_NO_STATUS;
}


bool PWL_RFM9X::rx_data_ready()
{
    if (!_rx_valid)
        if (poll() == PWL_RFM9X_POLL_NO_STATUS)
            if ((_mode != RFM9X_LORA_MODE_RX_CONTINUOUS) && (_mode != RFM9X_LORA_MODE_FSRX))
                set_mode(RFM9X_LORA_MODE_RX_CONTINUOUS);
    return _rx_valid;
}


uint8_t PWL_RFM9X::receive(uint8_t* buf, uint8_t* len)
{
    if (rx_data_ready())
    {
        uint8_t idx = 0;
        while (idx < *len && idx < _rxlength)
        {
            buf[idx] = _buffer[idx];
            ++idx;
        }
        *len = idx;
        _rx_valid = false;
        return idx;
    }

    return 0;
}


int PWL_RFM9X::send(uint8_t* data, uint8_t len)
{
    set_mode(RFM9X_LORA_MODE_STDBY);
    _write_byte(RFM9X_REG_FifoAddrPtr, 0);
    _write_func(RFM9X_REG_Fifo | 0x80, data, len);
    _write_byte(RFM9X_REG_PayloadLength, len);
    set_mode(RFM9X_LORA_MODE_TX);
    return PWL_RFM9X_STATUS_GOOD;
}


int PWL_RFM9X::wait_packet_tx(int max_wait_ms)
{
    while (1)
    {
        poll();

        if ((_mode != RFM9X_LORA_MODE_TX) && (_mode != RFM9X_LORA_MODE_FSTX))
            break;
        
        --max_wait_ms;
        if (!max_wait_ms)
        {
            set_mode(RFM9X_LORA_MODE_STDBY);
            return PWL_RFM9X_STATUS_BAD;
        }

        _delay_func(50);
    }

    return PWL_RFM9X_STATUS_GOOD;
}
