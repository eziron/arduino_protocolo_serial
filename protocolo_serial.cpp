#include "Arduino.h"
#include "protocolo_serial.h"


ProtocoloSerial::ProtocoloSerial(){}

bool ProtocoloSerial::read_command(uint8_t *out_rx_buffer,uint8_t *cmd, uint8_t *d_len, uint8_t *d_type, uint32_t time_out){
    if(time_out > 0){
        wait_response(time_out);
    }
    while (port.available() > 0) {
        rx_c = port.read();
        if(
            (rx_count == 0 && rx_c == synq_byte1) ||
            (rx_count == 1 && rx_c == synq_byte2) ||
            (rx_count > 1)
        ) {
            prev_rx_buffer[rx_count] = rx_c;
            rx_count++;

            if(rx_lim_en){
                if (rx_count == rx_lim) {
                    for(int i = 5;i<rx_lim;i++){
                        out_rx_buffer[i-5] = prev_rx_buffer[i];
                    }
                    *cmd    = prev_rx_buffer[2];
                    *d_type = prev_rx_buffer[3];
                    *d_len  = prev_rx_buffer[4];
                    
                    rx_count = 0;
                    return true;
                }
            }
            else{
                if(rx_count == 5){
                    switch (prev_rx_buffer[3]){
                        case 'b':
                            rx_lim = rx_c+5;
                            rx_lim_en = true;
                            break;
                        
                        case 'B':
                            rx_lim = rx_c+5;
                            rx_lim_en = true;
                            break;

                        case 'h':
                            rx_lim = (rx_c*2)+5;
                            rx_lim_en = true;
                            break;

                        case 'H':
                            rx_lim = (rx_c*2)+5;
                            rx_lim_en = true;
                            break;
                        
                        case 'i':
                            rx_lim = (rx_c*4)+5;
                            rx_lim_en = true;
                            break;

                        case 'I':
                            rx_lim = (rx_c*4)+5;
                            rx_lim_en = true;
                            break;

                        case 'l':
                            rx_lim = (rx_c*4)+5;
                            rx_lim_en = true;
                            break;
                        
                        case 'L':
                            rx_lim = (rx_c*4)+5;
                            rx_lim_en = true;
                            break;

                        case 'q':
                            rx_lim = (rx_c*8)+5;
                            rx_lim_en = true;
                            break;
                        
                        case 'Q':
                            rx_lim = (rx_c*8)+5;
                            rx_lim_en = true;
                            break;
                        
                        default:
                            rx_lim = 0;
                            rx_lim_en = false;
                            rx_count = 0;
                            break;
                    }
                }
            }
        }
        else {
            rx_count = 0;
            rx_lim_en = false;
            return false;
        }
    }
    return false;
}

void ProtocoloSerial::decode_uint16(uint8_t byte_buffer[], uint8_t d_len, uint16_t *out_buffer){
    uint8_t n_base;
    for(uint8_t i=0;i<d_len;i++){
        n_base = (i*2);
        out_buffer[i] = (uint16_t) (byte_buffer[n_base]<<8) | byte_buffer[n_base+1];
    }
}

void ProtocoloSerial::decode_int16(uint8_t byte_buffer[], uint8_t d_len, int16_t *out_buffer){
    uint8_t n_base;
    for(uint8_t i=0;i<d_len;i++){
        n_base = (i*2);
        out_buffer[i] = (int16_t) (byte_buffer[n_base]<<8) | byte_buffer[n_base+1];
    }
}

void ProtocoloSerial::decode_uint32(uint8_t byte_buffer[], uint8_t d_len, uint32_t *out_buffer){
    uint8_t n_base;
    for(uint8_t i=0;i<d_len;i++){
        n_base = (i*4);
        out_buffer[i] = (uint32_t) (byte_buffer[n_base]<<24) | (byte_buffer[n_base+1]<<16) | (byte_buffer[n_base+2]<<8) | byte_buffer[n_base+3];
    }
}

void ProtocoloSerial::decode_int32(uint8_t byte_buffer[], uint8_t d_len, int32_t *out_buffer){
    uint8_t n_base;
    for(uint8_t i=0;i<d_len;i++){
        n_base = (i*4);
        out_buffer[i] = (int32_t) (byte_buffer[n_base]<<24) | (byte_buffer[n_base+1]<<16) | (byte_buffer[n_base+2]<<8) | byte_buffer[n_base+3];
    }
}

void ProtocoloSerial::decode_uint64(uint8_t byte_buffer[], uint8_t d_len, uint64_t *out_buffer){
    uint8_t n_base;
    for(uint8_t i=0;i<d_len;i++){
        n_base = (i*8);
        out_buffer[i] = 0;
        for(uint8_t j = 7;j>=0;j--){
            out_buffer[i] = out_buffer[i]<<8;
            out_buffer[i] |= (uint64_t)byte_buffer[n_base+j];
        }
        //out_buffer[i] = (uint64_t) (byte_buffer[n_base]<<56) || (byte_buffer[n_base+1]<<48) || (byte_buffer[n_base+2]<<40) || (byte_buffer[n_base+3]<<32) || (byte_buffer[n_base+4]<<24) || (byte_buffer[n_base+5]<<16) || (byte_buffer[n_base+6]<<8) || byte_buffer[n_base+7];
    }
}

void ProtocoloSerial::decode_int64(uint8_t byte_buffer[], uint8_t d_len, int64_t *out_buffer){
    uint8_t n_base;
    for(uint8_t i=0;i<d_len;i++){
        n_base = (i*8);
        out_buffer[i] = 0;
        for(uint8_t j = 7;j>=0;j--){
            out_buffer[i] = out_buffer[i]<<8;
            out_buffer[i] |= (uint64_t)byte_buffer[n_base+j];
        }
        //out_buffer[i] = (int64_t) (byte_buffer[n_base]<<56) || (byte_buffer[n_base+1]<<48) || (byte_buffer[n_base+2]<<40) || (byte_buffer[n_base+3]<<32) || (byte_buffer[n_base+4]<<24) || (byte_buffer[n_base+5]<<16) || (byte_buffer[n_base+6]<<8) || byte_buffer[n_base+7];
    }
}

void ProtocoloSerial::send_command(uint8_t cmd, uint8_t d_len, int8_t data_array[]){
    tx_buffer[0] = synq_byte1;
    tx_buffer[1] = synq_byte2;

    tx_buffer[2] = cmd;
    tx_buffer[3] = 'b';
    tx_buffer[4] = d_len;

    for(uint8_t i = 0; i<d_len;i++){
        tx_buffer[i+5] = data_array[i];
    }

    port.write(tx_buffer,d_len+5);
}

void ProtocoloSerial::send_command(uint8_t cmd, uint8_t d_len, uint8_t data_array[]){
    tx_buffer[0] = synq_byte1;
    tx_buffer[1] = synq_byte2;

    tx_buffer[2] = cmd;
    tx_buffer[3] = 'B';
    tx_buffer[4] = d_len;

    for(uint8_t i = 0; i<d_len;i++){
        tx_buffer[i+5] = data_array[i];
    }

    port.write(tx_buffer,d_len+5);
}

void ProtocoloSerial::send_command(uint8_t cmd, uint8_t d_len, int16_t data_array[]){
    tx_buffer[0] = synq_byte1;
    tx_buffer[1] = synq_byte2;

    tx_buffer[2] = cmd;
    tx_buffer[3] = 'h';
    tx_buffer[4] = d_len;

    for(uint8_t i = 0; i<d_len;i++){
        uint8_t n_base = (i*2)+5;

        tx_buffer[n_base] = data_array[i]>>8;
        tx_buffer[n_base+1] = data_array[i];
    }

    port.write(tx_buffer,(d_len*2)+5);
}

void ProtocoloSerial::send_command(uint8_t cmd, uint8_t d_len, uint16_t data_array[]){
    tx_buffer[0] = synq_byte1;
    tx_buffer[1] = synq_byte2;

    tx_buffer[2] = cmd;
    tx_buffer[3] = 'H';
    tx_buffer[4] = d_len;

    for(uint8_t i = 0; i<d_len;i++){
        uint8_t n_base = (i*2)+5;

        tx_buffer[n_base] = data_array[i]>>8;
        tx_buffer[n_base+1] = data_array[i];
    }

    port.write(tx_buffer,(d_len*2)+5);
}

void ProtocoloSerial::send_command(uint8_t cmd, uint8_t d_len, int32_t data_array[]){
    tx_buffer[0] = synq_byte1;
    tx_buffer[1] = synq_byte2;

    tx_buffer[2] = cmd;
    tx_buffer[3] = 'i';
    tx_buffer[4] = d_len;

    for(uint8_t i = 0; i<d_len;i++){
        uint8_t n_base = (i*4)+5;

        tx_buffer[n_base] = data_array[i]>>24;
        tx_buffer[n_base+1] = data_array[i]>>16;
        tx_buffer[n_base+2] = data_array[i]>>8;
        tx_buffer[n_base+3] = data_array[i];
    }

    port.write(tx_buffer,(d_len*4)+5);
}

void ProtocoloSerial::send_command(uint8_t cmd, uint8_t d_len, uint32_t data_array[]){
    tx_buffer[0] = synq_byte1;
    tx_buffer[1] = synq_byte2;

    tx_buffer[2] = cmd;
    tx_buffer[3] = 'I';
    tx_buffer[4] = d_len;

    for(uint8_t i = 0; i<d_len;i++){
        uint8_t n_base = (i*4)+5;

        tx_buffer[n_base] = data_array[i]>>24;
        tx_buffer[n_base+1] = data_array[i]>>16;
        tx_buffer[n_base+2] = data_array[i]>>8;
        tx_buffer[n_base+3] = data_array[i];
    }

    port.write(tx_buffer,(d_len*4)+5);
}

void ProtocoloSerial::send_command(uint8_t cmd, uint8_t d_len, int64_t data_array[]){
    tx_buffer[0] = synq_byte1;
    tx_buffer[1] = synq_byte2;

    tx_buffer[2] = cmd;
    tx_buffer[3] = 'q';
    tx_buffer[4] = d_len;

    for(uint8_t i = 0; i<d_len;i++){
        uint8_t n_base = (i*8)+5;

        tx_buffer[n_base] = data_array[i]>>56;
        tx_buffer[n_base+1] = data_array[i]>>48;
        tx_buffer[n_base+2] = data_array[i]>>40;
        tx_buffer[n_base+3] = data_array[i]>>32;
        tx_buffer[n_base+4] = data_array[i]>>24;
        tx_buffer[n_base+5] = data_array[i]>>16;
        tx_buffer[n_base+6] = data_array[i]>>8;
        tx_buffer[n_base+7] = data_array[i];
    }

    port.write(tx_buffer,(d_len*8)+5);
}

void ProtocoloSerial::send_command(uint8_t cmd, uint8_t d_len, uint64_t data_array[]){
    tx_buffer[0] = synq_byte1;
    tx_buffer[1] = synq_byte2;

    tx_buffer[2] = cmd;
    tx_buffer[3] = 'Q';
    tx_buffer[4] = d_len;

    for(uint8_t i = 0; i<d_len;i++){
        uint8_t n_base = (i*8)+5;

        tx_buffer[n_base] = data_array[i]>>56;
        tx_buffer[n_base+1] = data_array[i]>>48;
        tx_buffer[n_base+2] = data_array[i]>>40;
        tx_buffer[n_base+3] = data_array[i]>>32;
        tx_buffer[n_base+4] = data_array[i]>>24;
        tx_buffer[n_base+5] = data_array[i]>>16;
        tx_buffer[n_base+6] = data_array[i]>>8;
        tx_buffer[n_base+7] = data_array[i];
    }

    port.write(tx_buffer,(d_len*8)+5);
}

bool ProtocoloSerial::wait_response(uint32_t time_out){
    PS_time_ref = millis();
    while (port.available() == 0 && millis() - PS_time_ref < time_out)
    {
        delay(1);
    }
    return port.available() > 0;
}

bool ProtocoloSerial::wait_response(uint32_t time_out,uint8_t buffer_target){
    PS_time_ref = millis();
    while (port.available() < buffer_target && millis() - PS_time_ref < time_out)
    {
        delay(1);
    }
    return port.available() >= buffer_target;
}