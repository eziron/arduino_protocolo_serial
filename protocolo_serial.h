#include "Arduino.h"

#define port Serial1
#define synq_byte1 254
#define synq_byte2 252
class ProtocoloSerial{
    private:
        uint8_t tx_buffer[2048];
        uint8_t prev_rx_buffer[2048];
        uint32_t PS_time_ref=0;
        uint8_t rx_c=0;
        
        uint8_t rx_count=0;
        bool rx_lim_en = false;
        uint8_t rx_lim=0;
    public:
        ProtocoloSerial();

        bool read_command(uint8_t *out_rx_buffer,uint8_t *cmd, uint8_t *d_len, uint8_t *d_type, uint32_t time_out = 0);

        bool wait_response(uint32_t time_out);
        bool wait_response(uint32_t time_out,uint8_t buffer_target);

        void decode_uint16(uint8_t byte_buffer[], uint8_t d_len, uint16_t *out_buffer);
        void decode_int16 (uint8_t byte_buffer[], uint8_t d_len,  int16_t *out_buffer);
        void decode_uint32(uint8_t byte_buffer[], uint8_t d_len, uint32_t *out_buffer);
        void decode_int32 (uint8_t byte_buffer[], uint8_t d_len,  int32_t *out_buffer);
        void decode_uint64(uint8_t byte_buffer[], uint8_t d_len, uint64_t *out_buffer);
        void decode_int64 (uint8_t byte_buffer[], uint8_t d_len,  int64_t *out_buffer);

        void send_command(uint8_t cmd, uint8_t d_len,  int8_t    data_array[]);
        void send_command(uint8_t cmd, uint8_t d_len, uint8_t    data_array[]);
        void send_command(uint8_t cmd, uint8_t d_len,  int16_t   data_array[]);
        void send_command(uint8_t cmd, uint8_t d_len, uint16_t   data_array[]);
        void send_command(uint8_t cmd, uint8_t d_len,  int32_t   data_array[]);
        void send_command(uint8_t cmd, uint8_t d_len, uint32_t   data_array[]);
        void send_command(uint8_t cmd, uint8_t d_len,  int64_t   data_array[]);
        void send_command(uint8_t cmd, uint8_t d_len, uint64_t   data_array[]);
};
