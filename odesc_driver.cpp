/**
 * @file odsec_driver.h
 * @brief This is the source file for the odesc class
 * @author Shane Wood
*/

#include "./odesc_driver.h"

odesc::odesc(uint8_t node_ID, twai_node_handle_t twai_handle)
{
    this->node_ID = node_ID;
    this->twai_handle = twai_handle;
    this-> battery_voltage = 0;
}



void odesc::send_message(const uint8_t message_ID, const uint8_t *message_contents, uint8_t message_length, const bool data_returned)
{
    // max data size is 8 bytes
    if (message_length > max_message_size)
    {
        return;
    }
    if (message_contents == nullptr)
    {
        message_length = 0;
    }

    // setup can node + message ID with some bit manipulation to ensure the messageID is the least sigificant node_id_offset bits
    uint32_t can_ID = (node_ID << node_id_offset) | (message_ID & lower_five_bits);

   

    // setup the message to transmit
    twai_frame_t tx_msg {};
    tx_msg.header.id = can_ID, // Message ID
    tx_msg.header.ide = false, // Use Normal Frame Format (11-bit ID)
    (void)memcpy(tx_msg.buffer, message_contents, message_length); // copy the message data
    tx_msg.buffer_len = message_length;  // Length of data to transmit

    uint8_t recv_buff[128] = {0}; // buffer to receive the data
    twai_frame_t rx_frame{};
    rx_frame.buffer = recv_buff;
    rx_frame.buffer_len = sizeof(recv_buff);
    rx_frame.tx_queue_priority = 0;

    if(data_returned){
        while (twai_node_receive_from_isr(twai_handle, &rx_frame) == ESP_OK)
        {
        }
    }

    esp_err_t result = twai_node_transmit(twai_handle, &tx_msg, pdMS_TO_TICKS(200));
    // if the data was sent successfully, and a response is expected
    if (data_returned && (result == ESP_OK))
    {
        receive_message(message_ID);
    }
    else if (data_returned && (result != ESP_OK))
    {
        // if the transmit failed, and data is expected, set the battery voltage to 0
        // if the transmit failed the, then the receive will not be called
        if (message_ID == Get_Bus_Voltage_Current)
        {
            clear_battery_voltage();
        }
    }
}




// not used in the code, but was used for testing to detect the heartbeat
// messages that are sending data to the microcontroller have a true for sendMessage()
void odesc::receive_message(uint8_t message_ID)
{
    uint8_t recv_buff[128] = {0}; // buffer to receive the data
    twai_frame_t rx_frame{};
    rx_frame.buffer = recv_buff;
    rx_frame.buffer_len = sizeof(recv_buff);
    rx_frame.tx_queue_priority = 0;

    esp_err_t result = (twai_node_receive_from_isr(twai_handle, &rx_frame) == ESP_OK);

    if (result == ESP_OK)
    {
        // majority of these are not implemented as they are not needed, only the battery voltage is needed
        // if (rx_frame.header.id == ((node_ID << node_id_offset) | Get_Version))
        // {
        // }
        // if (rx_frame.header.id == ((node_ID << node_id_offset) | Get_Error))
        // {
        // }
        if (rx_frame.header.id == ((node_ID << node_id_offset) | Get_Encoder_Estimates))
        {
            update_current_motor_velocity(rx_frame);
        }
        else if (rx_frame.header.id == ((node_ID << node_id_offset) | Get_Iq))
        {
        }
        // else if (rx_frame.header.id == ((node_ID << node_id_offset) | Get_Temperature))
        // {
        // }
        else if (rx_frame.header.id == ((node_ID << node_id_offset) | Get_Bus_Voltage_Current))
        {
            update_battery_voltage(rx_frame);   
        }
        // else if (rx_frame.header.id == ((node_ID << node_id_offset) | Get_Torques))
        // {
        // }
        // else if (rx_frame.header.id == ((node_ID << node_id_offset) | Get_Powers))
        // {
        // }
    }
    else
    {
        // if the data was not received, set the battery voltage to 0
        if (message_ID == Get_Bus_Voltage_Current)
        {
            clear_battery_voltage();
        }
    }
}




void odesc::update_battery_voltage(twai_frame_t rx_message)
{
    union
    {
        float a;
        uint8_t bytes[bytes_in_float];
    } temp_union;

    // convert the 4 bytes to a float
    temp_union.bytes[0] = rx_message.buffer[0];
    temp_union.bytes[1] = rx_message.buffer[1];
    temp_union.bytes[2] = rx_message.buffer[2];
    temp_union.bytes[3] = rx_message.buffer[3];
    // update the value to be displayed

    battery_voltage = temp_union.a;
}



void odesc::clear_battery_voltage()
{
    battery_voltage = 0;
}



void odesc::update_current_motor_velocity(twai_frame_t rx_message){
    union
    {
        float a;
        uint8_t bytes[bytes_in_float];
    } temp_union;

    // convert the 4 bytes to a float
    temp_union.bytes[0] = rx_message.buffer[4];
    temp_union.bytes[1] = rx_message.buffer[5];
    temp_union.bytes[2] = rx_message.buffer[6];
    temp_union.bytes[3] = rx_message.buffer[7];

    //battery_voltage = temp_union.a;
}