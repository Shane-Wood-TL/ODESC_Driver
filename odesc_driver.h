/**
 * @file odesc_driver.h
 * @brief This is the header for the odesc class
 * @author Shane Wood
*/

#ifndef __odesc__
#define __odesc__
#include "esp_twai.h"
#include <cstring>
#include "./odesc_commands.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define bytes_in_float 4 ///< the number of bytes in a float
#define max_message_size 8 ///< the maximum size of a message
#define node_id_offset 5 ///< the offset of the node id in the message
#define lower_five_bits 0x1F ///< the lower five bits of a byte

/**
 * @class odesc
 * @brief This is the class to control an odesc motor controller
 */
class odesc {
   private:
    uint8_t node_ID;  ///< node id as set in the odesc configurator
    float battery_voltage; ///< the battery voltage as read from the odesc
    twai_node_handle_t twai_handle; ///< the twai node handle for the odesc
    /**
     * @brief the function that is called to update the display's battery voltage
     *
     * @param rx_message : twai_message_t the message that has been received containing the id of Get_Bus_Voltage_Current
     */
    void update_battery_voltage(twai_frame_t rx_message);

    /**
     * @brief the function that handles when a message needs to be received from the odesc
     *
     * @param message_ID : the id of the message that is going to be recived
     */
    void receive_message(uint8_t message_ID);

    /**
     * @brief the function that is called when something has gone wrong, and no message is received when Get_Bus_Voltage_Current is called
     */
    void clear_battery_voltage();


    
    /**
     * @brief Read the motor velocity from an odesc
     * 
     * @param rx_message : twai_message_t the message that has been received containing the id of Get_Encoder_Estimates
     * @details This function will read the velocity of the motor from the odesc and update the velocity estimate
     */
    void update_current_motor_velocity(twai_frame_t rx_message);


   public:
    /**
     * @brief The constructor for the odesc class
     *
     * @param node_IDV : uint8_t the value of the node id to have commands sent to
     * @param twai_handle : twai_node_handle_t the TWAI node handle
     */
    odesc(uint8_t node_ID, twai_node_handle_t twai_handle);

    /**
     * @brief send a command to the odesc
     *
     * @param message_ID : const uint8_t the ID of the message to be sent
     * @param message_contents : const uint8_t* the data to be sent with the message
     * @param message_length : uint8_t the length of the data
     * @param data_returned : const bool if data is returned (determines if receive_message is called or not)
     */
    void send_message(const uint8_t message_ID, const uint8_t* message_contents, uint8_t message_length, const bool data_returned);


};


/*
```plantuml
@startuml
skinparam classAttributeIconSize 0
hide circle

class odesc {
    - node_ID : uint8_t
    - update_battery_voltage(rx_message : twai_message_t) : void
    - receive_message(message_ID : uint8_t) : void
    - clear_battery_voltage() : void
    - update_current_motor_velocity(rx_message : twai_message_t) : void
    + odesc(node_IDV : uint8_t)
    + send_message(message_ID : const uint8_t, message_contents : const uint8_t*, message_length : uint8_t, data_returned : const bool) : void
}

@enduml
```
*/
#endif