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
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "../can_manager/can_manager.h"
#include "../can_manager/can_driver.h"
#include "odesc_config.h"
#define bytes_in_float 4 ///< the number of bytes in a float

/**
 * @class odesc
 * @brief This is the class to control an odesc motor controller
 */
class odesc : public canDriver {
    private:

        // Values read from motor
        #ifdef USE_HEARTBEAT
            uint32_t axis_error; ///< Heartbeat
            uint8_t axis_current_state; ///< Heartbeat
            bool motor_error_flag;  ///< Heartbeat
            bool encoder_error_flag; ///< Heartbeat
            bool controller_error_flag; ///< Heartbeat
            bool trajectory_done_flag; ///< Heartbeat
        #endif

        #ifdef USE_GET_MOTOR_ERROR
            uint64_t motor_error; ///< Get Motor Error
        #endif
        
        #ifdef USE_GET_ENCODER_ERROR
            uint32_t encoder_error; ///< Get Encoder Error
        #endif
        
        #ifdef USE_GET_SENSORLESS_ERROR
            uint32_t sensorless_error; ///< Get Sensorless Error
        #endif

        #ifdef USE_GET_ENCODER_ESTIMATES
            float encoder_position_estimate; ///< Get Encoder Estimates
            float encoder_velocity_estimate; ///< Get Encoder Estimates
        #endif

        #ifdef USE_GET_ENCODER_COUNT
            int32_t encoder_shadow_count; ///< Get Encoder Count
            int32_t encoder_count_in_cpr; ///< Get Encoder Count
        #endif

        #ifdef USE_GET_IQ
            float current_setpoint; ///< Get IQ
            float current_measured; ///< Get IQ
        #endif

        #ifdef USE_GET_SENSORLESS_ESTIMATES
            float sensorless_position_estimate; ///< Get Sensorless Estimates
            float sensorless_velocity_estimate; ///< Get Sensorless Estimates
        #endif

        #ifdef USE_GET_BUS_VOLTAGE_CURRENT
            float battery_voltage; ///< Get Bus Voltage and Current
            float bus_current; ///< Get Bus Voltage and Current
        #endif

        #ifdef USE_GET_ADC_VOLTAGE
            float adc_voltage; ///< Get ADC Voltage
        #endif
        
        #ifdef USE_GET_CONTROLLER_ERROR
            uint32_t controller_error; ///< Get Controller Error
        #endif
        
        
        float convert_to_float(const uint8_t *bytes);

        

        void handle_receive_error(uint8_t message_ID);

    public:
        /**
         * @brief The constructor for the odesc class
         *
         * @param node_IDV : uint8_t the value of the node id to have commands sent to
         * @param twai_handle : twai_node_handle_t the TWAI node handle
         */
        odesc(uint8_t node_ID, twai_node_handle_t handle);

        
        #ifdef USE_HEARTBEAT
            void update_heartbeat();

            uint32_t get_axis_error() { return axis_error; }
            uint8_t get_axis_current_state() { return axis_current_state; }
            bool get_motor_error_flag() { return motor_error_flag; }
            bool get_encoder_error_flag() { return encoder_error_flag; }
            bool get_controller_error_flag() { return controller_error_flag; }
            bool get_trajectory_done_flag() { return trajectory_done_flag; }
        #endif

        #ifdef USE_GET_MOTOR_ERROR
            void update_motor_error();

            uint64_t get_motor_error() { return motor_error; } ///< Get Motor Error
        #endif
        
        #ifdef USE_GET_ENCODER_ERROR
            void update_encoder_error();

            uint32_t get_encoder_error() { return encoder_error; } ///< Get Encoder Error
        #endif
        
        #ifdef USE_GET_SENSORLESS_ERROR
            void update_sensorless_error();

            uint32_t get_sensorless_error() { return sensorless_error; } ///< Get Sensorless Error
        #endif

        #ifdef USE_GET_ENCODER_ESTIMATES
            void update_encoder_position_and_velocity_estimate(); ///< update the encoder position and velocity estimate of

            float get_encoder_position_estimate() { return encoder_position_estimate; } ///< get the encoder position estimate of the odesc
            float get_encoder_velocity_estimate() { return encoder_velocity_estimate; } ///< get the input velocity estimate of the odes
        #endif

        #ifdef USE_GET_ENCODER_COUNT
            void update_encoder_count();
             
            int32_t get_encoder_shadow_count() { return encoder_shadow_count; } ///< Get Encoder Count
            int32_t get_encoder_count_in_cpr() { return encoder_count_in_cpr; } ///< Get Encoder Count
        #endif

        #ifdef USE_GET_IQ
            void update_current_measured_and_setpoint(); ///< update the current measured and setpoint of the odesc

            float get_current_setpoint() { return current_setpoint; } ///< get the current setpoint of the odesc
            float get_current_measured() { return current_measured; } ///< get the current measured
        #endif

        #ifdef USE_GET_SENSORLESS_ESTIMATES
            void update_sensorless_estimates();

            float get_sensorless_position_estimate() { return sensorless_position_estimate; } ///< Get Sensorless Estimates
            float get_sensorless_velocity_estimate() { return sensorless_velocity_estimate; } ///< Get Sensorless Estimates
        #endif

        #ifdef USE_GET_BUS_VOLTAGE_CURRENT
            void update_bus_voltage_and_current(); ///< update the battery voltage of the odesc

            float get_battery_voltage() { return battery_voltage; } ///< get the battery voltage of the odesc
            float get_bus_current() { return bus_current; } ///< get the bus current of the odesc
        #endif

        #ifdef USE_GET_ADC_VOLTAGE
            void update_adc_voltage();

            float get_adc_voltage() { return adc_voltage; } ///< Get ADC Voltage
        #endif
        
        #ifdef USE_GET_CONTROLLER_ERROR
            void update_controller_error();

            uint32_t get_controller_error() { return controller_error; } ///< Get Controller Error
        #endif

        void estop();
        void set_axis_state(uint32_t state); ///< set the axis state of the odesc
        void set_cotroller_modes(uint32_t control_mode, uint32_t input_mode);
        void set_input_position(float input_position, uint16_t velocity_feedforward, uint16_t torque_feedforward);
        void set_input_velocity(float input_velocity, float torque_feedforward);
        void set_input_torque(float input_torque);
        void set_limits(float velocity_limit, float current_limit);
        void start_anticogging();
        void set_trajectory_velocity_limit(float trajectory_velocity_limit);
        void set_trajectory_accel_limits(float trajectory_accel_limit, float trajectory_decel_limit);
        void set_trajectory_inertia(float trajectory_interia);
        void set_linear_count(int32_t position);
        void set_position_gain(float position_gain);
        void set_vel_gains(float velocity_gain, float velocity_integrator_gain);
        void reboot(); ///< reboot the odesc
        void clear_errors(); ///< clear the errors of the odesc

        void process_rx_frame(
            const RxQueueItem& item
        ) override;

};

#endif