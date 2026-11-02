/**
 * @file odesc_driver.h
 * @brief This is the source file for the odesc class
 * @author Shane Wood
*/

#include "./odesc_driver.h"

odesc::odesc(uint8_t node_ID, twai_node_handle_t handle) : canDriver(node_ID, handle) 
{
    battery_voltage = 0;
    bus_current = 0;
    current_setpoint = 0;
    current_measured = 0;
    encoder_position_estimate = 0;
    encoder_velocity_estimate = 0;
}

float odesc::convert_to_float(const uint8_t *bytes)
{
    float value;
    memcpy(&value, bytes, sizeof(value));
    return value;
}

void odesc::process_rx_frame(const RxQueueItem& item)
{
    const uint32_t expected_id_base =
        static_cast<uint32_t>(get_node_ID()) << node_id_offset;

    if (item.len > 8) {
        return;
    }

    if (item.id == 0) {
        return;
    }
    #ifdef USE_HEARTBEAT
        else if (item.id == (expected_id_base | Heartbeat)) {
            axis_error = ((uint32_t)item.id)        |
                        ((uint32_t)(item.id + 1) << 8)   |
                        ((uint32_t)(item.id + 2) << 16)  |
                        ((uint32_t)(item.id + 3) << 24);

            axis_current_state = ((uint8_t)(item.id + 4) << 24);

            motor_error_flag = ((item.id + 5) & 1);
            encoder_error_flag = ((item.id + 6) & 1);
            controller_error_flag = ((item.id + 7) & 1);
            trajectory_done_flag =  ((item.id + 7) & 0x80) != 0;
        }
    #endif
    #ifdef USE_GET_MOTOR_ERROR
        else if (item.id == (expected_id_base | Get_Motor_Error)) {
            motor_error  = ((uint64_t)item.id)         |
                        ((uint64_t)(item.id + 1) << 8)   |
                        ((uint64_t)(item.id + 2) << 16)  |
                        ((uint64_t)(item.id + 3) << 24)  |
                        ((uint64_t)(item.id + 4) << 32)  |
                        ((uint64_t)(item.id + 5) << 40)  |
                        ((uint64_t)(item.id + 6) << 48)  |
                        ((uint64_t)(item.id + 7) << 56);
        }
    #endif
    #ifdef USE_GET_ENCODER_ERROR
        else if (item.id == (expected_id_base | Get_Encoder_Error)) {
            encoder_error = ((uint32_t)item.id)  |
                        ((uint32_t)(item.id + 1) << 8)   |
                        ((uint32_t)(item.id + 2) << 16)  |
                        ((uint32_t)(item.id + 3) << 24);
        }
    #endif
    #ifdef USE_GET_SENSORLESS_ERROR
        else if (item.id == (expected_id_base | Get_Sensorless_Error)) {
            sensorless_error = ((uint32_t)item.id)  |
                        ((uint32_t)(item.id + 1) << 8)   |
                        ((uint32_t)(item.id + 2) << 16)  |
                        ((uint32_t)(item.id + 3) << 24);
        }
    #endif
    #ifdef USE_GET_ENCODER_ESTIMATES
        else if (item.id == (expected_id_base | Get_Encoder_Estimates)) {
            encoder_position_estimate = convert_to_float(item.data);
            encoder_velocity_estimate = convert_to_float(item.data + 4);
        }
    #endif
    #ifdef USE_GET_ENCODER_COUNT
        else if (item.id == (expected_id_base | Get_Encoder_Count)) {
            encoder_shadow_count = ((int32_t)item.id)     | 
                        ((int32_t)(item.id + 1) << 8)       |
                        ((int32_t)(item.id + 2) << 16)      |
                        ((int32_t)(item.id + 3) << 24);

            encoder_count_in_cpr = ((int32_t)item.id + 4)     | 
                        ((int32_t)(item.id + 5) << 8)                |
                        ((int32_t)(item.id + 6) << 16)                |
                        ((int32_t)(item.id + 7) << 24);
        }
    #endif
    #ifdef USE_GET_IQ
        else if (item.id == (expected_id_base | Get_Iq)) {
            current_setpoint = convert_to_float(item.data);
            current_measured = convert_to_float(item.data + 4);
        }
    #endif
    #ifdef USE_GET_SENSORLESS_ESTIMATES
        else if (item.id == (expected_id_base | Get_Sensorless_Estimates)) {
            sensorless_position_estimate = convert_to_float(item.data);
            sensorless_velocity_estimate = convert_to_float(item.data + 4);
        }
    #endif
    #ifdef USE_GET_BUS_VOLTAGE_CURRENT
        else if (item.id == (expected_id_base | Get_Bus_Voltage_Current)) {
            battery_voltage = convert_to_float(item.data);
            bus_current = convert_to_float(item.data + 4);
        }
    #endif
    #ifdef USE_GET_ADC_VOLTAGE
        else if (item.id == (expected_id_base | Get_ADC_Voltage)) {
            adc_voltage = convert_to_float(item.data);
        }
    #endif
    #ifdef USE_GET_CONTROLLER_ERROR
        else if (item.id == (expected_id_base | Get_Controller_Error)) {
            controller_error =  ((uint32_t)item.id)  |
                        ((uint32_t)(item.id + 1) << 8)   |
                        ((uint32_t)(item.id + 2) << 16)  |
                        ((uint32_t)(item.id + 3) << 24);
        }
    #endif
   
    
    
}

#ifdef USE_HEARTBEAT
    void odesc::update_heartbeat() {
        odesc::send_message(Heartbeat, nullptr, 0, true);
    }
#endif
#ifdef USE_GET_MOTOR_ERROR
    void odesc::update_motor_error() {
        odesc::send_message(Get_Motor_Error, nullptr, 0, true);
    }
#endif
#ifdef USE_GET_ENCODER_ERROR
    void odesc::update_encoder_error() {
        odesc::send_message(Get_Encoder_Error, nullptr, 0, true);
    }
#endif
#ifdef USE_GET_SENSORLESS_ERROR
    void odesc::update_sensorless_error() {
        odesc::send_message(Get_Sensorless_Error, nullptr, 0, true);
    }
#endif
#ifdef USE_GET_ENCODER_ESTIMATES
    void odesc::update_encoder_position_and_velocity_estimate() {
        odesc::send_message(Get_Encoder_Estimates, nullptr, 0, true);
    }
#endif
#ifdef USE_GET_ENCODER_COUNT
    void odesc::update_encoder_count() {
        odesc::send_message(Get_Encoder_Count, nullptr, 0, true);
    }
#endif
#ifdef USE_GET_IQ
    void odesc::update_current_measured_and_setpoint() {
        odesc::send_message(Get_Iq, nullptr, 0, true);
    }
#endif
#ifdef USE_GET_SENSORLESS_ESTIMATES
    void odesc::update_sensorless_estimates() {
        odesc::send_message(Get_Sensorless_Estimates, nullptr, 0, true);
    }
#endif
#ifdef USE_GET_BUS_VOLTAGE_CURRENT
    void odesc::update_bus_voltage_and_current() {
        odesc::send_message(Get_Bus_Voltage_Current, nullptr, 0, true);
    }
#endif
#ifdef USE_GET_ADC_VOLTAGE
    void odesc::update_adc_voltage() {
        odesc::send_message(Get_ADC_Voltage, nullptr, 0, true);
    }
#endif
#ifdef USE_GET_CONTROLLER_ERROR
    void odesc::update_controller_error() {
        odesc::send_message(Get_Controller_Error, nullptr, 0, true);
    }
#endif


void odesc::estop() {
    odesc::send_message(Estop, nullptr, 0, false);
}

void odesc::set_axis_state(uint32_t state){
    uint8_t state_bytes[4];
    (void)memcpy(state_bytes, &state, sizeof(state));

    odesc::send_message(Set_Axis_Requested_State, state_bytes, 4, false);
}

void odesc::set_cotroller_modes(uint32_t control_mode, uint32_t input_mode){
    uint8_t payload_bytes[8];
    (void)memcpy(payload_bytes, &control_mode, sizeof(control_mode));
    (void)memcpy(payload_bytes + sizeof(control_mode), &input_mode, sizeof(input_mode));

    odesc::send_message(Set_Controller_Modes, payload_bytes, 8, false);
}

void odesc::set_input_position(float input_position, uint16_t velocity_feedforward, uint16_t torque_feedforward){
    uint8_t payload_bytes[8];
    (void)memcpy(payload_bytes, &input_position, sizeof(input_position));
    (void)memcpy(payload_bytes + sizeof(input_position), &velocity_feedforward, sizeof(velocity_feedforward));
    (void)memcpy(payload_bytes + sizeof(input_position) + sizeof(velocity_feedforward), &torque_feedforward, sizeof(torque_feedforward));
    odesc::send_message(Set_Input_Pos, payload_bytes, 8, false);
}

void odesc::set_input_velocity(float input_velocity, float torque_feedforward) {

    uint8_t payload_bytes[8];
    (void)memcpy(payload_bytes, &input_velocity, sizeof(input_velocity));
    (void)memcpy(payload_bytes + sizeof(input_velocity), &torque_feedforward, sizeof(torque_feedforward));
    odesc::send_message(Set_Input_Vel, payload_bytes, 8, false);
}

void odesc::set_input_torque(float input_torque) {
    uint8_t payload_bytes[4];
    (void)memcpy(payload_bytes, &input_torque, sizeof(input_torque));
    odesc::send_message(Set_Input_Torque, payload_bytes, 4, false);
}

void odesc::set_limits(float velocity_limit, float current_limit) {
    uint8_t payload_bytes[8];
    (void)memcpy(payload_bytes, &velocity_limit, sizeof(velocity_limit));
    (void)memcpy(payload_bytes + sizeof(velocity_limit), &current_limit, sizeof(current_limit));
    odesc::send_message(Set_Limits, payload_bytes, 8, false);
}

void odesc::start_anticogging() {
    odesc::send_message(Start_Anitcogging, nullptr, 0, false);
}

void odesc::set_trajectory_velocity_limit(float trajectory_velocity_limit) {
    uint8_t payload_bytes[4];
    (void)memcpy(payload_bytes, &trajectory_velocity_limit, sizeof(trajectory_velocity_limit));
    odesc::send_message(Set_Traj_Vel_Limit, payload_bytes, 4, false);
}

void odesc::set_trajectory_accel_limits(float trajectory_accel_limit, float trajectory_decel_limit) {
    uint8_t payload_bytes[8];
    (void)memcpy(payload_bytes, &trajectory_accel_limit, sizeof(trajectory_accel_limit));
    (void)memcpy(payload_bytes + sizeof(trajectory_accel_limit), &trajectory_decel_limit, sizeof(trajectory_decel_limit));
    odesc::send_message(Set_Traj_Accel_Limits, payload_bytes, 8, false);
}

void odesc::set_trajectory_inertia(float trajectory_interia) {
    uint8_t payload_bytes[4];
    (void)memcpy(payload_bytes, &trajectory_interia, sizeof(trajectory_interia));
    odesc::send_message(Set_Traj_Inertia, payload_bytes, 4, false);
}

void odesc::reboot(){
    odesc::send_message(Reboot, nullptr, 0, false);
}

void odesc::clear_errors(){
    odesc::send_message(Clear_Errors, nullptr, 0, false);
}

void odesc::set_linear_count(int32_t position) {
    uint8_t payload_bytes[4];
    (void)memcpy(payload_bytes, &position, sizeof(position));

    odesc::send_message(Set_Linear_Count, payload_bytes, 4, false);
}

void odesc::set_position_gain(float position_gain) {
    uint8_t payload_bytes[4];
    (void)memcpy(payload_bytes, &position_gain, sizeof(position_gain));
    odesc::send_message(Set_Position_Gain, payload_bytes, 4, false);
}

void odesc::set_vel_gains(float velocity_gain, float velocity_integrator_gain) {
    uint8_t payload_bytes[8];
    (void)memcpy(payload_bytes, &velocity_gain, sizeof(velocity_gain));
    (void)memcpy(payload_bytes + sizeof(velocity_gain), &velocity_integrator_gain, sizeof(velocity_integrator_gain));
    odesc::send_message(Set_Vel_Gains, payload_bytes, 8, false);
}