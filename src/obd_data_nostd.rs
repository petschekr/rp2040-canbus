pub trait Process {
    fn process(data: &[u8]) -> Option<Data>;
}

#[derive(Debug, Clone, defmt::Format, Copy, Eq, PartialEq, Default)]
pub enum ChargingType {
    #[default]
    NotCharging,
    AC,
    DC,
    Other,
}
#[derive(Debug, Clone, defmt::Format)]
pub struct Battery01 {
    pub charging: ChargingType,
    pub maximum_charge_voltage: f32,
    pub maximum_charge_current: f32,
    pub maximum_charge_power: f32,

    pub bms_ignition: bool,
    pub bms_relay: bool,
    pub aux_battery_voltage: f32,

    pub bms_soc: f32,
    pub battery_current: f32,
    pub battery_voltage: f32,
    pub battery_power: f32,

    pub fan_status: u8,
    pub fan_speed: u8,

    pub cumulative_energy_charged: f64,
    pub cumulative_energy_discharged: f64,
    pub cumulative_charge_current: f64,
    pub cumulative_discharge_current: f64,

    pub cumulative_operating_time: u32,

    pub dc_battery_inlet_temp: i8,
    pub dc_battery_max_temp: i8,
    pub dc_battery_min_temp: i8,
    pub dc_battery_cell_max_voltage: f32,
    pub dc_battery_cell_max_voltage_id: u8,
    pub dc_battery_cell_min_voltage: f32,
    pub dc_battery_cell_min_voltage_id: u8,

    pub inverter_capacitor_voltage: u16,
    pub isolation_resistance: u16,

    pub rear_drive_motor_speed: i16,
    pub front_drive_motor_speed: i16,
    pub dc_battery_module_temp1: i8,
    pub dc_battery_module_temp2: i8,
    pub dc_battery_module_temp3: i8,
    pub dc_battery_module_temp4: i8,
    pub dc_battery_module_temp5: i8,
}
impl Process for Battery01 {
    fn process(data: &[u8]) -> Option<Data> {
        let mut data = Self {
            // Car Scanner says charging flag is data[9]
            // On my car, data[9] is always 0x00 :(
            charging: if data[9] & 0x20 > 0 {
                ChargingType::AC
            } else if data[9] & 0x40 > 0 {
                ChargingType::DC
            } else if data[9] & 0x80 > 0 {
                ChargingType::Other
            } else {
                ChargingType::NotCharging
            },
            maximum_charge_voltage: u16::from_be_bytes(data[5..7].try_into().unwrap()) as f32
                / 10.0,
            maximum_charge_current: u16::from_be_bytes(data[7..9].try_into().unwrap()) as f32
                / 10.0,
            maximum_charge_power: 0.0,

            bms_ignition: data[50] & 0x04 > 0,
            bms_relay: data[9] & 0x01 > 0,
            bms_soc: data[4] as f32 / 2.0,
            battery_current: i16::from_be_bytes(data[10..12].try_into().unwrap()) as f32 / 10.0,
            battery_voltage: u16::from_be_bytes(data[12..14].try_into().unwrap()) as f32 / 10.0,
            battery_power: 0.0,
            aux_battery_voltage: data[29] as f32 / 10.0,
            fan_status: data[27],
            fan_speed: data[28],
            cumulative_energy_charged: u32::from_be_bytes(data[38..42].try_into().unwrap()) as f64
                / 10.0,
            cumulative_energy_discharged: u32::from_be_bytes(data[42..46].try_into().unwrap())
                as f64
                / 10.0,
            cumulative_charge_current: u32::from_be_bytes(data[30..34].try_into().unwrap()) as f64
                / 10.0,
            cumulative_discharge_current: u32::from_be_bytes(data[34..38].try_into().unwrap())
                as f64
                / 10.0,
            cumulative_operating_time: u32::from_be_bytes(data[46..50].try_into().unwrap()),
            dc_battery_inlet_temp: data[22] as i8 - 40,
            dc_battery_max_temp: data[14] as i8,
            dc_battery_min_temp: data[15] as i8,
            dc_battery_cell_max_voltage: data[23] as f32 / 50.0,
            dc_battery_cell_max_voltage_id: data[24],
            dc_battery_cell_min_voltage: data[25] as f32 / 50.0,
            dc_battery_cell_min_voltage_id: data[26],
            rear_drive_motor_speed: i16::from_be_bytes(data[53..55].try_into().unwrap()),
            front_drive_motor_speed: i16::from_be_bytes(data[55..57].try_into().unwrap()),
            dc_battery_module_temp1: data[16] as i8,
            dc_battery_module_temp2: data[17] as i8,
            dc_battery_module_temp3: data[18] as i8,
            dc_battery_module_temp4: data[19] as i8,
            dc_battery_module_temp5: data[20] as i8,
            inverter_capacitor_voltage: u16::from_be_bytes(data[51..53].try_into().unwrap()),
            isolation_resistance: u16::from_be_bytes(data[57..59].try_into().unwrap()),
        };
        data.battery_power = data.battery_voltage * data.battery_current / 1000.0;
        data.maximum_charge_power = data.maximum_charge_current * data.battery_voltage / 1000.0;
        if data.inverter_capacitor_voltage == 6553 {
            data.inverter_capacitor_voltage = 0;
        }
        if data.aux_battery_voltage == 0.0
            || data.dc_battery_inlet_temp == -40
            || data.dc_battery_inlet_temp == 127 - 40
        {
            return None;
        }
        Some(Data::Battery01(data))
    }
}

#[derive(Debug, Clone, defmt::Format)]
pub struct Battery05 {
    pub available_charge_power: f32,
    pub available_discharge_power: f32,
    pub battery_cell_voltage_deviation: f32,
    pub battery_heater_1_temp: i8,
    pub battery_heater_2_temp: i8,

    pub remaining_energy: f32, // in Wh
    pub soc: f32,
    pub soh: f32,

    pub dc_battery_module_temp6: i8,
    pub dc_battery_module_temp7: i8,
    pub dc_battery_module_temp8: i8,
    pub dc_battery_module_temp9: i8,
    pub dc_battery_module_temp10: i8,
    pub dc_battery_module_temp11: i8,
    pub dc_battery_module_temp12: i8,
    pub dc_battery_module_temp13: i8,
    pub dc_battery_module_temp14: i8,
    pub dc_battery_module_temp15: i8,
    pub dc_battery_module_temp16: i8,
}
impl Process for Battery05 {
    fn process(data: &[u8]) -> Option<Data> {
        let data = Self {
            available_charge_power: u16::from_be_bytes(data[16..18].try_into().unwrap()) as f32
                / 100.0,
            available_discharge_power: u16::from_be_bytes(data[18..20].try_into().unwrap()) as f32
                / 100.0,
            battery_cell_voltage_deviation: data[20] as f32 / 50.0,
            battery_heater_1_temp: data[23] as i8,
            battery_heater_2_temp: data[24] as i8,
            remaining_energy: u16::from_be_bytes(data[28..30].try_into().unwrap()) as f32 * 2.0,
            soc: data[31] as f32 / 2.0,
            soh: u16::from_be_bytes(data[25..27].try_into().unwrap()) as f32 / 10.0,
            dc_battery_module_temp6: data[9] as i8,
            dc_battery_module_temp7: data[10] as i8,
            dc_battery_module_temp8: data[11] as i8,
            dc_battery_module_temp9: data[12] as i8,
            dc_battery_module_temp10: data[13] as i8,
            dc_battery_module_temp11: data[14] as i8,
            dc_battery_module_temp12: data[15] as i8,
            dc_battery_module_temp13: data[39] as i8,
            dc_battery_module_temp14: data[40] as i8,
            dc_battery_module_temp15: data[41] as i8,
            dc_battery_module_temp16: data[42] as i8,
        };
        if data.remaining_energy == 131070.0 || data.remaining_energy == 0.0 {
            return None;
        }
        if data.soc == 0.0 || data.soh == 0.0 {
            return None;
        }
        Some(Data::Battery05(data))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct Battery11 {
    pub ac_charging_events: u32,
    pub dc_charging_events: u32,
    pub cumulative_ac_charging_energy: u32,
    pub cumulative_dc_charging_energy: u32,

    pub dc_battery_module_temp17: i8,
    pub dc_battery_module_temp18: i8,
}
impl Process for Battery11 {
    fn process(data: &[u8]) -> Option<Data> {
        Some(Data::Battery11(Self {
            ac_charging_events: u32::from_be_bytes(data[4..8].try_into().unwrap()),
            dc_charging_events: u32::from_be_bytes(data[8..12].try_into().unwrap()),
            cumulative_ac_charging_energy: u32::from_be_bytes(data[12..16].try_into().unwrap()),
            cumulative_dc_charging_energy: u32::from_be_bytes(data[16..20].try_into().unwrap()),
            dc_battery_module_temp17: data[26] as i8,
            dc_battery_module_temp18: data[27] as i8,
        }))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct TirePressures {
    pub front_left_psi: f32,
    pub front_left_temp: i8,
    pub front_right_psi: f32,
    pub front_right_temp: i8,
    pub rear_left_psi: f32,
    pub rear_left_temp: i8,
    pub rear_right_psi: f32,
    pub rear_right_temp: i8,
}
impl Process for TirePressures {
    fn process(data: &[u8]) -> Option<Data> {
        let data = Self {
            front_left_psi: data[4] as f32 / 5.0,
            front_left_temp: (data[5] as i8) - 55,
            front_right_psi: data[9] as f32 / 5.0,
            front_right_temp: (data[10] as i8) - 55,
            rear_left_psi: data[14] as f32 / 5.0,
            rear_left_temp: (data[15] as i8) - 55,
            rear_right_psi: data[19] as f32 / 5.0,
            rear_right_temp: (data[20] as i8) - 55,
        };
        if data.front_right_psi == 0.0
            || data.front_left_psi == 0.0
            || data.rear_left_psi == 0.0
            || data.rear_right_psi == 0.0
        {
            return None;
        }
        Some(Data::TirePressures(data))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct HVAC {
    pub indoor_temp: f32,
    pub outdoor_temp: f32,
    pub vehicle_speed: u8,
}
impl Process for HVAC {
    fn process(data: &[u8]) -> Option<Data> {
        Some(Data::HVAC(Self {
            indoor_temp: data[5] as f32 / 2.0 - 40.0,
            outdoor_temp: data[6] as f32 / 2.0 - 40.0,
            vehicle_speed: data[29],
        }))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct ICCU01 {
    pub ac_maximum_current_limit: f32,
    pub dc_maximum_current_limit: f32,
    pub dc_target_voltage: f32,
    pub v2l_ac_target_voltage: f32,
    pub v2l_ac_current_limit: f32,
    pub v2l_dc_current_limit: f32,
}
impl Process for ICCU01 {
    fn process(data: &[u8]) -> Option<Data> {
        Some(Data::ICCU01(Self {
            ac_maximum_current_limit: u16::from_be_bytes(data[12..14].try_into().unwrap()) as f32
                / 10.0,
            dc_maximum_current_limit: u16::from_be_bytes(data[10..12].try_into().unwrap()) as f32
                / 10.0,
            dc_target_voltage: u16::from_be_bytes(data[8..10].try_into().unwrap()) as f32 / 10.0,
            v2l_ac_target_voltage: u16::from_be_bytes(data[14..16].try_into().unwrap()) as f32
                / 10.0,
            v2l_ac_current_limit: data[17] as f32,
            v2l_dc_current_limit: data[16] as f32,
        }))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct ICCU02 {
    pub obc_ac_current_a: f32,
    pub obc_ac_current_b: f32,
    pub obc_ac_frequency: f32,
    pub obc_ac_total_current: f32,
    pub obc_ac_voltage_a: f32,
    pub obc_ac_voltage_b: f32,
    pub obc_ac_power: f32,

    pub obc_dc_current_a: f32,
    pub obc_dc_current_b: f32,
    pub obc_dc_total_current: f32,
    pub obc_dc_voltage: f32,
    pub obc_dc_power: f32,
    pub obc_charging_loss: f32,

    pub obc_temp_a: i8,
    pub obc_temp_b: i8,
}
impl Process for ICCU02 {
    fn process(data: &[u8]) -> Option<Data> {
        let mut data = Self {
            obc_ac_current_a: i16::from_be_bytes(data[13..15].try_into().unwrap()) as f32 / 100.0,
            obc_ac_current_b: i16::from_be_bytes(data[15..17].try_into().unwrap()) as f32 / 100.0,
            obc_ac_frequency: data[10] as f32,
            obc_ac_total_current: i16::from_be_bytes(data[11..13].try_into().unwrap()) as f32
                / 100.0,
            obc_ac_voltage_a: u16::from_be_bytes(data[4..6].try_into().unwrap()) as f32 / 10.0,
            obc_ac_voltage_b: u16::from_be_bytes(data[6..8].try_into().unwrap()) as f32 / 10.0,
            obc_ac_power: 0.0,

            obc_dc_current_a: i16::from_be_bytes(data[23..25].try_into().unwrap()) as f32 / 100.0,
            obc_dc_current_b: i16::from_be_bytes(data[25..27].try_into().unwrap()) as f32 / 100.0,
            obc_dc_total_current: i16::from_be_bytes(data[21..23].try_into().unwrap()) as f32
                / 100.0,
            obc_dc_voltage: u16::from_be_bytes(data[19..21].try_into().unwrap()) as f32 / 10.0,
            obc_dc_power: 0.0,
            obc_charging_loss: 0.0,

            obc_temp_a: data[27] as i8 - 40,
            obc_temp_b: data[28] as i8 - 40,
        };
        data.obc_ac_power = data.obc_ac_total_current * data.obc_ac_voltage_a / 1000.0;
        data.obc_dc_power = data.obc_dc_total_current * data.obc_dc_voltage / 1000.0;
        data.obc_charging_loss = data.obc_ac_power - data.obc_dc_power;
        Some(Data::ICCU02(data))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct ICCU03 {
    pub obc_dc_target_current: f32,
    pub obc_dc_target_voltage: f32,
}
impl Process for ICCU03 {
    fn process(data: &[u8]) -> Option<Data> {
        Some(Data::ICCU03(Self {
            obc_dc_target_current: u16::from_be_bytes(data[10..12].try_into().unwrap()) as f32
                / 10.0,
            obc_dc_target_voltage: u16::from_be_bytes(data[8..10].try_into().unwrap()) as f32
                / 10.0,
        }))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct ICCU11 {
    pub aux_battery_current: f32,
    pub aux_battery_soc: u8,
    pub aux_battery_temp: f32,
    pub aux_battery_voltage_iccu: f32,
    pub ldc_input_voltage: f32,
    pub ldc_output_current: f32,
    pub ldc_output_voltage: f32,
    pub ldc_output_power: f32,
    pub ldc_temp: i8,
}
impl Process for ICCU11 {
    fn process(data: &[u8]) -> Option<Data> {
        let mut data = Self {
            aux_battery_current: u16::from_be_bytes(data[18..20].try_into().unwrap()) as f32
                / 100.0
                - 327.0,
            aux_battery_soc: data[20],
            aux_battery_temp: u16::from_be_bytes(data[23..25].try_into().unwrap()) as f32 / 2.0
                - 40.0,
            aux_battery_voltage_iccu: u16::from_be_bytes(data[21..23].try_into().unwrap()) as f32
                / 1000.0
                + 6.0,
            ldc_input_voltage: u16::from_be_bytes(data[14..16].try_into().unwrap()) as f32 / 10.0,
            ldc_output_current: u16::from_be_bytes(data[12..14].try_into().unwrap()) as f32 / 100.0,
            ldc_output_voltage: u16::from_be_bytes(data[10..12].try_into().unwrap()) as f32
                / 1000.0,
            ldc_output_power: 0.0,
            ldc_temp: data[9] as i8 - 100,
        };
        data.ldc_output_power = data.ldc_output_current * data.ldc_output_voltage;
        Some(Data::ICCU11(data))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct VCMS01 {
    pub ac_charging_state: bool,       // [17] & bit 2
    pub dc_charging_state: bool,       // [17] & bit 3
    pub aux_battery_voltage_vcms: f32, // [11..13] / 100.0
    pub estimated_charging_power: f32, // [28..30] * 10.0 - 50.0
    pub v2l_discharging_current: f32,  // [27]
}
impl Process for VCMS01 {
    fn process(data: &[u8]) -> Option<Data> {
        let data = Self {
            ac_charging_state: data[17] & (1 << 2) != 0,
            dc_charging_state: data[17] & (1 << 3) != 0,
            aux_battery_voltage_vcms: u16::from_be_bytes(data[11..13].try_into().unwrap()) as f32
                / 100.0,
            estimated_charging_power: u16::from_be_bytes(data[28..30].try_into().unwrap()) as f32
                * 10.0
                - 50.0,
            v2l_discharging_current: data[27] as f32,
        };
        Some(Data::VCMS01(data))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct VCMS02 {
    pub evse_delivered_power: f32, // [23..25] * 10.0
    pub evse_main_power: f32,      // [27]
    pub evse_max_voltage: f32,     // [9..11] / 10.0
    pub evse_max_current: f32,     // [11..13] / 10.0
    pub evse_min_voltage: f32,     // [13..15] / 10.0
    pub evse_min_current: f32,     // [15..17] / 10.0
    pub evse_output_voltage: f32,  // [17..19] / 10.0
    pub evse_output_current: f32,  // [19..21] / 10.0
    pub evse_max_power: f32,       // [25..27] * 10.0
}
impl Process for VCMS02 {
    fn process(data: &[u8]) -> Option<Data> {
        let data = Self {
            evse_delivered_power: u16::from_be_bytes(data[23..25].try_into().unwrap()) as f32
                * 10.0,
            evse_main_power: data[27] as f32,
            evse_max_voltage: u16::from_be_bytes(data[9..11].try_into().unwrap()) as f32 / 10.0,
            evse_max_current: u16::from_be_bytes(data[11..13].try_into().unwrap()) as f32 / 10.0,
            evse_min_voltage: u16::from_be_bytes(data[13..15].try_into().unwrap()) as f32 / 10.0,
            evse_min_current: u16::from_be_bytes(data[15..17].try_into().unwrap()) as f32 / 10.0,
            evse_output_voltage: u16::from_be_bytes(data[17..19].try_into().unwrap()) as f32 / 10.0,
            evse_output_current: u16::from_be_bytes(data[19..21].try_into().unwrap()) as f32 / 10.0,
            evse_max_power: u16::from_be_bytes(data[25..27].try_into().unwrap()) as f32 * 10.0,
        };
        Some(Data::VCMS02(data))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct VCMS03 {
    pub ac_charger_current: f32,            // [38..40] / 100.0
    pub ac_charging_counter: u16,           // [30..32]
    pub ac_charging_time: u16,              // [34..36]
    pub ac_charging_time_after_plugin: u16, // [42..44]
    pub bms_current_target: f32,            // [26..28] / 10.0
    pub bms_voltage_target: f32,            // [24..26] / 10.0
    pub cp_duty_cycle: f32,                 // [7..9] / 10.0
    pub cp_voltage: f32,                    // [6] / 10.0
    pub plugged_in: bool,
    pub dc_charging_time: u16,              // [36..38]
    pub dc_charging_time_after_plugin: u16, // [44..46]
    pub dc_max_current: f32,                // [40..42] / 100.0
    pub dc_charging_counter: u16,           // [32..34]
    pub ev_max_current: f32,                // [22..24] / 10.0
    pub ev_max_voltage: f32,                // [20..22] / 10.0
    pub evse_target_voltage: f32,           // [16..18] / 10.0
    pub evse_target_current: f32,           // [18..20] / 10.0
    pub hv_battery_soc: f32,                // [15] / 2.0
    pub hv_battery_voltage: f32,            // [13..15] / 10.0
}
impl Process for VCMS03 {
    fn process(data: &[u8]) -> Option<Data> {
        let mut data = Self {
            ac_charger_current: u16::from_be_bytes(data[38..40].try_into().unwrap()) as f32 / 10.0,
            ac_charging_counter: u16::from_be_bytes(data[30..32].try_into().unwrap()),
            ac_charging_time: u16::from_be_bytes(data[34..36].try_into().unwrap()),
            ac_charging_time_after_plugin: u16::from_be_bytes(data[42..44].try_into().unwrap()),
            bms_current_target: u16::from_be_bytes(data[26..28].try_into().unwrap()) as f32 / 10.0,
            bms_voltage_target: u16::from_be_bytes(data[24..26].try_into().unwrap()) as f32 / 10.0,
            cp_duty_cycle: u16::from_be_bytes(data[7..9].try_into().unwrap()) as f32 / 10.0,
            cp_voltage: data[6] as f32 / 10.0,
            plugged_in: false,
            dc_charging_time: u16::from_be_bytes(data[36..38].try_into().unwrap()),
            dc_charging_time_after_plugin: u16::from_be_bytes(data[44..46].try_into().unwrap()),
            dc_max_current: u16::from_be_bytes(data[40..42].try_into().unwrap()) as f32 / 100.0,
            dc_charging_counter: u16::from_be_bytes(data[32..34].try_into().unwrap()),
            ev_max_current: u16::from_be_bytes(data[22..24].try_into().unwrap()) as f32 / 10.0,
            ev_max_voltage: u16::from_be_bytes(data[20..22].try_into().unwrap()) as f32 / 10.0,
            evse_target_voltage: u16::from_be_bytes(data[16..18].try_into().unwrap()) as f32 / 10.0,
            evse_target_current: u16::from_be_bytes(data[18..20].try_into().unwrap()) as f32 / 10.0,
            hv_battery_soc: data[15] as f32 / 2.0,
            hv_battery_voltage: u16::from_be_bytes(data[13..15].try_into().unwrap()) as f32 / 10.0,
        };
        if data.cp_voltage > 1.0 {
            data.plugged_in = true;
        }
        Some(Data::VCMS03(data))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct VCMS04 {
    pub ac_inlet_1_temperature: i8,      // [10] - 50
    pub main_battery_relay_status: bool, // [4] & bit 2
    pub charging_socket_connected: bool, // [4] & bit 1
    pub dc_inlet_1_temperature: i8,      // [8] - 50,
    pub dc_inlet_2_temperature: i8,      // [9] - 50,
}
impl Process for VCMS04 {
    fn process(data: &[u8]) -> Option<Data> {
        let data = Self {
            ac_inlet_1_temperature: data[10] as i8 - 50,
            main_battery_relay_status: data[4] & (1 << 2) != 0,
            charging_socket_connected: data[4] & (1 << 1) != 0,
            dc_inlet_1_temperature: data[8] as i8 - 50,
            dc_inlet_2_temperature: data[9] as i8 - 50,
        };
        Some(Data::VCMS04(data))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct Dashboard {
    pub odometer: u32,
}
impl Process for Dashboard {
    fn process(data: &[u8]) -> Option<Data> {
        Some(Data::Dashboard(Self {
            odometer: ((data[9] as u32) << 16) + ((data[10] as u32) << 8) + (data[11] as u32),
        }))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct IGPM03 {
    pub ignition_on: bool,
    pub driver_seat_belt: bool,
    pub passenger_seat_belt: bool,
    pub hood_open: bool,
    pub rear_right_door_open: bool,
    pub rear_right_door_unlocked: bool,
    pub rear_left_door_open: bool,
    pub rear_left_door_unlocked: bool,
    pub driver_door_open: bool,
    pub passenger_door_open: bool,
    pub trunk_open: bool,
}
impl Process for IGPM03 {
    fn process(data: &[u8]) -> Option<Data> {
        Some(Data::IGPM03(Self {
            ignition_on: data[5] & 0x60 > 0,
            driver_seat_belt: data[5] & (1 << 1) > 0,
            passenger_seat_belt: data[5] & (1 << 2) > 0,
            hood_open: data[5] & (1 << 0) > 0,
            rear_left_door_open: data[4] & (1 << 0) > 0,
            rear_left_door_unlocked: data[4] & (1 << 1) > 0,
            rear_right_door_open: data[4] & (1 << 2) > 0,
            rear_right_door_unlocked: data[4] & (1 << 3) > 0,
            passenger_door_open: data[4] & (1 << 4) > 0,
            driver_door_open: data[4] & (1 << 5) > 0,
            trunk_open: data[4] & (1 << 7) > 0,
        }))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct IGPM04 {
    pub driver_door_unlocked: bool,
    pub passenger_door_unlocked: bool,
    pub rear_left_seat_belt: bool,
    pub rear_center_seat_belt: bool,
    pub rear_right_seat_belt: bool,
}
impl Process for IGPM04 {
    fn process(data: &[u8]) -> Option<Data> {
        Some(Data::IGPM04(Self {
            driver_door_unlocked: data[4] & (1 << 3) > 0,
            passenger_door_unlocked: data[4] & (1 << 2) > 0,
            rear_left_seat_belt: data[6] & (1 << 2) > 0,
            rear_center_seat_belt: data[6] & (1 << 3) > 0,
            rear_right_seat_belt: data[6] & (1 << 4) > 0,
        }))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct CabinEnvironment {
    pub pressure: f32,
    pub temperature: f32,
    pub humidity: f32,
}
impl Process for CabinEnvironment {
    fn process(data: &[u8]) -> Option<Data> {
        let data = Self {
            pressure: f32::from_be_bytes(data[0..4].try_into().unwrap()),
            temperature: f32::from_be_bytes(data[4..8].try_into().unwrap()),
            humidity: f32::from_be_bytes(data[8..12].try_into().unwrap()),
        };
        if data.pressure == 0.0 {
            return None;
        }
        Some(Data::CabinEnvironment(data))
    }
}
#[derive(Debug, Clone, defmt::Format, Eq, PartialEq)]
pub enum Gear {
    Park,
    Reverse,
    Neutral,
    Drive,
}
#[derive(Debug, Clone, defmt::Format)]
pub struct Shifter {
    pub gear: Gear,
}
impl Process for Shifter {
    fn process(data: &[u8]) -> Option<Data> {
        let gear = match data[8] {
            1 => Gear::Park,
            2 => Gear::Reverse,
            3 => Gear::Neutral,
            4 => Gear::Drive,
            _ => return None,
        };
        Some(Data::Shifter(Self { gear }))
    }
}
#[derive(Debug, Clone, defmt::Format)]
pub struct Panda {
    pub panda_aux_battery_voltage: f32,
    pub panda_aux_battery_current: f32,
    pub panda_fan_speed: u16,
}
#[derive(Debug, Clone, defmt::Format)]
pub struct Location {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub speed: f32,
    pub bearing: f32,
    pub unix_timestamp_seconds: i64,
    pub vertical_accuracy: f32,
    pub bearing_accuracy: f32,
    pub speed_accuracy: f32,
    pub has_fix: bool,
}
#[derive(Debug, Clone, defmt::Format)]
pub struct SeaLevelPressure {
    pub sea_level_pressure: f32,
}
#[derive(Debug, Clone, defmt::Format)]
pub struct EnergyUse {
    pub energy_since_ignition: f32,
    pub energy_since_charging: f32,
}

#[derive(Debug, Clone, defmt::Format)]
pub enum Data {
    Battery01(Battery01),
    Battery05(Battery05),
    Battery11(Battery11),
    TirePressures(TirePressures),
    HVAC(HVAC),
    ICCU01(ICCU01),
    ICCU02(ICCU02),
    ICCU03(ICCU03),
    ICCU11(ICCU11),
    VCMS01(VCMS01),
    VCMS02(VCMS02),
    VCMS03(VCMS03),
    VCMS04(VCMS04),
    Dashboard(Dashboard),
    IGPM03(IGPM03),
    IGPM04(IGPM04),
    CabinEnvironment(CabinEnvironment),
    Shifter(Shifter),
    Panda(Panda),
    Location(Location),
    SeaLevelPressure(SeaLevelPressure),
    EnergyUse(EnergyUse),
}
