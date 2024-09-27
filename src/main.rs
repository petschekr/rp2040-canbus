#![no_std]
#![no_main]

use bme280::i2c::BME280;
use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c;
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{self, Spi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_time::{Delay, Timer};
use embedded_can::{ExtendedId, Id, StandardId};
use heapless::Vec;
use mcp25xxfd::frame::Frame;
use mcp25xxfd::{config::{BitRate, Clock, Config, FIFOConfig, FilterConfig, MaskConfig}, registers, MCP25xxFD};
use mcp25xxfd::registers::PayloadSize;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

type SPI0Type<BUS> = Spi<'static, BUS, spi::Async>;
static SPI_BUS0: StaticCell<Mutex<CriticalSectionRawMutex, SPI0Type<SPI0>>> = StaticCell::new();

static FORWARDING_CHANNEL: Channel<CriticalSectionRawMutex, obd_data::Data, 10> = Channel::new();

static OBD_CONTROLLER: StaticCell<Mutex<CriticalSectionRawMutex, MCP25xxFD<SpiDevice<CriticalSectionRawMutex, SPI0Type<SPI0>, Output>>>> = StaticCell::new();

fn construct_uds_query(command: &[u8]) -> [u8; 8] {
    let mut query = [0u8; 8];
    if command.len() <= 6 {
        query[0] = command.len() as u8 + 1; // Length of UDS command byte + ECU command
        query[1] = 0x22; // UDS command = diagnostic read
        // Copy over the ECU subcommand
        for (i, byte) in command.iter().enumerate() {
            query[i + 2] = *byte;
        }
    }
    query
}
struct ECUAddresses {
    bms: Id,
    tpms: Id,
}
impl ECUAddresses {
    fn new() -> (Self, Self) {
        let tx = Self {
            bms: StandardId::new(0x7E4).unwrap().into(),
            tpms: StandardId::new(0x7A0).unwrap().into(),
        };
        let rx = Self {
            bms: Self::rx_address(tx.bms),
            tpms: Self::rx_address(tx.tpms),
        };
        (tx, rx)
    }
    fn rx_address(ecu_addr: impl Into<Id>) -> Id {
        let ecu_addr = ecu_addr.into();
        match ecu_addr {
            Id::Standard(addr) => StandardId::new(addr.as_raw() + 8).unwrap().into(),
            Id::Extended(addr) => ExtendedId::new(addr.as_raw() + 8).unwrap().into(),
        }
    }
}

mod obd_data {
    use defmt::Format;
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Format)]
    pub enum ChargingType {
        NotCharging,
        AC,
        DC,
        Other,
    }
    #[derive(Serialize, Deserialize, Format)]
    pub struct BatteryData1 {
        pub charging: ChargingType,
        pub bms_ignition: bool,
        pub bms_relay: bool,
        pub aux_battery_voltage: u8,

        pub bms_soc: u8,
        pub battery_current: i16,
        pub battery_voltage: u16,

        pub fan_speed: u8,
        pub fan_feedback: u8,

        pub cumulative_energy_charged: u32,
        pub cumulative_energy_discharged: u32,
        pub cumulative_charge_current: u32,
        pub cumulative_discharge_current: u32,

        pub cumulative_operating_time: u32,

        pub available_charge_power: u16,
        pub available_discharge_power: u16,

        pub dc_battery_inlet_temp: i8,
        pub dc_battery_max_temp: i8,
        pub dc_battery_min_temp: i8,
        pub dc_battery_cell_max_voltage: u8,
        pub dc_battery_cell_max_voltage_id: u8,
        pub dc_battery_cell_min_voltage: u8,
        pub dc_battery_cell_min_voltage_id: u8,

        pub rear_drive_motor_speed: i16,
        pub front_drive_motor_speed: i16,
        pub dc_battery_module_temp1: i8,
        pub dc_battery_module_temp2: i8,
        pub dc_battery_module_temp3: i8,
        pub dc_battery_module_temp4: i8,
        pub dc_battery_module_temp5: i8,
    }
    #[derive(Serialize, Deserialize, Format)]
    pub struct TirePressures {
        pub front_left_psi: u8,
        pub front_left_temp: u8, // Subtract 55 degrees C
        pub front_right_psi: u8,
        pub front_right_temp: u8,
        pub rear_left_psi: u8,
        pub rear_left_temp: u8,
        pub rear_right_psi: u8,
        pub rear_right_temp: u8,
    }

    #[derive(Serialize, Deserialize, Format)]
    pub enum Data {
        BatteryData1(BatteryData1),
        TirePressures(TirePressures),
        CabinEnvironmentData(()), // TODO
        Error(&'static str),
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Hello World!");

    let miso = p.PIN_20;
    let mosi = p.PIN_19;
    let sclk = p.PIN_18;
    let spi0 = Spi::new(
        p.SPI0,
        sclk,
        mosi,
        miso,
        p.DMA_CH0,
        p.DMA_CH1,
        spi::Config::default(),
    );
    let spi0 = SPI_BUS0.init(Mutex::new(spi0));

    let obd_cs = Output::new(p.PIN_21, Level::High);
    let obd_int = Input::new(p.PIN_14, Pull::Up);
    let mut obd_stby = Output::new(p.PIN_24, Level::Low);
    obd_stby.set_low();

    let comma_cs = Output::new(p.PIN_22, Level::High);
    let comma_int = Input::new(p.PIN_15, Pull::Up);
    let mut comma_stby = Output::new(p.PIN_25, Level::Low);
    comma_stby.set_low();

    let i2c = i2c::I2c::new_blocking(p.I2C0, p.PIN_1, p.PIN_0, i2c::Config::default());
    let mut bme280 = BME280::new_primary(i2c);
    bme280.init(&mut Delay).unwrap();

    spawner.must_spawn(obd_task(spawner, spi0, obd_cs, obd_int));
    spawner.must_spawn(comma_task(spi0, comma_cs, comma_int));
}

const TRANSMIT_FIFO: u8 = 1;
const RX_BATTERY_FIFO: u8 = 2;
const RX_TPMS_FIFO: u8 = 3;

#[embassy_executor::task]
async fn obd_task(spawner: Spawner, spi_bus: &'static Mutex<CriticalSectionRawMutex, SPI0Type<SPI0>>, cs: Output<'static>, mut int: Input<'static>) {

    let (tx_addrs, rx_addrs) = ECUAddresses::new();

    let obd_device = SpiDevice::new(spi_bus, cs);
    let obd_controller = OBD_CONTROLLER.init(Mutex::new(MCP25xxFD::new(obd_device)));

    {
        let mut obd_controller = obd_controller.lock().await;
        obd_controller.reset_and_apply_config(&Config {
            clock: Clock::Clock20MHz,
            bit_rate: BitRate::default(),
            ecc_enabled: true,
            restrict_retx_attempts: false,
            txq_enabled: false,
            tx_event_fifo_enabled: false,
            iso_crc_enabled: true,
        }).await.unwrap();

        obd_controller.configure_fifo(
            FIFOConfig::<TRANSMIT_FIFO>::tx_with_size(8, PayloadSize::Bytes8)
        ).await.unwrap();

        obd_controller.configure_fifo(
            FIFOConfig::<RX_BATTERY_FIFO>::rx_with_size(16, PayloadSize::Bytes8)
        ).await.unwrap();
        obd_controller.configure_filter(
            FilterConfig::<RX_BATTERY_FIFO, RX_BATTERY_FIFO>::from_id(rx_addrs.bms),
            MaskConfig::<RX_BATTERY_FIFO>::match_exact(),
        ).await.unwrap();

        obd_controller.configure_fifo(
            FIFOConfig::<RX_TPMS_FIFO>::rx_with_size(16, PayloadSize::Bytes8)
        ).await.unwrap();
        obd_controller.configure_filter(
            FilterConfig::<RX_TPMS_FIFO, RX_TPMS_FIFO>::from_id(rx_addrs.tpms),
            MaskConfig::<RX_TPMS_FIFO>::match_exact(),
        ).await.unwrap();

        obd_controller.set_mode(registers::OperationMode::Normal).await.unwrap();
        Timer::after_millis(500).await;
    }
    spawner.must_spawn(obd_sender_task(obd_controller, tx_addrs));

    // Receive loop
    loop {
        // Wait for interrupt pin to go low (aka active) before calling receive so we don't spinlock
        int.wait_for_low().await;
        // Receive query responses
        let mut iso_tp_data: Vec<u8, 64> = Vec::new();
        let mut iso_tp_length: Option<u16> = None;
        let mut current_fifo: Option<u8> = None;

        let (id, pid, data) = loop {
            let rx_result = {
                obd_controller.lock().await.receive(current_fifo).await
            };
            match rx_result {
                Ok((fifo, frame)) => {
                    current_fifo = Some(fifo);
                    debug!("Received message from FIFO{}: {:x} ({} bytes): {:x}", fifo, frame.raw_id(), frame.data().len(), frame.data());

                    match frame.data()[0] >> 4 {
                        0 => {
                            // Single ISO-TP frame
                            debug!("Single frame of data");
                            // ISO-TP transmission complete
                            iso_tp_data.extend_from_slice(&frame.data()[1..]).unwrap();
                            break (frame, &iso_tp_data[1..3], &iso_tp_data[3..]); // Strip 0x62 (UDS response) + PID (2 bytes) from data
                        },
                        1 => {
                            // First ISO-TP frame
                            let length = frame.data()[1] as u16 + ((frame.data()[0] as u16 & 0b1111) << 8);
                            debug!("First frame of data with total length {}", length);
                            iso_tp_length = Some(length);
                            iso_tp_data.clear();
                            iso_tp_data.extend_from_slice(&frame.data()[2..]).unwrap();

                            // Send flow control message
                            let tx_id: Id = match frame.id() {
                                Id::Standard(id) => StandardId::new(id.as_raw() - 8).unwrap().into(),
                                Id::Extended(id) => ExtendedId::new(id.as_raw() - 8).unwrap().into(),
                            };
                            let flow_control_frame = Frame::new(tx_id, &[0x30, 0x00, 10, 0x00, 0x00, 0x00, 0x00, 0x00]).unwrap();
                            obd_controller
                                .lock().await
                                .transmit::<TRANSMIT_FIFO>(&flow_control_frame).await
                                .unwrap();
                        },
                        2 => {
                            // Consecutive ISO-TP frame
                            let frame_number = frame.data()[0] & 0b1111;
                            debug!("Consecutive frame #{}", frame_number);
                            iso_tp_data.extend_from_slice(&frame.data()[1..]).unwrap();

                            if iso_tp_data.len() as u16 >= iso_tp_length.unwrap_or(u16::MAX) {
                                // ISO-TP transmission complete
                                break (frame, &iso_tp_data[1..3], &iso_tp_data[3..]); // Strip 0x62 (UDS response) + PID (2 bytes) from data
                            }
                        },
                        _ => {},
                    }
                },
                Err(mcp25xxfd::Error::ControllerError(description)) => {
                    FORWARDING_CHANNEL.send(obd_data::Data::Error(description)).await;
                },
                err => { err.unwrap(); },
            }
        };

        let data = match id {
            frame if frame.id() == rx_addrs.bms => Some(process_battery_data(data)),
            frame if frame.id() == rx_addrs.tpms => Some(process_tpms_data(data)),
            frame => {
                warn!("Unhandled ISO-TP response from address {:x} to PID {:x}: {:x}", frame.raw_id(), pid, data);
                None
            },
        };
        if let Some(data) = data {
            FORWARDING_CHANNEL.send(data).await;
        }
    }

    fn process_battery_data(data: &[u8]) -> obd_data::Data {
        obd_data::Data::BatteryData1(obd_data::BatteryData1 {
            charging:
                if data[9] & 0x20 > 0 { obd_data::ChargingType::AC }
                else if data[9] & 0x40 > 0 { obd_data::ChargingType::DC }
                else if data[9] & 0x80 > 0 { obd_data::ChargingType::Other }
                else { obd_data::ChargingType::NotCharging },
            bms_ignition: data[50] & 0x04 > 0,
            bms_relay: data[9] & 0x01 > 0,
            bms_soc: data[4],
            battery_current: i16::from_be_bytes(data[10..12].try_into().unwrap()),
            battery_voltage: u16::from_be_bytes(data[12..14].try_into().unwrap()),
            aux_battery_voltage: data[29],
            fan_speed: data[27],
            fan_feedback: data[28],
            cumulative_energy_charged: u32::from_be_bytes(data[38..42].try_into().unwrap()),
            cumulative_energy_discharged: u32::from_be_bytes(data[42..46].try_into().unwrap()),
            cumulative_charge_current: u32::from_be_bytes(data[30..34].try_into().unwrap()),
            cumulative_discharge_current: u32::from_be_bytes(data[34..38].try_into().unwrap()),
            cumulative_operating_time: u32::from_be_bytes(data[46..50].try_into().unwrap()),
            available_discharge_power: u16::from_be_bytes(data[5..7].try_into().unwrap()),
            available_charge_power: u16::from_be_bytes(data[7..9].try_into().unwrap()),
            dc_battery_inlet_temp: data[22] as i8,
            dc_battery_max_temp: data[14] as i8,
            dc_battery_min_temp: data[15] as i8,
            dc_battery_cell_max_voltage: data[23],
            dc_battery_cell_max_voltage_id: data[24],
            dc_battery_cell_min_voltage: data[25],
            dc_battery_cell_min_voltage_id: data[26],
            rear_drive_motor_speed: i16::from_be_bytes(data[53..55].try_into().unwrap()),
            front_drive_motor_speed: i16::from_be_bytes(data[55..57].try_into().unwrap()),
            dc_battery_module_temp1: data[16] as i8,
            dc_battery_module_temp2: data[17] as i8,
            dc_battery_module_temp3: data[18] as i8,
            dc_battery_module_temp4: data[19] as i8,
            dc_battery_module_temp5: data[20].try_into().unwrap(), // TODO: same as inlet temp???
        })
    }

    fn process_tpms_data(data: &[u8]) -> obd_data::Data {
        obd_data::Data::TirePressures(obd_data::TirePressures {
            front_left_psi: data[4],
            front_left_temp: data[5],
            front_right_psi: data[9],
            front_right_temp: data[10],
            rear_left_psi: data[14],
            rear_left_temp: data[15],
            rear_right_psi: data[19],
            rear_right_temp: data[20],
        })
    }
}

#[embassy_executor::task]
async fn obd_sender_task(obd_controller: &'static Mutex<CriticalSectionRawMutex, MCP25xxFD<SpiDevice<'_, CriticalSectionRawMutex, SPI0Type<SPI0>, Output<'_>>>>, tx_addrs: ECUAddresses) {
    let queries = [
        Frame::new(tx_addrs.bms, &construct_uds_query(&[0x01, 0x01])).unwrap(),
        // Frame::new(tx_addrs.tpms, &construct_uds_query(&[0xC0, 0x02])).unwrap(),
        Frame::new(tx_addrs.tpms, &construct_uds_query(&[0xC0, 0x0B])).unwrap(),
    ];

    loop {
        // Send all queries once per second
        for frame in queries.iter() {
            obd_controller
                .lock().await
                .transmit::<TRANSMIT_FIFO>(frame).await
                .unwrap();
        }

        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
async fn comma_task(spi_bus: &'static Mutex<CriticalSectionRawMutex, SPI0Type<SPI0>>, cs: Output<'static>, _int: Input<'static>) {
    let comma_device = SpiDevice::new(spi_bus, cs);
    let mut comma_controller = MCP25xxFD::new(comma_device);

    comma_controller.reset_and_apply_config(&Config {
        clock: Clock::Clock20MHz,
        bit_rate: BitRate::default(),
        ecc_enabled: true,
        restrict_retx_attempts: false,
        txq_enabled: false,
        tx_event_fifo_enabled: false,
        iso_crc_enabled: true,
    }).await.unwrap();

    comma_controller.configure_fifo(
        FIFOConfig::<TRANSMIT_FIFO>::tx_with_size(8, PayloadSize::Bytes64)
    ).await.unwrap();

    comma_controller.set_mode(registers::OperationMode::Normal).await.unwrap();
    Timer::after_millis(500).await;

    loop {
        let forward_data = FORWARDING_CHANNEL.receive().await;

        let data: Vec<u8, 64> = postcard::to_vec(&forward_data).unwrap();
        debug!("{:?}", &forward_data);
        debug!("Serialized length is: {}", data.len());

        let forward_frame = Frame::new(StandardId::new(0x715).unwrap(), data.as_slice()).unwrap();
        comma_controller.transmit::<TRANSMIT_FIFO>(&forward_frame).await.unwrap();
    }
}