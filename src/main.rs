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
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Timer};
use embedded_can::{ExtendedId, Id, StandardId};
use heapless::Vec;
use mcp25xxfd::frame::Frame;
use mcp25xxfd::{config::{BitRate, Clock, Config, FIFOConfig, FilterConfig, MaskConfig}, registers, MCP25xxFD};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

type SPI0Type<BUS> = Spi<'static, BUS, spi::Async>;
static SPI_BUS0: StaticCell<Mutex<CriticalSectionRawMutex, SPI0Type<SPI0>>> = StaticCell::new();

static BATTERY_SIGNAL: Signal<CriticalSectionRawMutex, BatteryData> = Signal::new();

const BATTERY_ADDRESS: u16 = 0x7E4;
const BATTERY_QUERY: [u8; 8] = [0x03, 0x22, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00];
struct BatteryData {
    bms_soc: f32,
    bms_relay: bool,
    battery_current: f32,
    battery_voltage: f32,
    aux_battery_voltage: f32,
    available_power: u16,
    charge_current_request: f32,
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

    spawner.must_spawn(obd_task(spi0, obd_cs, obd_int));
    spawner.must_spawn(comma_task(spi0, comma_cs, comma_int));
}

#[embassy_executor::task]
async fn obd_task(spi_bus: &'static Mutex<CriticalSectionRawMutex, SPI0Type<SPI0>>, cs: Output<'static>, int: Input<'static>) {
    const TRANSMIT_FIFO: u8 = 1;
    const RECEIVE_FIFO: u8 = 2;

    let obd_device = SpiDevice::new(spi_bus, cs);
    let mut obd_controller = MCP25xxFD::new(obd_device, int);

    obd_controller.reset_and_apply_config(&Config {
        clock: Clock::Clock20MHz,
        bit_rate: BitRate::default(),
        ecc_enabled: true,
        restrict_retx_attempts: false,
        txq_enabled: false,
        tx_event_fifo_enabled: false,
        iso_crc_enabled: true,
    }).await.unwrap();

    obd_controller.configure_fifo(FIFOConfig::<TRANSMIT_FIFO> {
        transmit: true,
        size: 8,
        payload_size: registers::PayloadSize::Bytes8,
        priority: 1,
        tx_attempts: registers::RetransmissionAttempts::Unlimited1,
    }).await.unwrap();
    obd_controller.configure_fifo(FIFOConfig::<RECEIVE_FIFO> {
        transmit: false,
        size: 16,
        payload_size: registers::PayloadSize::Bytes8,
        priority: 0,
        tx_attempts: registers::RetransmissionAttempts::Unlimited1,
    }).await.unwrap();

    obd_controller.configure_filter(
        FilterConfig::<0, RECEIVE_FIFO>::from_id(StandardId::new(BATTERY_ADDRESS).unwrap()),
        MaskConfig::<0>::from_id(StandardId::MAX), // Match filter address exactly
    ).await.unwrap();

    obd_controller.set_mode(registers::OperationMode::Normal).await.unwrap();

    Timer::after_millis(500).await;

    let battery_query_frame = Frame::new(StandardId::new(BATTERY_ADDRESS).unwrap(), &BATTERY_QUERY).unwrap();

    let mut iso_tp_data: Vec<u8, 64> = Vec::new();
    let mut iso_tp_length: Option<u16> = None;
    loop {
        obd_controller.transmit::<TRANSMIT_FIFO>(&battery_query_frame).await.unwrap();

        let (id, data) = loop {
            let frame = obd_controller.receive().await.unwrap();
            // debug!("Received message from: {:x} ({} bytes): {:x}", frame.raw_id(), frame.data().len(), frame.data());

            match frame.data()[0] >> 4 {
                0 => {
                    // Single ISO-TP frame
                    iso_tp_length = None;
                    iso_tp_data.clear();
                    iso_tp_data.extend_from_slice(&frame.data()[1..]).unwrap();
                    break (frame.raw_id(), &iso_tp_data[3..]); // Strip 0x62 (UDS response) + PID (2 bytes) from data
                },
                1 => {
                    // First ISO-TP frame
                    let size = frame.data()[1] as u16 + ((frame.data()[0] as u16 & 0b1111) << 8);
                    // debug!("First frame of data with total size {}", size);
                    iso_tp_length = Some(size);
                    iso_tp_data.clear();
                    iso_tp_data.extend_from_slice(&frame.data()[2..]).unwrap();

                    // Send flow control message
                    let tx_id: Id = match frame.id() {
                        Id::Standard(id) => StandardId::new(id.as_raw() - 8).unwrap().into(),
                        Id::Extended(id) => ExtendedId::new(id.as_raw() - 8).unwrap().into(),
                    };
                    let flow_control_frame = Frame::new(tx_id, &[0x30, 0x00, 10, 0x00, 0x00, 0x00, 0x00, 0x00]).unwrap();
                    obd_controller.transmit::<TRANSMIT_FIFO>(&flow_control_frame).await.unwrap();
                },
                2 => {
                    // Consecutive ISO-TP frame
                    let _frame_number = frame.data()[0] & 0b1111;
                    // debug!("Consecutive frame #{}", frame_number);
                    iso_tp_data.extend_from_slice(&frame.data()[1..]).unwrap();

                    if iso_tp_data.len() as u16 >= iso_tp_length.unwrap_or(0) {
                        // ISO-TP transmission complete
                        iso_tp_length = None;
                        break (frame.raw_id(), &iso_tp_data[3..]); // Strip 0x62 (UDS response) + PID (2 bytes) from data
                    }
                },
                _ => {},
            }
        };

        match id {
            addr if addr == (BATTERY_ADDRESS as u32 + 8) => process_battery_data(data),
            addr => warn!("Unhandled ISO-TP response from address {:x}", addr),
        }

        Timer::after_millis(5000).await;
    }

    fn process_battery_data(data: &[u8]) {
        let data = BatteryData {
            bms_soc: data[4] as f32 / 2.0,
            bms_relay: data[9] > 0,
            battery_current: i16::from_be_bytes(data[10..12].try_into().unwrap()) as f32 / 10.0,
            battery_voltage: u16::from_be_bytes(data[12..14].try_into().unwrap()) as f32 / 10.0,
            aux_battery_voltage: data[29] as f32 / 10.0,
            available_power: u16::from_be_bytes(data[5..7].try_into().unwrap()),
            charge_current_request: u16::from_be_bytes(data[7..9].try_into().unwrap()) as f32 / 10.0,
        };
        BATTERY_SIGNAL.signal(data);
    }
}

#[embassy_executor::task]
async fn comma_task(spi_bus: &'static Mutex<CriticalSectionRawMutex, SPI0Type<SPI0>>, cs: Output<'static>, int: Input<'static>) {
    const TRANSMIT_FIFO: u8 = 1;

    let comma_device = SpiDevice::new(spi_bus, cs);
    let mut comma_controller = MCP25xxFD::new(comma_device, int);

    comma_controller.reset_and_apply_config(&Config {
        clock: Clock::Clock20MHz,
        bit_rate: BitRate::default(),
        ecc_enabled: true,
        restrict_retx_attempts: false,
        txq_enabled: false,
        tx_event_fifo_enabled: false,
        iso_crc_enabled: true,
    }).await.unwrap();

    comma_controller.configure_fifo(FIFOConfig::<TRANSMIT_FIFO> {
        transmit: true,
        size: 8,
        payload_size: registers::PayloadSize::Bytes64,
        priority: 1,
        tx_attempts: registers::RetransmissionAttempts::Unlimited1,
    }).await.unwrap();

    comma_controller.set_mode(registers::OperationMode::Normal).await.unwrap();

    Timer::after_millis(500).await;

    loop {
        let battery_data = BATTERY_SIGNAL.wait().await;

        info!("\nSOC: {}%\nRelay: {}\nBattery Current: {} A\nBattery Voltage: {} V\nAux Battery Voltage: {} V\nAvailable Power: {} kW\nCharge Current Request: {} A",
        battery_data.bms_soc, battery_data.bms_relay, battery_data.battery_current, battery_data.battery_voltage, battery_data.aux_battery_voltage, battery_data.available_power, battery_data.charge_current_request);

        let data = &[battery_data.bms_soc as u8 * 2, battery_data.bms_soc as u8 * 2, battery_data.bms_soc as u8 * 2, battery_data.bms_soc as u8 * 2, battery_data.bms_soc as u8 * 2, battery_data.bms_soc as u8 * 2, battery_data.bms_soc as u8 * 2, battery_data.bms_soc as u8 * 2];
        let battery_query_frame = Frame::new(StandardId::new(0x715).unwrap(), data).unwrap();

        comma_controller.transmit::<TRANSMIT_FIFO>(&battery_query_frame).await.unwrap();
    }
}