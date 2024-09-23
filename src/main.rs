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
use embassy_time::{Delay, Timer};
use embedded_can::{ExtendedId, Id, StandardId};
use heapless::Vec;
use mcp25xxfd::frame::Frame;
use mcp25xxfd::{config::{BitRate, Clock, Config, FIFOConfig, FilterConfig, MaskConfig}, registers, Error, MCP25xxFD};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

type SPI0Type<BUS> = Spi<'static, BUS, spi::Async>;
static SPI_BUS0: StaticCell<Mutex<CriticalSectionRawMutex, SPI0Type<SPI0>>> = StaticCell::new();

const TRANSMIT_FIFO: u8 = 1;
const RECEIVE_FIFO: u8 = 2;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
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
    let mut obd_stby = Output::new(p.PIN_25, Level::Low);
    obd_stby.set_low();

    let obd_device = SpiDevice::new(spi0, obd_cs);
    let mut obd_controller = MCP25xxFD::new(obd_device, obd_int);

    let i2c = i2c::I2c::new_blocking(p.I2C0, p.PIN_1, p.PIN_0, i2c::Config::default());
    let mut bme280 = BME280::new_primary(i2c);
    bme280.init(&mut Delay).unwrap();

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
        FilterConfig::<0, RECEIVE_FIFO>::from_id(StandardId::new(0x00).unwrap()),
        MaskConfig::<0>::from_id(StandardId::new(0x00).unwrap()), // Accept all
    ).await.unwrap();

    obd_controller.set_mode(registers::OperationMode::Normal).await.unwrap();

    Timer::after_millis(500).await;

    const BATTERY_ADDRESS: u16 = 0x7E4;
    const BATTERY_QUERY: [u8; 8] = [0x03, 0x22, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00];

    let tx_frame = Frame::new(StandardId::new(BATTERY_ADDRESS).unwrap(), &BATTERY_QUERY).unwrap();
    obd_controller.transmit::<TRANSMIT_FIFO>(&tx_frame).await.unwrap();

    let mut battery_data: Vec<u8, 64> = Vec::new();
    let mut iso_tp_length: Option<u16> = None;
    loop {
        let frame = obd_controller.receive().await.unwrap();

        debug!("Received message from: {:x} ({} bytes): {:x}", frame.raw_id(), frame.data().len(), frame.data());

        match frame.data()[0] >> 4 {
            0 => {},
            1 => {
                // First ISO-TP frame
                let size = frame.data()[1] as u16 + ((frame.data()[0] as u16 & 0b1111) << 8);
                debug!("First frame of data with total size {}", size);
                iso_tp_length = Some(size);
                battery_data.clear();
                battery_data.extend_from_slice(&frame.data()[2..]).unwrap();

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
                let frame_number = frame.data()[0] & 0b1111;
                debug!("Consecutive frame #{}", frame_number);
                battery_data.extend_from_slice(&frame.data()[1..]).unwrap();

                if battery_data.len() as u16 >= iso_tp_length.unwrap_or(0) {
                    iso_tp_length = None;

                    process_battery_data(&battery_data[3..]); // Strip 0x62 (UDS response) + PID (2 bytes) from data
                }
            },
            _ => {},
        }

        Timer::after_millis(10).await;
    }
}

fn process_battery_data(data: &[u8]) {
    let bms_soc: f32 = data[4] as f32 / 2.0;
    let bms_relay: bool = data[9] > 0;
    let battery_current: f32 = i16::from_be_bytes(data[10..12].try_into().unwrap()) as f32 / 10.0;
    let battery_voltage: f32 = u16::from_be_bytes(data[12..14].try_into().unwrap()) as f32 / 10.0;
    let aux_battery_voltage: f32 = data[29] as f32 / 10.0;
    let available_power: u16 = u16::from_be_bytes(data[5..7].try_into().unwrap());
    let charge_current_request: f32 = u16::from_be_bytes(data[7..9          ].try_into().unwrap()) as f32 / 10.0;

    info!("\nSOC: {}%\nRelay: {}\nBattery Current: {} A\nBattery Voltage: {} V\nAux Battery Voltage: {} V\nAvailable Power: {} kW\nCharge Current Request: {} A",
        bms_soc, bms_relay, battery_current, battery_voltage, aux_battery_voltage, available_power, charge_current_request);
}
