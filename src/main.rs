#![no_std]
#![no_main]

use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{ SPI0 };
use embassy_rp::spi::{self, Spi};
use embassy_rp::i2c;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Timer, Delay};
use mcp25xxfd::{registers, MCP25xxFD};
use bme280::i2c::BME280;
use static_cell::StaticCell;
use heapless::Vec;

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
    let mut obd_controller = MCP25xxFD::new(obd_device, None::<Input>);

    let i2c = i2c::I2c::new_blocking(p.I2C0, p.PIN_1, p.PIN_0, i2c::Config::default());
    let mut bme280 = BME280::new_primary(i2c);
    bme280.init(&mut Delay).unwrap();

    // Reset device
    obd_controller.reset().await.unwrap();
    // Enable ECC
    let mut ecc_register = obd_controller.read_register::<registers::ECCControl>().await.unwrap();
    ecc_register.set_eccen(true);
    obd_controller.write_register(ecc_register).await.unwrap();
    // Initialize RAM
    obd_controller.initialize_ram(0xFF).await.unwrap();
    // Configure device
    let mut can_config = obd_controller.read_register::<registers::CANControl>().await.unwrap();
    can_config.set_isocrcen(true);
    can_config.set_stef(false);
    //can_config.set_rtxat(true); // TODO: disable this for unlimited retransmissions
    obd_controller.write_register(can_config).await.unwrap();
    // Setup TX FIFO
    let mut tx_config = obd_controller.read_register::<registers::FIFOControl<TRANSMIT_FIFO>>().await.unwrap();
    tx_config.contents.set_txen(true); // Set as transmit FIFO
    tx_config.contents.set_fsize(7); // 8 messages deep
    tx_config.contents.set_plsize(registers::PayloadSize::Bytes64);
    tx_config.contents.set_txpri(1); // 0 is lowest priority
    //tx_config.contents.set_txat(registers::RetransmissionAttempts::Three); // TODO: disable this for unlimited retransmissions
    obd_controller.write_register(tx_config).await.unwrap();
    // Setup RX FIFO
    let mut rx_config = obd_controller.read_register::<registers::FIFOControl<RECEIVE_FIFO>>().await.unwrap();
    rx_config.contents.set_txen(false); // Set as receive FIFO
    rx_config.contents.set_fsize(15); // 16 messages deep
    rx_config.contents.set_plsize(registers::PayloadSize::Bytes64);
    rx_config.contents.set_tfnrfnie(true); // Interrupt enabled for FIFO not empty (i.e. message received)
    obd_controller.write_register(rx_config).await.unwrap();

    // Setup filters
    let mut mask0: registers::Mask<0> = obd_controller.read_register().await.unwrap();
    mask0.contents.set_msid(0); // Accept all
    mask0.contents.set_mide(true); // Match only (standard|extended ID) that correspond to EXIDE bit in filter
    obd_controller.write_register(mask0).await.unwrap();

    let mut filter0: registers::FilterObject<0> = obd_controller.read_register().await.unwrap();
    filter0.contents.set_exide(false); // Only accept standard identifier
    filter0.contents.set_sid(0); // Doesn't matter -- masked out
    obd_controller.write_register(filter0).await.unwrap();

    let mut filter_control: registers::FilterControl<0> = obd_controller.read_register().await.unwrap();
    filter_control.contents.set_f0bp(RECEIVE_FIFO);
    filter_control.contents.set_flten0(true);
    obd_controller.write_register(filter_control).await.unwrap();

    // For a 20MHz clock, 500 kbit/s + 2 Mbit/s
    let mut nominal_bit_time_config = obd_controller.read_register::<registers::NominalBitTimeConfig>().await.unwrap();
    nominal_bit_time_config.set_brp(0);
    nominal_bit_time_config.set_tseg1(30);
    nominal_bit_time_config.set_tseg2(7);
    nominal_bit_time_config.set_sjw(7);
    obd_controller.write_register(nominal_bit_time_config).await.unwrap();

    let mut data_bit_time_config = obd_controller.read_register::<registers::DataBitTimeConfig>().await.unwrap();
    data_bit_time_config.set_brp(0);
    data_bit_time_config.set_tseg1(6);
    data_bit_time_config.set_tseg2(1);
    data_bit_time_config.set_sjw(1);
    obd_controller.write_register(data_bit_time_config).await.unwrap();

    let mut tx_delay_compensation = obd_controller.read_register::<registers::TransmitterDelayCompensation>().await.unwrap();
    tx_delay_compensation.set_tdcmod(2);
    tx_delay_compensation.set_tdco(7);
    tx_delay_compensation.set_tdcv(0);
    obd_controller.write_register(tx_delay_compensation).await.unwrap();

    // Setup interrupts
    let mut interrupt_config = obd_controller.read_register::<registers::Interrupts>().await.unwrap();
    interrupt_config.set_txie(false);
    interrupt_config.set_rxie(true);
    interrupt_config.set_cerrie(true);
    obd_controller.write_register(interrupt_config).await.unwrap();

    // Enter Normal Mode
    let mut can_config = obd_controller.read_register::<registers::CANControl>().await.unwrap();
    can_config.set_reqop(registers::OperationMode::Normal);
    obd_controller.write_register(can_config).await.unwrap();

    Timer::after_millis(500).await;

    const BATTERY_ADDRESS: u16 = 0x7E4;
    const BATTERY_QUERY: [u8; 8] = [0x03, 0x22, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00];
    transmit_message(&mut obd_controller, BATTERY_ADDRESS, &BATTERY_QUERY).await;

    let mut battery_data: Vec<u8, 64> = Vec::new();
    let mut iso_tp_length: Option<u16> = None;
    loop {
        if obd_int.is_low() {
            let mut interrupts: registers::Interrupts = obd_controller.read_register().await.unwrap();

            if interrupts.cerrif() {
                warn!("CAN Bus error!");
                interrupts.set_cerrif(false);
                obd_controller.write_register(interrupts).await.unwrap();
            }

            if interrupts.rxif() {
                let rx_addr = obd_controller.read_register::<registers::FIFOUserAddress<RECEIVE_FIFO>>().await.unwrap()
                    .contents
                    .fifoua() as u16;

                let rx_header = registers::ReceiveMessageObjectHeader::from_bytes(obd_controller.read_bytes(rx_addr).await.unwrap());
                let rx_raw_data: [u8; 64] = obd_controller.read_bytes(rx_addr + size_of::<registers::ReceiveMessageObjectHeader>() as u16).await.unwrap();
                let rx_data = &rx_raw_data[0..rx_header.dlc().bytes()];

                let mut rx_control = obd_controller.read_register::<registers::FIFOControl<RECEIVE_FIFO>>().await.unwrap();
                rx_control.contents.set_uinc(true);
                obd_controller.write_register(rx_control).await.unwrap();

                debug!("Received message from: {} ({} bytes): {:x}", rx_header.sid(), rx_data.len(), rx_data);

                match rx_data[0] >> 4 {
                    0 => {},
                    1 => {
                        // First ISO-TP frame
                        let size = rx_data[1] as u16 + ((rx_data[0] as u16 & 0b1111) << 8);
                        debug!("First frame of data with total size {}", size);
                        iso_tp_length = Some(size);
                        battery_data.clear();
                        battery_data.extend_from_slice(&rx_data[2..]).unwrap();

                        // Send flow control message
                        transmit_message(&mut obd_controller, rx_header.sid() - 8, &[0x30, 0x00, 10, 0x00, 0x00, 0x00, 0x00, 0x00]).await;
                    },
                    2 => {
                        // Consecutive ISO-TP frame
                        let frame_number = rx_data[0] & 0b1111;
                        debug!("Consecutive frame #{}", frame_number);
                        battery_data.extend_from_slice(&rx_data[1..]).unwrap();

                        if battery_data.len() as u16 >= iso_tp_length.unwrap_or(0) {
                            iso_tp_length = None;

                            process_battery_data(&battery_data[3..]); // Strip 0x62 (UDS response) + PID (2 bytes) from data
                        }
                    },
                    _ => {},
                }
            }
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

async fn transmit_message(controller: &mut MCP25xxFD<SpiDevice<'_, CriticalSectionRawMutex, SPI0Type<SPI0>, Output<'_>>, Input<'_>>, address: u16, message: &[u8]) {
    // Check FIFO availability
    let tx_status = controller.read_register::<registers::FIFOStatus<TRANSMIT_FIFO>>().await.unwrap();
    defmt::assert!(tx_status.contents.tfnrfnif(), "No room in TX FIFO!");

    let mut tx_obj = registers::TransmitMessageObjectHeader::new();
    tx_obj.set_dlc(registers::DataLengthCode::DLC_8);
    tx_obj.set_ide(false);
    tx_obj.set_rtr(false);
    tx_obj.set_brs(false);
    tx_obj.set_fdf(false);
    tx_obj.set_sid(address);

    let tx_addr = controller.read_register::<registers::FIFOUserAddress<TRANSMIT_FIFO>>().await.unwrap()
        .contents
        .fifoua() as u16;

    controller.write_bytes(tx_addr, &tx_obj.into_bytes()).await.unwrap();
    controller.write_bytes(tx_addr + size_of::<registers::TransmitMessageObjectHeader>() as u16, message).await.unwrap();

    let mut tx_control = controller.read_register::<registers::FIFOControl<TRANSMIT_FIFO>>().await.unwrap();
    tx_control.contents.set_uinc(true);
    tx_control.contents.set_txreq(true);
    controller.write_register(tx_control).await.unwrap();
}