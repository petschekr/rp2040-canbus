#![no_std]
#![no_main]

use bme280_rs::{AsyncBme280, Humidity, Temperature};
use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, I2C0, SPI0};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{dma, i2c};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use embedded_can::{ExtendedId, Id, StandardId};
use heapless::Vec;
use mcp25xxfd::frame::Frame;
use mcp25xxfd::registers::PayloadSize;
use mcp25xxfd::{
    MCP25xxFD,
    config::{BitRate, Clock, Config, FIFOConfig, FilterConfig, MaskConfig},
    registers,
};
use micromath::F32Ext;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

type SPI0Type<BUS> = Spi<'static, BUS, spi::Async>;
static SPI_BUS0: StaticCell<Mutex<CriticalSectionRawMutex, SPI0Type<SPI0>>> = StaticCell::new();

static FORWARDING_CHANNEL: Channel<CriticalSectionRawMutex, (StandardId, Vec<u8, 64>), 10> =
    Channel::new();
static RESPONSE_COMPLETE_CHANNEL: Channel<CriticalSectionRawMutex, StandardId, 10> = Channel::new();

static OBD_CONTROLLER: StaticCell<
    Mutex<
        CriticalSectionRawMutex,
        MCP25xxFD<SpiDevice<CriticalSectionRawMutex, SPI0Type<SPI0>, Output>>,
    >,
> = StaticCell::new();
static COMMA_CONTROLLER: StaticCell<
    Mutex<
        CriticalSectionRawMutex,
        MCP25xxFD<SpiDevice<CriticalSectionRawMutex, SPI0Type<SPI0>, Output>>,
    >,
> = StaticCell::new();

static CAR_OFF_SINCE: StaticCell<Mutex<CriticalSectionRawMutex, Option<Instant>>> =
    StaticCell::new();

embassy_rp::bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

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
    hvac: Id,
    adas: Id,
    iccu: Id,
    vcms: Id,
    dash: Id,
    bdc: Id,
    vcu: Id,
}
impl ECUAddresses {
    fn new() -> (Self, Self) {
        let tx = Self {
            bms: StandardId::new(0x7E4).unwrap().into(),
            tpms: StandardId::new(0x7A0).unwrap().into(),
            hvac: StandardId::new(0x7B3).unwrap().into(),
            adas: StandardId::new(0x730).unwrap().into(),
            iccu: StandardId::new(0x7E5).unwrap().into(),
            vcms: StandardId::new(0x744).unwrap().into(),
            dash: StandardId::new(0x7C6).unwrap().into(),
            bdc: StandardId::new(0x770).unwrap().into(),
            vcu: StandardId::new(0x7E2).unwrap().into(),
        };
        let rx = Self {
            bms: Self::rx_address(tx.bms),
            tpms: Self::rx_address(tx.tpms),
            hvac: Self::rx_address(tx.hvac),
            adas: Self::rx_address(tx.adas),
            iccu: Self::rx_address(tx.iccu),
            vcms: Self::rx_address(tx.vcms),
            dash: Self::rx_address(tx.dash),
            bdc: Self::rx_address(tx.bdc),
            vcu: Self::rx_address(tx.vcu),
        };
        (tx, rx)
    }
    fn address_offset<const O: i32>(ecu_addr: impl Into<Id>) -> Id {
        let ecu_addr = ecu_addr.into();
        match ecu_addr {
            Id::Standard(addr) => StandardId::new(((addr.as_raw() as i32) + O) as u16)
                .unwrap()
                .into(),
            Id::Extended(addr) => ExtendedId::new(((addr.as_raw() as i32) + O) as u32)
                .unwrap()
                .into(),
        }
    }
    fn rx_address(ecu_addr: impl Into<Id>) -> Id {
        Self::address_offset::<8>(ecu_addr)
    }
    fn tx_address(ecu_addr: impl Into<Id>) -> Id {
        Self::address_offset::<-8>(ecu_addr)
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
        Irqs,
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

    let i2c = i2c::I2c::new_async(p.I2C0, p.PIN_1, p.PIN_0, Irqs, i2c::Config::default());

    let car_off_since = CAR_OFF_SINCE.init(Mutex::new(None));

    spawner.spawn(obd_task(spawner, spi0, obd_cs, obd_int, car_off_since).unwrap());
    spawner.spawn(bme_sender_task(i2c).unwrap());
    spawner.spawn(comma_task(spawner, spi0, comma_cs, comma_int, car_off_since).unwrap());
}

const TRANSMIT_FIFO: u8 = 1;
const RX_BATTERY_FIFO: u8 = 2;
const RX_TPMS_FIFO: u8 = 3;
const RX_HVAC_FIFO: u8 = 4;
const RX_ADAS_FIFO: u8 = 5;
const RX_ICCU_FIFO: u8 = 6;
const RX_VCMS_FIFO: u8 = 7;
const RX_DASH_FIFO: u8 = 8;
const RX_BDC_FIFO: u8 = 9;
const RX_VCU_FIFO: u8 = 10;

#[embassy_executor::task]
async fn obd_task(
    spawner: Spawner,
    spi_bus: &'static Mutex<CriticalSectionRawMutex, SPI0Type<SPI0>>,
    cs: Output<'static>,
    mut int: Input<'static>,
    car_off_since: &'static Mutex<CriticalSectionRawMutex, Option<Instant>>,
) {
    let (tx_addrs, rx_addrs) = ECUAddresses::new();

    let obd_device = SpiDevice::new(spi_bus, cs);
    let obd_controller = OBD_CONTROLLER.init(Mutex::new(MCP25xxFD::new(obd_device)));

    {
        let mut obd_controller = obd_controller.lock().await;
        obd_controller
            .reset_and_apply_config(&Config {
                clock: Clock::Clock20MHz,
                bit_rate: BitRate::default(),
                ecc_enabled: true,
                restrict_retx_attempts: true,
                txq_enabled: false,
                tx_event_fifo_enabled: false,
                iso_crc_enabled: true,
            })
            .await
            .unwrap();

        obd_controller
            .configure_fifo(FIFOConfig::<TRANSMIT_FIFO>::tx_with_size(
                32,
                PayloadSize::Bytes8,
            ))
            .await
            .unwrap();

        async fn configure_rx_fifo<'a, const FIFO: u8>(
            controller: &mut MCP25xxFD<
                SpiDevice<'a, CriticalSectionRawMutex, SPI0Type<SPI0>, Output<'a>>,
            >,
            rx_addr: impl Into<Id>,
        ) {
            controller
                .configure_fifo(FIFOConfig::<FIFO>::rx_with_size(8, PayloadSize::Bytes8))
                .await
                .unwrap();
            controller
                .configure_filter(
                    FilterConfig::<FIFO, FIFO>::from_id(rx_addr),
                    MaskConfig::<FIFO>::match_exact(),
                )
                .await
                .unwrap();
        }

        configure_rx_fifo::<RX_BATTERY_FIFO>(&mut obd_controller, rx_addrs.bms).await;
        configure_rx_fifo::<RX_TPMS_FIFO>(&mut obd_controller, rx_addrs.tpms).await;
        configure_rx_fifo::<RX_HVAC_FIFO>(&mut obd_controller, rx_addrs.hvac).await;
        configure_rx_fifo::<RX_ADAS_FIFO>(&mut obd_controller, rx_addrs.adas).await;
        configure_rx_fifo::<RX_ICCU_FIFO>(&mut obd_controller, rx_addrs.iccu).await;
        configure_rx_fifo::<RX_VCMS_FIFO>(&mut obd_controller, rx_addrs.vcms).await;
        configure_rx_fifo::<RX_DASH_FIFO>(&mut obd_controller, rx_addrs.dash).await;
        configure_rx_fifo::<RX_BDC_FIFO>(&mut obd_controller, rx_addrs.bdc).await;
        configure_rx_fifo::<RX_VCU_FIFO>(&mut obd_controller, rx_addrs.vcu).await;

        obd_controller
            .set_mode(registers::OperationMode::Normal)
            .await
            .unwrap();
        Timer::after_millis(500).await;
    }
    spawner.spawn(obd_sender_task(obd_controller, tx_addrs, car_off_since).unwrap());

    #[derive(Format)]
    struct ISOTPTransfer {
        rx_addr: Id,
        raw_data: Vec<u8, 80>,
        length: u16,
        rx_fifo: u8,
    }
    impl ISOTPTransfer {
        fn new(rx_addr: Id, data: &[u8], length: u16, rx_fifo: u8) -> Self {
            Self {
                rx_addr,
                raw_data: Vec::from_slice(data).unwrap(),
                length,
                rx_fifo,
            }
        }
        fn pid(&self) -> &[u8] {
            // First byte is UDS response type
            // Next two bytes are requested PID
            &self.raw_data[1..3]
        }
        fn data(&self) -> &[u8] {
            &self.raw_data[3..]
        }
        fn raw_tx_addr(&self) -> u32 {
            self.raw_rx_addr() - 8
        }
        fn raw_rx_addr(&self) -> u32 {
            match self.rx_addr {
                Id::Standard(id) => id.as_raw() as u32,
                Id::Extended(id) => id.as_raw(),
            }
        }
    }

    // Receive loop
    loop {
        // Wait for interrupt pin to go low (aka active) before calling receive so we don't spinlock
        int.wait_for_low().await;
        // Lock the mutex for this receive cycle (sender thread must wait until we're done receiving)
        let mut obd_controller = obd_controller.lock().await;
        let mut transfer: Option<ISOTPTransfer> = None;
        let transfer_start = Instant::now();

        loop {
            let rx_fifo = transfer.as_ref().map(|t| t.rx_fifo); // Hold the RX FIFO number if there is an active transfer
            match obd_controller.receive(rx_fifo).await {
                Ok(Some((fifo, frame))) => {
                    trace!(
                        "Received message from FIFO{}: {:x} ({} bytes): {:x}",
                        fifo,
                        frame.raw_id(),
                        frame.data().len(),
                        frame.data()
                    );

                    match frame.data()[0] >> 4 {
                        0 => {
                            // Single ISO-TP frame
                            trace!("Single frame of data");
                            // ISO-TP transmission complete
                            transfer = Some(ISOTPTransfer::new(
                                frame.id(),
                                &frame.data()[1..],
                                8 - 3,
                                fifo,
                            ));
                            break;
                        }
                        1 => {
                            // First ISO-TP frame
                            let length =
                                frame.data()[1] as u16 + ((frame.data()[0] as u16 & 0b1111) << 8);
                            trace!("First frame of data with total length {}", length);
                            if length >= 80 {
                                warn!(
                                    "Unable to handle ISO-TP transmission with length {} (ECU: {:x}, PID: {:x})",
                                    length,
                                    frame.raw_id(),
                                    &frame.data()
                                );
                                transfer = None;
                                break;
                            }
                            transfer = Some(ISOTPTransfer::new(
                                frame.id(),
                                &frame.data()[2..],
                                length,
                                fifo,
                            ));

                            // Send flow control message to receive the rest of the data
                            let flow_control_frame = Frame::new(
                                ECUAddresses::tx_address(frame.id()),
                                &[0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                            )
                            .unwrap();
                            obd_controller
                                .transmit::<TRANSMIT_FIFO>(&flow_control_frame)
                                .await
                                .unwrap();
                        }
                        2 => {
                            // Consecutive ISO-TP frame
                            let frame_number = frame.data()[0] & 0b1111;
                            trace!("Consecutive frame #{}", frame_number);

                            match transfer {
                                Some(ref mut transfer) => {
                                    let remaining_bytes: usize =
                                        transfer.length as usize - transfer.raw_data.len();
                                    if remaining_bytes > 7 {
                                        transfer
                                            .raw_data
                                            .extend_from_slice(&frame.data()[1..])
                                            .unwrap();
                                    } else {
                                        // Don't copy more bytes than the transfer size
                                        transfer
                                            .raw_data
                                            .extend_from_slice(
                                                &frame.data()[1..1 + remaining_bytes],
                                            )
                                            .unwrap();
                                    }

                                    if transfer.raw_data.len() as u16 >= transfer.length {
                                        // ISO-TP transmission complete
                                        trace!(
                                            "Frame sequence complete {:x} -> {:x}",
                                            transfer.raw_tx_addr(),
                                            transfer.raw_rx_addr()
                                        );
                                        break;
                                    }
                                }
                                None => {
                                    warn!("Received consecutive frame without an active transfer!")
                                }
                            }
                        }
                        _ => {}
                    }
                }
                Ok(None) => {
                    // No message in the specified RX FIFO, wait for another RX interrupt before continuing (but not more than 250 ms)
                    let _ =
                        embassy_time::with_timeout(Duration::from_millis(250), int.wait_for_low())
                            .await;
                }
                Err(mcp25xxfd::Error::ControllerError(description)) => {
                    error!("{} Transfer: {}", description, transfer);
                    FORWARDING_CHANNEL
                        .send((
                            StandardId::new(0x700).unwrap(),
                            Vec::from_slice(description.as_bytes()).unwrap(),
                        ))
                        .await;
                    transfer = None;
                    break;
                }
                Err(err) => {
                    dbg!(err);
                    transfer = None;
                    break;
                }
            }
            // If we've been receiving for more than 250 milliseconds, abort so that we don't hold the mutex lock forever
            if transfer_start.elapsed().as_millis() > 250 {
                if let Some(transfer) = transfer {
                    warn!(
                        "Transfer from {:x} timed out: {:?}",
                        transfer.raw_rx_addr(),
                        transfer
                    );
                } else {
                    warn!("Unknown transfer timed out");
                }
                transfer = None;
                break;
            }
        }

        if let Some(transfer) = transfer.take() {
            let forwarding_address = match transfer.rx_addr {
                addr if addr == rx_addrs.bms && transfer.pid() == [0x01, 0x01] => {
                    let mut car_off_since = car_off_since.lock().await;
                    // Poll more frequently when the HV battery is connected (current > 0 amps)
                    if transfer.data()[10..12] == [0x00, 0x00] {
                        // Battery current is 0.0 amps -- car is off
                        if car_off_since.is_none() {
                            *car_off_since = Some(Instant::now());
                        }
                    } else {
                        // Car is on
                        *car_off_since = None;
                    }
                    0x701
                }
                addr if addr == rx_addrs.bms && transfer.pid() == [0x01, 0x05] => 0x705,
                addr if addr == rx_addrs.bms && transfer.pid() == [0x01, 0x06] => 0x706,
                addr if addr == rx_addrs.bms && transfer.pid() == [0x01, 0x11] => 0x70B,
                addr if addr == rx_addrs.tpms && transfer.pid() == [0xC0, 0x0B] => 0x710,
                addr if addr == rx_addrs.hvac && transfer.pid() == [0x01, 0x00] => 0x720,
                addr if addr == rx_addrs.adas && transfer.pid() == [0xF0, 0x10] => 0x730,
                addr if addr == rx_addrs.iccu && transfer.pid() == [0xE0, 0x01] => 0x741,
                addr if addr == rx_addrs.iccu && transfer.pid() == [0xE0, 0x02] => 0x742,
                addr if addr == rx_addrs.iccu && transfer.pid() == [0xE0, 0x03] => 0x743,
                addr if addr == rx_addrs.iccu && transfer.pid() == [0xE1, 0x01] => 0x74B,
                addr if addr == rx_addrs.vcms && transfer.pid() == [0xE0, 0x01] => 0x751,
                addr if addr == rx_addrs.vcms && transfer.pid() == [0xE0, 0x02] => 0x752,
                addr if addr == rx_addrs.vcms && transfer.pid() == [0xE0, 0x03] => 0x753,
                addr if addr == rx_addrs.vcms && transfer.pid() == [0xE0, 0x04] => 0x754,
                addr if addr == rx_addrs.dash && transfer.pid() == [0xB0, 0x02] => 0x760,
                addr if addr == rx_addrs.dash && transfer.pid() == [0x22, 0x78] => {
                    // Phantom PID that gets emitted in the format [0x7F, 0x22, 0x78] before the actual dash/cluster response
                    0x761
                }
                addr if addr == rx_addrs.bdc && transfer.pid() == [0xBC, 0x13] => {
                    // Byte 4 is 0x00 when locked
                    if transfer.data()[4] != 0x00 {
                        // Consider car to be on and keep quick polling until all doors closed and locked
                        car_off_since.lock().await.take();
                    }

                    0x773
                }
                addr if addr == rx_addrs.bdc && transfer.pid() == [0xBC, 0x15] => {
                    let door_open = transfer.data()[4] & (1 << 5) > 0 // Driver door open
                        || transfer.data()[4] & (1 << 6) > 0 // Passenger door open
                        || transfer.data()[4] & (1 << 7) > 0 // Rear left door open
                        || transfer.data()[5] & (1 << 0) > 0 // Rear right door open
                        || transfer.data()[5] & (1 << 1) > 0 // Hood open
                        || transfer.data()[4] & (1 << 4) > 0; // Trunk open
                    if door_open {
                        // Consider car to be on and keep quick polling until all doors closed and locked
                        car_off_since.lock().await.take();
                    }

                    0x775
                }
                addr if addr == rx_addrs.bdc && transfer.pid() == [0xBC, 0x17] => 0x777,
                _ => {
                    warn!(
                        "Unhandled ISO-TP response from address {:x} -> {:x} to PID {:x}: {:x}\nFull raw data: {:x}",
                        transfer.raw_tx_addr(),
                        transfer.raw_rx_addr(),
                        transfer.pid(),
                        transfer.data(),
                        transfer.raw_data,
                    );
                    continue;
                }
            };
            let forwarding_address = StandardId::new(forwarding_address).unwrap();
            FORWARDING_CHANNEL
                .send((
                    forwarding_address,
                    Vec::from_slice(&transfer.data().chunks(64).next().unwrap()).unwrap(),
                ))
                .await;
            RESPONSE_COMPLETE_CHANNEL
                .send(StandardId::new(transfer.raw_tx_addr() as u16).unwrap())
                .await;
        }
    }
}

#[embassy_executor::task]
async fn obd_sender_task(
    obd_controller: &'static Mutex<
        CriticalSectionRawMutex,
        MCP25xxFD<SpiDevice<'static, CriticalSectionRawMutex, SPI0Type<SPI0>, Output<'static>>>,
    >,
    tx_addrs: ECUAddresses,
    car_off_since: &'static Mutex<CriticalSectionRawMutex, Option<Instant>>,
) {
    let queries = [
        Frame::new(tx_addrs.bms, &construct_uds_query(&[0x01, 0x01])).unwrap(),
        Frame::new(tx_addrs.bms, &construct_uds_query(&[0x01, 0x05])).unwrap(),
        Frame::new(tx_addrs.bdc, &construct_uds_query(&[0xBC, 0x17])).unwrap(), // Brake light
        Frame::new(tx_addrs.bms, &construct_uds_query(&[0x01, 0x11])).unwrap(),
        Frame::new(tx_addrs.tpms, &construct_uds_query(&[0xC0, 0x0B])).unwrap(),
        Frame::new(tx_addrs.hvac, &construct_uds_query(&[0x01, 0x00])).unwrap(),
        Frame::new(tx_addrs.bdc, &construct_uds_query(&[0xBC, 0x17])).unwrap(), // Brake light
        Frame::new(tx_addrs.iccu, &construct_uds_query(&[0xE1, 0x01])).unwrap(),
        Frame::new(tx_addrs.vcms, &construct_uds_query(&[0xE0, 0x01])).unwrap(),
        Frame::new(tx_addrs.vcms, &construct_uds_query(&[0xE0, 0x02])).unwrap(),
        Frame::new(tx_addrs.bdc, &construct_uds_query(&[0xBC, 0x17])).unwrap(), // Brake light
        Frame::new(tx_addrs.vcms, &construct_uds_query(&[0xE0, 0x03])).unwrap(),
        Frame::new(tx_addrs.vcms, &construct_uds_query(&[0xE0, 0x04])).unwrap(),
        Frame::new(tx_addrs.dash, &construct_uds_query(&[0xB0, 0x02])).unwrap(),
        Frame::new(tx_addrs.bdc, &construct_uds_query(&[0xBC, 0x13])).unwrap(),
        Frame::new(tx_addrs.bdc, &construct_uds_query(&[0xBC, 0x15])).unwrap(),
        // Querying brake light after these 2 BDC requests fails to return anything
        // Query it earlier and more often than once per cycle above inbetween other queries

        // Frame::new(tx_addrs.bms, &construct_uds_query(&[0x01, 0x06])).unwrap(), // No useful values
        // Frame::new(tx_addrs.adas, &construct_uds_query(&[0xF0, 0x10])).unwrap(), // No useful values (steering wheel?, g-forces?)
        // Frame::new(tx_addrs.iccu, &construct_uds_query(&[0xE0, 0x01])).unwrap(), // Doesn't work on MY2025
        // Frame::new(tx_addrs.iccu, &construct_uds_query(&[0xE0, 0x02])).unwrap(), // Doesn't work on MY2025
        // Frame::new(tx_addrs.iccu, &construct_uds_query(&[0xE0, 0x03])).unwrap(), // Doesn't work on MY2025
        // Frame::new(tx_addrs.vcu, &construct_uds_query(&[0xBC, 0x06])).unwrap(), // Command bytes are wrong 0x21, 0x01??
    ];

    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        for frame in queries.iter() {
            {
                let mut obd_controller = obd_controller.lock().await;

                match obd_controller.transmit::<TRANSMIT_FIFO>(frame).await {
                    Ok(_) => {
                        debug!("Sent to {:x}", frame.raw_id());
                    }
                    Err(mcp25xxfd::Error::ControllerError(err)) => {
                        error!("OBD CAN controller error: {}", err);
                        // The only reason we would get a controller error is if the TX FIFO is full
                        // So reset the TX FIFO before sending again
                        debug!("TX FIFO reset...");
                        obd_controller.reset_fifo::<TRANSMIT_FIFO>().await.unwrap();
                    }
                    Err(err) => error!("OBD CAN SPI error: {}", err),
                }
            }
            let response = RESPONSE_COMPLETE_CHANNEL.receive();
            match embassy_time::with_timeout(Duration::from_millis(250), response).await {
                Ok(id) => trace!("Received response from {:x}", id.as_raw()),
                Err(_) => warn!("Response timed out but continuing anyway"),
            }
        }
        // Wait 5 minutes between polls if car is off to allow ECUs to deep sleep and save battery
        // Check once per second while waiting to see if car is on again
        for _ in 0..(60 * 5) {
            if let Some(off_time) = *car_off_since.lock().await {
                // If car turned off less than 1 minute ago, exit timer loop and keep quick polling
                if off_time.elapsed().as_secs() < 60 {
                    break;
                }
            } else {
                // Car is on, exit 5-minute timer loop
                break;
            }
            Timer::after_millis(1000).await;
        }
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn bme_sender_task(i2c: i2c::I2c<'static, I2C0, i2c::Async>) {
    let mut bme280 = AsyncBme280::new(i2c, Delay);
    bme280.init().await.unwrap();
    bme280
        .set_sampling_configuration(
            bme280_rs::Configuration::default()
                .with_sensor_mode(bme280_rs::SensorMode::Normal)
                .with_standby_time(bme280_rs::StandbyTime::Millis1000)
                .with_pressure_oversampling(bme280_rs::Oversampling::Oversample8)
                .with_temperature_oversampling(bme280_rs::Oversampling::Oversample8)
                .with_humidity_oversampling(bme280_rs::Oversampling::Oversample8)
                .with_filter(bme280_rs::Filter::Filter4),
        )
        .await
        .unwrap();

    fn compensate_temperature(sensor_temp: Temperature) -> Temperature {
        sensor_temp - 6.0
    }
    fn compensate_humidity(sensor_temp: Temperature, sensor_humidity: Humidity) -> Humidity {
        // From https://www.renesas.com/ja/document/apn/compensating-temperature-and-relative-humidity-pcb
        const A: f32 = 6.1162;
        const M: f32 = 7.5892;
        const TN: f32 = 240.71;

        let corrected_temp = compensate_temperature(sensor_temp);
        let sensor_saturation_vapor_pressure =
            A * f32::powf(10.0, (M * sensor_temp) / (sensor_temp + TN));
        let corrected_saturation_vapor_pressure =
            A * f32::powf(10.0, (M * corrected_temp) / (corrected_temp + TN));
        let vapor_pressure = (sensor_humidity * sensor_saturation_vapor_pressure) / 100.0;
        (vapor_pressure / corrected_saturation_vapor_pressure) * 100.0
    }

    let mut ticker = Ticker::every(Duration::from_secs(30));
    loop {
        let mut forward_data: Vec<u8, 64> = Vec::new();

        let sample = bme280.read_sample().await.unwrap();
        let pressure = sample.pressure.unwrap_or(0.0).to_be_bytes();
        let temperature = compensate_temperature(sample.temperature.unwrap_or(0.0)).to_be_bytes();
        let humidity = compensate_humidity(
            sample.temperature.unwrap_or(0.0),
            sample.humidity.unwrap_or(0.0),
        )
        .to_be_bytes();

        forward_data.extend_from_slice(&pressure).unwrap();
        forward_data.extend_from_slice(&temperature).unwrap();
        forward_data.extend_from_slice(&humidity).unwrap();
        FORWARDING_CHANNEL
            .send((StandardId::new(0x7A0).unwrap(), forward_data))
            .await;

        ticker.next().await;
    }
}

const IGNITION_FIFO: u8 = 2;
#[embassy_executor::task]
async fn comma_task(
    spawner: Spawner,
    spi_bus: &'static Mutex<CriticalSectionRawMutex, SPI0Type<SPI0>>,
    cs: Output<'static>,
    int: Input<'static>,
    car_off_since: &'static Mutex<CriticalSectionRawMutex, Option<Instant>>,
) {
    let comma_device = SpiDevice::new(spi_bus, cs);
    let comma_controller = COMMA_CONTROLLER.init(Mutex::new(MCP25xxFD::new(comma_device)));
    {
        let mut comma_controller = comma_controller.lock().await;
        comma_controller
            .reset_and_apply_config(&Config {
                clock: Clock::Clock20MHz,
                bit_rate: BitRate::default(),
                ecc_enabled: true,
                restrict_retx_attempts: true,
                txq_enabled: false,
                tx_event_fifo_enabled: false,
                iso_crc_enabled: true,
            })
            .await
            .unwrap();

        comma_controller
            .configure_fifo(FIFOConfig::<TRANSMIT_FIFO>::tx_with_size(
                16,
                PayloadSize::Bytes64,
            ))
            .await
            .unwrap();

        comma_controller
            .configure_fifo(FIFOConfig::<IGNITION_FIFO>::rx_with_size(
                32,
                PayloadSize::Bytes8,
            ))
            .await
            .unwrap();
        comma_controller
            .configure_filter(
                FilterConfig::<IGNITION_FIFO, IGNITION_FIFO>::from_id(
                    // Front radar ECU that transmits when car is on on MY2025 Ioniq 5 (was 0x201 for MY2024)
                    StandardId::new(0x216).unwrap(),
                ),
                MaskConfig::<IGNITION_FIFO>::match_exact(),
            )
            .await
            .unwrap();

        comma_controller
            .set_mode(registers::OperationMode::Normal)
            .await
            .unwrap();
        Timer::after_millis(500).await;
    }
    spawner.spawn(comma_car_on_task(comma_controller, int, car_off_since).unwrap());

    loop {
        let (forward_addr, forward_data) = FORWARDING_CHANNEL.receive().await;
        let forward_frame = Frame::new(forward_addr, forward_data.as_slice()).unwrap();

        debug!(
            "Forwarding {} bytes to address {:x}",
            forward_data.len(),
            forward_addr.as_raw()
        );

        match comma_controller
            .lock()
            .await
            .transmit::<TRANSMIT_FIFO>(&forward_frame)
            .await
        {
            Ok(()) => {}
            Err(err) => {
                error!("Forwarding error: {}", err);
            }
        }
    }
}

#[embassy_executor::task]
async fn comma_car_on_task(
    comma_controller: &'static Mutex<
        CriticalSectionRawMutex,
        MCP25xxFD<SpiDevice<'static, CriticalSectionRawMutex, SPI0Type<SPI0>, Output<'static>>>,
    >,
    mut int: Input<'static>,
    car_off_since: &'static Mutex<CriticalSectionRawMutex, Option<Instant>>,
) {
    loop {
        // Wait for interrupt pin to go low (aka active) before calling receive so we don't spinlock
        int.wait_for_low().await;
        if let Ok(Some(_)) = comma_controller
            .lock()
            .await
            .receive(Some(IGNITION_FIFO))
            .await
        {
            debug!("Car ignition detected via CAN 0");
            *car_off_since.lock().await = None;
        }
        Timer::after_millis(1000).await;
    }
}
