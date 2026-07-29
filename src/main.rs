#![no_std]
#![no_main]

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
use static_cell::StaticCell;
mod obd_data_nostd;

use crate::obd_data_nostd::Process;

use {defmt_rtt as _, panic_probe as _};

type SPI0Type<BUS> = Spi<'static, BUS, spi::Async>;
static SPI_BUS0: StaticCell<Mutex<CriticalSectionRawMutex, SPI0Type<SPI0>>> = StaticCell::new();

static OBD_CONTROLLER: StaticCell<
    Mutex<
        CriticalSectionRawMutex,
        MCP25xxFD<SpiDevice<CriticalSectionRawMutex, SPI0Type<SPI0>, Output>>,
    >,
> = StaticCell::new();

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
    bcm: Id,
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
            bcm: StandardId::new(0x770).unwrap().into(),
        };
        let rx = Self {
            bms: Self::rx_address(tx.bms),
            tpms: Self::rx_address(tx.tpms),
            hvac: Self::rx_address(tx.hvac),
            adas: Self::rx_address(tx.adas),
            iccu: Self::rx_address(tx.iccu),
            vcms: Self::rx_address(tx.vcms),
            dash: Self::rx_address(tx.dash),
            bcm: Self::rx_address(tx.bcm),
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

    let mut comma_stby = Output::new(p.PIN_25, Level::Low);
    comma_stby.set_high();

    spawner.spawn(obd_task(spawner, spi0, obd_cs, obd_int).unwrap());
}

const TX_FIFO: u8 = 1;
const RX_FIFO: u8 = 2;

#[embassy_executor::task]
async fn obd_task(
    spawner: Spawner,
    spi_bus: &'static Mutex<CriticalSectionRawMutex, SPI0Type<SPI0>>,
    cs: Output<'static>,
    mut int: Input<'static>,
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
                restrict_retx_attempts: false,
                txq_enabled: false,
                tx_event_fifo_enabled: false,
                iso_crc_enabled: true,
            })
            .await
            .unwrap();

        obd_controller
            .configure_fifo(FIFOConfig::<TX_FIFO>::tx_with_size(8, PayloadSize::Bytes8))
            .await
            .unwrap();

        obd_controller
            .configure_fifo(FIFOConfig::<RX_FIFO>::rx_with_size(32, PayloadSize::Bytes8))
            .await
            .unwrap();
        obd_controller
            .configure_filter(
                FilterConfig::<RX_FIFO, RX_FIFO>::from_id(rx_addrs.bms),
                MaskConfig::<RX_FIFO>::match_anything(),
            )
            .await
            .unwrap();

        obd_controller
            .set_mode(registers::OperationMode::Normal)
            .await
            .unwrap();
        Timer::after_millis(500).await;
    }
    spawner.spawn(obd_sender_task(obd_controller, tx_addrs).unwrap());

    #[derive(Format)]
    struct ISOTPTransfer {
        rx_addr: Id,
        raw_data: Vec<u8, 256>,
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
        let mut obd_controller = obd_controller.lock().await;
        let mut transfer: Option<ISOTPTransfer> = None;

        loop {
            match obd_controller.receive(None).await {
                Ok(Some((fifo, frame))) => {
                    trace!(
                        "RX: {:x} ({} bytes): {:x}",
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
                            if length >= 256 {
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
                            // Handled by other device
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
                                        break;
                                    }
                                }
                                None => {
                                    warn!("Received consecutive frame without an active transfer!")
                                }
                            }
                        }
                        3 => {} // Continuation frame
                        x => {
                            warn!("Unknown value: {}", x);
                        }
                    }
                }
                Ok(None) => {
                    // No message in the specified RX FIFO, wait for another RX interrupt before continuing);
                }
                Err(mcp25xxfd::Error::ControllerError(description)) => {
                    error!("{}", description);
                }
                Err(err) => {
                    error!("{:?}", err);
                }
            }
        }
        if let Some(transfer) = transfer {
            if transfer.data().iter().all(|&x| x == 0x00 || x == 0xaa) {
                continue;
            }
            info!(
                "ISO-TP: {:x} -> {:x} -> {:02x}: {:x}",
                transfer.raw_rx_addr() - 8,
                transfer.raw_rx_addr(),
                transfer.pid(),
                transfer.data()
            );
            match transfer.rx_addr {
                x if x == rx_addrs.dash => {
                    debug!("{:?}", obd_data_nostd::Dashboard::process(transfer.data()));
                }
                _ => {}
            }
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
) {
    let query: [u8; 8] = [0x03, 0x2c, 0x01, 0xF2, 0x01, 0x00, 0x00, 0x00];
    // let query: [u8; 8] = [0x03, 0x2a, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00];

    let frame = Frame::new(tx_addrs.bms, &query).unwrap();

    obd_controller
        .lock()
        .await
        .transmit::<TX_FIFO>(&frame)
        .await
        .unwrap();

    debug!("Sent!");
}
