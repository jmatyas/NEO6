#![no_main]
#![no_std]

use panic_semihosting as _;

use core::cell::{UnsafeCell, RefCell};
use core::ops::{DerefMut};


use cortex_m_rt::entry;
use cortex_m::singleton;
use cortex_m::interrupt::{
    free,
    Mutex
};

mod neo;
use neo::{MSG, NEO6, GPS_Data};
use nb::block;
use embedded_hal::serial::Write;

use stm32f1xx_hal as hal;
use hal::{
    prelude::*,
    stm32,
    dma,
    delay::Delay,
    serial::{self, Serial, Config, Rx1, Rx3, Tx1, Tx3},
    stm32::{interrupt, NVIC},
    // gpio::{gpioa, Alternate, PushPull, Input, Floating},
};
use cortex_m_semihosting::{hprintln};

// static G_UART: Mutex<RefCell<Option<Serial<stm32::USART1, (gpioa::PA9<Alternate<PushPull>>, gpioa::PA10<Input<Floating>>)>>>> = Mutex::new(RefCell::new(None));
// static G_RXDMA: Mutex<RefCell<Option<RxDma1>>> = Mutex::new(RefCell::new(None));
// static G_TXDMA: Mutex<RefCell<Option<TxDma1>>> = Mutex::new(RefCell::new(None));
// static G_SERIAL_CIRC: Mutex<RefCell<Option<SerialCircDMA<RxDma1>>>> = Mutex::new(RefCell::new(None));
static G_NEO: Mutex<RefCell<Option<NEO6<Rx3, Tx>>>> = Mutex::new(RefCell::new(None));
// static G_DATA: Mutex<RefCell<Option<GPS_Data>>> = Mutex::new(RefCell::new(None));
// static GPS_VALID: Mutex<RefCell<Option<bool>>> = Mutex::new(RefCell::new(None));

pub type Tx = Tx1;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = stm32::CorePeripherals::take().unwrap();
    
    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();

    let clocks = rcc
                .cfgr
                .use_hse(8.mhz())
                .hclk(72.mhz())
                .pclk1(36.mhz())
                .pclk2(72.mhz())
                .sysclk(72.mhz())
                .freeze(&mut flash.acr);


    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut _gpioc = dp.GPIOC.split(&mut rcc.apb2);

    let mut dma_channels = dp.DMA1.split(&mut rcc.ahb);


    let mut delay = Delay::new(cp.SYST, clocks);

    let tx_pin = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let rx_pin = gpioa.pa10.into_floating_input(&mut gpioa.crh);

    let mut serial = Serial::usart1(
        dp.USART1,
        (tx_pin, rx_pin),
        &mut afio.mapr,
        Config::default()
                .baudrate(115200.bps())
                .parity_none(),
        clocks,
        &mut rcc.apb2,
    );

    // serial.listen(serial::Event::Rxne);
    // serial.listen(serial::Event::Txe);
    // serial.listen(serial::Event::Tc);
    // serial.listen(serial::Event::Idle);
    let (mut log_tx, log_rx) = serial.split();
    for byte in b"ADAS" {
        block!(log_tx.write(*byte)).ok();
    }
    const LEN: usize = 200;

    let tx_buff = singleton!(: [u8; LEN] = [0; LEN] ).unwrap();
    let mut msg_buff = [0u8; 1024];
    let msg_len = msg_buff.len();


    let mut tx_pin = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    let mut rx_pin = gpiob.pb11.into_floating_input(&mut gpiob.crh);

    let mut gps_serial = Serial::usart3(
        dp.USART3,
        (tx_pin, rx_pin),
        &mut afio.mapr,
        Config::default()
                .baudrate(115200.bps())
                .parity_none(),
        clocks,
        &mut rcc.apb1,
    );

    gps_serial.listen(serial::Event::Rxne);

    let (gps_tx, gps_rx) = gps_serial.split();
    let mut neo = NEO6::new(tx_buff, gps_rx, log_tx);
    
    let mut GPS_VALID=false;

    free(|cs| {
        G_NEO.borrow(cs).replace(Some(neo));
    });

    NVIC::unpend(stm32::Interrupt::USART3);
    unsafe {
        NVIC::unmask(stm32::Interrupt::USART3);
    };
    delay.delay_ms(500u16);


    let mut gps_data = GPS_Data::new();
    let mut s = 0u8;
    let mut update = true;

    loop {
        if update {
            free(|cs| {
                let mut neo_ref = G_NEO.borrow(cs).borrow_mut();
                if let Some(ref mut neo) = neo_ref.deref_mut() {
                    neo.parse();
                    if neo.data_valid() {
                        neo.unlisten();
                        GPS_VALID = true;
                        gps_data = neo.get_data();
                        update = false;                        
                    }
                };
            });
        }
        if GPS_VALID {
            hprintln!("{}", gps_data.get_date());
        }
    }
}

#[interrupt]
fn USART3() {
    free(|cs| {
        // hprintln!("d");
        let mut neo_ref = G_NEO.borrow(cs).borrow_mut();
        if let Some(ref mut neo) = neo_ref.deref_mut() {
            neo.receive();
        }

    });
}
