#![no_main]
#![no_std]

use panic_semihosting as _;

use core::cell::RefCell;
use core::ops::{Deref, DerefMut};

// use embedded_hal::embedded_dma::{StaticReadBuffer, StaticWriteBuffer};

use cortex_m_rt::entry;
use cortex_m::singleton;
use cortex_m::interrupt::{
    free,
    CriticalSection,
    Mutex
};


use stm32f1xx_hal as hal;
use hal::{
    prelude::*,
    stm32,
    dma::{self, Half, CircBuffer},
    delay::Delay,
    serial::{self, Serial, Config, RxDma1, TxDma1},
    stm32::{interrupt, NVIC},
    gpio::{gpioa, Alternate, PushPull, Input, Floating},
};
use cortex_m_semihosting::{hprintln, hprint};
use nb::block;

// static uart: Mutex<RefCell<Option<Serial<stm32::USART1, (gpio::gpiob::PB6<Alternate<PushPull>>, gpio::gpiob::PB7<Input<Floating>>)>>>> = Mutex::new(RefCell::new(None));
// static TX: Mutex<RefCell<Option<serial::Tx<stm32::USART1>>>> = Mutex::new(RefCell::new(None));
// static RX: Mutex<RefCell<Option<serial::Rx<stm32::USART1>>>> = Mutex::new(RefCell::new(None));

static G_UART: Mutex<RefCell<Option<Serial<stm32::USART1, (gpioa::PA9<Alternate<PushPull>>, gpioa::PA10<Input<Floating>>)>>>> = Mutex::new(RefCell::new(None));
static G_RXDMA: Mutex<RefCell<Option<RxDma1>>> = Mutex::new(RefCell::new(None));
static G_TXDMA: Mutex<RefCell<Option<TxDma1>>> = Mutex::new(RefCell::new(None));
static G_CIRC_BUFFER: Mutex<RefCell<Option<CircBuffer<[u8; 10], RxDma1>>>> = Mutex::new(RefCell::new(None));

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
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    let mut dma_channels = dp.DMA1.split(&mut rcc.ahb);


    let mut delay = Delay::new(cp.SYST, clocks);

    let mut tx_pin = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let mut rx_pin = gpioa.pa10.into_floating_input(&mut gpioa.crh);

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
    serial.listen(serial::Event::Idle);
    let (mut log_tx, mut log_rx) = serial.split();
    dma_channels.5.listen(dma::Event::HalfTransfer);
    dma_channels.5.listen(dma::Event::TransferComplete);
    let mut log_rx_dma = log_rx.with_dma(dma_channels.5);
    let mut log_tx_dma = log_tx.with_dma(dma_channels.4);
    
    let gps_buff = singleton!(: [[u8; 10]; 2] = [[0; 10]; 2]).unwrap();
    let mut circ_buffer = log_rx_dma.circ_read(gps_buff);

    // log_rx_dma.listen(serial::Event::Idle);    
    
    free(|cs| {
        G_CIRC_BUFFER.borrow(cs).replace(Some(circ_buffer));
        G_TXDMA.borrow(cs).replace(Some(log_tx_dma));
    });

    NVIC::unpend(stm32::Interrupt::DMA1_CHANNEL5);
    NVIC::unpend(stm32::Interrupt::USART1);
    unsafe {
        NVIC::unmask(stm32::Interrupt::DMA1_CHANNEL5);
        NVIC::unmask(stm32::Interrupt::USART1);
    };

    // let log_rx_dma = log_rx.with_dma(dma_channels.5);


    // let mut tx_pin = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    // let mut rx_pin = gpiob.pb11.into_floating_input(&mut gpiob.crh);

    // let mut gps_serial = Serial::usart3(
    //     dp.USART3,
    //     (tx_pin, rx_pin),
    //     &mut afio.mapr,
    //     Config::default()
    //             .baudrate(9600.bps())
    //             .parity_none(),
    //     clocks,
    //     &mut rcc.apb1,
    // );

    // let (mut gps_tx, mut gps_rx) = gps_serial.split();
    // let gps_rx_dma = gps_rx.with_dma(dma_channels.3);
    
    // for byte in b"HelloMordo\r\n" {
    //     block!(log_tx.write(*byte)).unwrap();
    // }


    loop {
        // while circ_buffer.readable_half().unwrap() != Half::First {}
        // let _first_half = circ_buffer.peek(|half, _| *half).unwrap();

        // for byte in _first_half.iter() {
        //     block!(log_tx.write(*byte)).unwrap();
        // }
    
        // while circ_buffer.readable_half().unwrap() != Half::Second {}
        // let _second_half = circ_buffer.peek(|half, _| *half).unwrap();
    
    
       
    
        // for byte in _second_half.iter() {
        //     block!(log_tx.write(*byte)).unwrap();
        // }
    }
}


#[interrupt]
fn USART1() {
    // static mut UART: Option<Serial<stm32::USART1, (gpioa::PA9<Alternate<PushPull>>, gpioa::PA10<Input<Floating>>)>> = None;
    free(|cs|{
        let mut circ_buffer_ref = G_CIRC_BUFFER.borrow(cs).borrow_mut();
        if let Some(ref mut circ_buffer) = circ_buffer_ref.deref_mut() {
            // hprintln!("{:?}", circ_buffer.payload.which_event());
            circ_buffer.payload.clear_event();
            // circ_buffer.stop();
            // hprintln!("USART")
            ()
        // if let Some(ref mut rxdma) = rx_dma_ref.deref_mut() {
            // if let Some(ev) = rxdma.which_event() {
            //     match ev {
            //         serial::Event::Idle => { hprintln!("Idle")},
            //         serial::Event::Rxne => { 
            //             hprintln!("Rxne")
            //         },
            //         _ => hprintln!("Tx"),
            //     }
            } else {
                // hprintln!("D")
            };
            // uart.clear_event()

        // }
    });
}

#[interrupt]
fn DMA1_CHANNEL5() {
    // static mut UART: Option<Serial<stm32::USART1, (gpioa::PA9<Alternate<PushPull>>, gpioa::PA10<Input<Floating>>)>> = None;
    free(|cs|{
        
        let mut circ_buffer_ref = G_CIRC_BUFFER.borrow(cs).borrow_mut();
        let mut txdma_ref = G_TXDMA.borrow(cs).borrow_mut();
        if let (Some(ref mut circ_buffer), Some(tx_dma)) = (circ_buffer_ref.deref_mut(), txdma_ref.deref_mut()) {

            let half = match circ_buffer.readable_half().unwrap() {
                Half::First => {hprintln!("Pierwszy"); Half::First},
                Half::Second => {hprintln!("Drugi"); Half::Second},
            };
            let _msg = circ_buffer.peek(|half, _| *half).unwrap();
            // tx.write(_msg).wait();
            for byte in _msg.iter(){
                hprint!("{:?}", *byte as char);
            }
            // };
            // uart.clear_event()
            } else {
                // hprintln!("Duza dua")
                ()
            }
        // }
    });
}