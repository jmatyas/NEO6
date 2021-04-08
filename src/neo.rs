extern crate embedded_hal;
use embedded_hal::serial::{Read, Write};
use nb::block;
use cortex_m_semihosting::{hprintln, hprint};
use core::sync::atomic::{self, Ordering};

use crate::stm32::{USART1, USART3};
use crate::serial::Event as SEvent;
use crate::{Rx1, Tx1, Rx3};

pub fn atoi(barray: & [u8]) -> u32 {
    let mut value = 0u32;
    let arr_iter = barray.iter();
    for element in barray.iter() {
        value = value*10 + *element as u32 - b'0' as u32;
    }
    value
}

#[derive(Debug)]
pub enum GPS_Statement {
    GPRMC,
    GPGSA,
    GPGGA,
    GPGSV,
    Other,
}


#[derive(Debug)]
pub enum Half {
    First,
    Second,
}

#[derive(Debug)]
pub struct GPSTime {
    hour: u8,
    minute: u8,
    second: u8,
}

impl GPSTime {
    pub fn new() -> Self {
        GPSTime {
            hour: 0,
            minute: 0,
            second: 0,
        }
    }
}

#[derive(Debug)]
pub struct GPSDate {
    day: u8,
    month: u8,
    year: u8,
}


impl GPSDate {
    pub fn new() -> Self {
        GPSDate {
            day: 0,
            month: 0,
            year: 0,
        }
    }
}

#[derive(Debug)]
pub struct Position {
    lattitude: f32,
    ns_indicator: char,
    longitude: f32,
    ew_indicator: char,
    altitude: f32,
}

impl Position {
    pub fn new() -> Self {
        Position {
            lattitude: 0.0,
            ns_indicator: 'N',
            longitude: 0.0,
            ew_indicator: 'E',
            altitude: 0.0,
        }
    }
}

#[derive(Debug)]
pub struct GPSSatellite {
    ID: u8,
    elevation: u8,
    azimuth: u16,
    SNR: u8,
}

#[derive(Debug)]
pub enum FixType {
    NoFix,
    GPSFix,
    DifferentialFix,
}

#[derive(Debug)]
pub enum FixMode {
    NoFix,
    D2,
    D3,
}

pub struct RMC {
    valid: bool,
    date: GPSDate,
}

pub struct GGA {
    position: Position,
    satellites_used: u8,
    fix: FixType,
    time: GPSTime,
}

pub struct GSA {
    hdop: f32,
    vdop: f32,
    pdop: f32,
    fix: FixMode,
    satellite_id: [u8; 12],
}


pub struct MSG <'a> {
    buffer: &'a mut [u8],
    start: bool,
    ptr: usize,
    cap: usize,
    len: usize,
    last_read: Half,
}

impl <'a> MSG <'a> {
    pub fn new(buff:&'a mut[u8], capactiy: usize) -> Self {
        MSG {
            buffer: buff,
            start: false,
            ptr: 0,
            cap: capactiy,
            len: 0,
            last_read: Half::Second,
        }
    }
    pub fn add(&mut self, c: u8) {
        match self.last_read {
            Half::Second => {
                if self.ptr < self.cap {
                    if c == b'$' {
                        self.start = true;
                        self.buffer[self.ptr] = c;
                        self.ptr += 1;
                    } else if c == b'\n' {
                        self.buffer[self.ptr] = c;
                        self.ptr = self.cap;
                        self.last_read = Half::First;
                        self.len += 1;
                        self.start=false;
                    } else if self.start {
                        self.buffer[self.ptr] = c;
                        self.ptr += 1;
                    }
                }
            },
            Half::First => {
                if self.ptr < 2*self.cap {
                    if c == b'$' {
                        self.start = true;
                        self.buffer[self.ptr] = c;
                        self.ptr += 1;
                    } else if c == b'\n' {
                        self.buffer[self.ptr] = c;
                        self.ptr = 0;
                        self.last_read = Half::Second;
                        self.len += 1;
                        self.start = false;
                    } else if self.start {
                        self.buffer[self.ptr] = c;
                        self.ptr += 1;
                    }
                }
            },
        }
    }

    pub fn clear(&mut self) {
        // match self.last_read {
        //     Half::First => {
        //         for byte in self.buffer[..self.cap].iter_mut(){
        //             // hprint!("{:?}", *byte as char);
        //             *byte = 0;
        //         }
        //     },
        //     Half::Second => {
        //         for byte in self.buffer[self.cap..].iter_mut(){
        //             // hprint!("{:?}", *byte as char);
        //             *byte = 0;
        //         }
        //     }
        // };
        self.len -= 1;
    }
    pub fn is_full(&self) -> bool {
        let tmp = if self.len == 2 {true} else {false};
        tmp
    }
    pub fn is_empty(&self) -> bool {
        let tmp = if self.len == 0 {true} else {false};
        tmp
    }


    pub fn get_line(&self) -> (GPS_Statement, &[u8]) {
        if self.len > 0 {
            let mut data = match self.last_read {
                Half::First => &self.buffer[..self.cap],
                Half::Second => &self.buffer[self.cap..],
            };
            let mut msg = data.splitn(2, |c| *c == 0x2C);       // 0d44 split at ,
            let (cmd, info) = (msg.next().unwrap(), msg.next().unwrap());
            // hprintln!("cmd {:?}, info {:?}", cmd, info);
            let (cmd, info) = match cmd {
                b"$GPRMC" => { (GPS_Statement::GPRMC, info) },
                b"$GPGSA" => { (GPS_Statement::GPGSA, info) },
                b"$GPGGA" => { (GPS_Statement::GPGGA, info) },
            //     // b"$GPGSV" => {hprintln!("GPGSV");},
                _ => { (GPS_Statement::Other, info) },

            };
            return (cmd, info);
        }
        (GPS_Statement::Other, &[0u8; 1])
    }
}



pub struct NEO6 <'a, Rx3, Tx1> {
    rx: Rx3,
    tx: Tx1,
    buffer: MSG<'a>,
}

impl <'a> NEO6 <'a, Rx3, Tx1> {
    pub fn new(buf: &'a mut [u8], rx: Rx3, tx: Tx1) -> Self {
        let buf_len =buf.len();
        NEO6 {
            rx: rx,
            tx: tx,
            buffer: MSG::new(buf, buf_len/2),
            
        }
    }
    pub fn listen(&mut self) {
        self.rx.listen(crate::serial::Event::Rxne);
    }
    pub fn unlisten(&mut self) {
        self.rx.unlisten(crate::serial::Event::Rxne);
    }
    pub fn is_valid(&self) -> bool {
        true
    }
    pub fn receive(&mut self) {
        let mut i = 0;
        // block!(self.tx.write(b'a')).ok();
        if let Some(ev) = self.rx.which_event() {
            match ev {
                SEvent::Rxne => {
                    self.rx.clear_event();
                    if !self.buffer.is_full() {
                        let a = unsafe { (*USART3::ptr()).dr.read().bits() as u8};

                        // let a = block!(self.rx.read()).unwrap();
                        self.buffer.add(a);
                        block!(self.tx.write(a)).ok();
                        i += 1;
                    }
                },
                _ => {
                    self.rx.clear_event();
                }
            }
        }
    }
    pub fn get_line(& self) -> (GPS_Statement, &[u8]) {
        self.buffer.get_line()
    }
    pub fn parse(&mut self) {
        if !self.buffer.is_empty() {
            // hprintln!("s");
            let (mut cmd, mut info) = self.get_line();
            match cmd {
                    GPS_Statement::GPRMC => { 
                        let rmc_data = self.parse_rmc(info);
                    },
                    GPS_Statement::GPGSA => {
                        let gsa_data = self.parse_gsa(info);
                    },
                    GPS_Statement::GPGGA => {
                        let gga_data = self.parse_gga(info);
                    },
                    _ => (),

            }
        
            self.buffer.clear();
        }
    }
    pub fn parse_rmc(&self, data: &[u8]) -> RMC {
        let mut GPSdate = GPSDate::new();
        let mut validity = false;

        for (i, field) in data.split(|c| *c == b',').enumerate() {
            if field.len() > 0 {
                match i {
                    // GPSTIME
                    0 => {
                        let (hour, minute, seconds) = (atoi(&field[2..]) as u8, atoi(&field[4..2]) as u8, atoi(& field[6..4]) as u8);
                    },
                    // Receiver Validity
                    1 => {
                        validity = if field[0] == b'A' {
                            true
                        } else {
                            false
                        };
                    },
                    // Lattitude
                    2 => {
                    },
                    // HEMISPHERE indicator
                    3 => {
                    },
                    // LONGITUDE
                    4 => {
                    },
                    // H ind
                    5 => {
                    },
                    // Speed over ground
                    6 => {

                    },
                    // Course over ground
                    7 => {

                    },
                    // GPSDATE
                    8 => {
                        let (day, month, year) = (atoi(&field[2..]) as u8, atoi(&field[4..2]) as u8, atoi(&field[6..4]) as u8);
                        GPSdate = GPSDate{day, month, year};
                    },
                    _ => (),
                }
            }
        } 
        RMC {
            valid: validity, 
            date: GPSdate,
        }
    }
    pub fn parse_gsa(&self, data: &[u8]) -> GSA {
        let mut fixmode = FixMode::NoFix;
        let mut sat_ids = [0u8; 12];
        let mut hdop= 0.0f32;
        let mut vdop= 0.0f32;
        let mut pdop= 0.0f32;

        for (i, field) in data.split(|c| *c == b',').enumerate() {
            if field.len() > 0 {
                match i {
                    // Mode 1
                    0 => {

                    },
                    // Mode 2 - FixMode
                    1 => {
                        fixmode = match atoi(&field) {
                            2 => FixMode::D2,
                            3 => FixMode::D3,
                            _ => FixMode::NoFix,
                        };
                    },
                    // IDs of sattelites in use
                    2..=12 => {
                        let id = atoi(&field) as u8;
                        sat_ids[i-2] = id;
                    },
                    // Position Dilution of Precision
                    13 => {
                        let (int, fract) = (field[0] as u32, field[3] as u32);
                        pdop = int as f32 + (fract as f32)/10.0;
                    },
                    // Horizontal Dilution of Precision
                    14 => {
                        let (int, fract) = (field[0] as u32, field[3] as u32);
                        hdop = int as f32 + (fract as f32)/10.0;
                    },
                    // Vertical Dilution of Precision
                    15 => {
                        let (int, fract) = (field[0] as u32, field[3] as u32);
                        vdop = int as f32 + (fract as f32)/10.0;
                    },
                    _ => (),
                }
            }
        }
        GSA {
            hdop: hdop,
            vdop: vdop,
            pdop: pdop,
            fix: fixmode,
            satellite_id: sat_ids,
        }
    }
    pub fn parse_gga(&self, data: &[u8]) -> GGA {
        let mut GPStime = GPSTime::new();
        let mut pos = Position::new();
        let mut satellites = 0u8;
        let mut fix_mode = FixType::NoFix;

        for (i, field) in data.split(|c| *c == b',').enumerate() {
            if field.len() > 0 {
                match i {
                    // UTC time
                    0 => {

                    },
                    // Lattitude
                    1 => {
                        let lat = atoi(&field[4..]);
                        pos.lattitude = (lat as f32)/100.0;
                    },
                    // N/S indicator
                    2 => {
                        pos.ns_indicator = field[0] as char;
                    },
                    // Longitude
                    3 => {
                        let long = atoi(&field[4..]);
                        pos.longitude = (long as f32)/100.0;
                    },
                    // E/W indicator
                    4 => {
                        pos.ew_indicator = field[0] as char;
                    },
                    // FIX
                    5 => {
                        fix_mode = match atoi(&field) {
                            0 => FixType::NoFix,
                            1 => FixType::GPSFix,
                            2 => FixType::DifferentialFix,
                            _ => FixType::NoFix,
                        };
                    },
                    // Satellites used
                    6 => {
                        satellites = atoi(&field) as u8;
                    },
                    // Altitude 
                    8 => {
                        let (alt_int, alt_frac) = (atoi(&field[..2]), field[4]);
                        pos.altitude = alt_int as f32 + (alt_frac as f32)/10.0
                    },
                    _ => (),
                }
            }
        }
        GGA {
            time: GPStime, 
            position: pos,
            satellites_used: satellites,
            fix: fix_mode,
        }

    }
}
