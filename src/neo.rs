extern crate embedded_hal;
use embedded_hal::serial::{Write};
use nb::block;

use crate::stm32::{USART1, USART2, USART3};
use crate::serial::{Rx1, Tx1, Rx2, Tx2, Rx3, Tx3};
use crate::serial::Event as SEvent;

use core::fmt;

pub fn atoi(barray: & [u8]) -> u32 {
    let mut value = 0u32;
    for element in barray.iter() {
        value = value*10 + *element as u32 - b'0' as u32;
    }
    value
}

#[derive(Debug, Copy, Clone)]
pub struct GPSFloat {
    pub int: u32,
    pub fract: u32,
}

impl fmt::Display for GPSFloat {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result{
        write!(f, "{}.{}", self.int, self.fract)
    }
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

#[derive(Debug, Copy, Clone)]
pub struct GPSTime {
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
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

impl fmt::Display for GPSTime {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result{
        write!(f, "{}:{}:{}", self.hour, self.minute, self.second)
    }
}


#[derive(Debug, Copy, Clone)]
pub struct GPSDate {
    pub day: u8,
    pub month: u8,
    pub year: u8,
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


impl fmt::Display for GPSDate {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result{
        write!(f, "{}/{}/{}", self.day, self.month, self.year)
    }
}



#[derive(Debug, Copy, Clone)]
pub struct Position {
    lattitude: GPSFloat,
    ns_indicator: char,
    longitude: GPSFloat,
    ew_indicator: char,
    altitude: GPSFloat,
}

impl Position {
    pub fn new() -> Self {
        Position {
            lattitude: GPSFloat{int:0, fract:0},
            ns_indicator: 'N',
            longitude: GPSFloat{int:0, fract:0},
            ew_indicator: 'E',
            altitude: GPSFloat{int:0, fract:0},
        }
    }
}


impl fmt::Display for Position {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result{
        write!(f, "Latt: {} {}; Long: {} {}; Alt: {} m", self.lattitude,  self.ns_indicator, self.longitude, self.ew_indicator, self.altitude)
    }
}


#[derive(Debug)]
pub struct GPSSatellite {
    ID: u8,
    elevation: u8,
    azimuth: u16,
    SNR: u8,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum FixType {
    NoFix,
    GPSFix,
    DifferentialFix,
}

#[derive(Debug, Copy, Clone, PartialEq)]
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
    hdop: GPSFloat,
    vdop: GPSFloat,
    pdop: GPSFloat,
    fix: FixMode,
    satellite_ids: [u8; 12],
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
        self.len -= 1;
    }
    pub fn is_full(&self) -> bool {
        if self.len == 2 { return true} else {return false};
    }
    pub fn is_empty(&self) -> bool {
        if self.len == 0 { return true} else {return false};
    }


    pub fn get_line(&self) -> (GPS_Statement, &[u8]) {
        if self.len > 0 {
            let mut data = match self.last_read {
                Half::First => &self.buffer[..self.cap],
                Half::Second => &self.buffer[self.cap..],
            };
            let mut msg = data.splitn(2, |c| *c == 0x2C);       // 0d44 split at ,
            let (cmd, info) = (msg.next().unwrap(), msg.next().unwrap());
            let (cmd, info) = match cmd {
                b"$GPRMC" => { (GPS_Statement::GPRMC, info) },
                b"$GPGSA" => { (GPS_Statement::GPGSA, info) },
                b"$GPGGA" => { (GPS_Statement::GPGGA, info) },
            //     // b"$GPGSV" => {},
                _ => { (GPS_Statement::Other, info) },

            };
            return (cmd, info);
        }
        (GPS_Statement::Other, &[0u8; 1])
    }
}

pub struct NEO6 <'a, Rx, Tx> {
    rx: Rx,
    tx: Tx,
    buffer: MSG<'a>,
    gps_data: GPS_Data,
}

pub trait New<'a, Rx, Tx> {
    fn new(buf: &'a mut [u8], rx: Rx, tx: Tx) -> NEO6<Rx, Tx>;
}

impl<'a, Rx, Tx> New<'a, Rx, Tx> for NEO6<'a, Rx, Tx> {
    fn new(buf: &'a mut [u8], rx: Rx, tx: Tx) -> Self {
        let buf_len =buf.len();
        NEO6 {
            rx: rx,
            tx: tx,
            buffer: MSG::new(buf, buf_len/2),
            gps_data: GPS_Data::new(),
        }
    }
}

macro_rules! neo {
    ($(
        $(#[$meta:meta])*
        NEO6: (
            $USARTX:ident,
            $rxX: ident,
            $txX: ident,
        ),
    )+) => {
        $(
            $(#[$meta])*
            impl <'a> NEO6 <'a, $rxX, $txX> {
                pub fn listen(&mut self) {
                    self.rx.listen(crate::serial::Event::Rxne);
                }
                pub fn unlisten(&mut self) {
                    self.rx.unlisten(crate::serial::Event::Rxne);
                }
                pub fn data_valid(&self) -> bool {
                    self.gps_data.is_valid() && self.gps_data.get_fix_type() != FixType::NoFix && self.gps_data.get_fix_mode() != FixMode::NoFix
                }
                pub fn receive(&mut self) {
                    let mut i = 0;
                    if let Some(ev) = self.rx.which_event() {
                        match ev {
                            SEvent::Rxne => {
                                self.rx.clear_event();
                                if !self.buffer.is_full() {
                                    let a = unsafe { (*$USARTX::ptr()).dr.read().bits() as u8};            
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
                pub fn get_line(&self) -> (GPS_Statement, &[u8]) {
                    self.buffer.get_line()
                }
                pub fn buffer_is_empty(&self) -> bool {
                    self.buffer.is_empty()
                }
                pub fn clear_buffer(&mut self) {
                    self.buffer.clear();
                }
                pub fn parse(&mut self) {
                    if !self.buffer.is_empty() {
                        let (mut cmd, mut info) = self.get_line();
                        match cmd {
                                GPS_Statement::GPRMC => { 
                                    let rmc_data = self.parse_rmc(info);
                                    self.gps_data.update_rmc(rmc_data);
                                },
                                GPS_Statement::GPGSA => {
                                    let gsa_data = self.parse_gsa(info);
                                    self.gps_data.update_gsa(gsa_data);
                                },
                                GPS_Statement::GPGGA => {
                                    let gga_data = self.parse_gga(info);
                                    self.gps_data.update_gga(gga_data);
                                },
                                _ => (),
            
                        }
                    
                        self.buffer.clear();
                    }
                }
                pub fn parse_rmc(&self, data: &[u8]) -> RMC {
                    let mut gpsdate = GPSDate::new();
                    let mut validity = false;
            
                    for (i, field) in data.split(|c| *c == b',').enumerate() {
                        if field.len() > 0 {
                            match i {
                                // GPSTIME
                                0 => {
                                    // let (hour, minute, seconds) = (atoi(&field[..2]) as u8, atoi(&field[2..4]) as u8, atoi(& field[4..6]) as u8);
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
                                    let (day, month, year) = (atoi(&field[..2]) as u8, atoi(&field[2..4]) as u8, atoi(&field[4..6]) as u8);
                                    gpsdate = GPSDate{day, month, year};
                                },
                                _ => (),
                            }
                        }
                    } 
                    RMC {
                        valid: validity, 
                        date: gpsdate,
                    }
                }
                pub fn parse_gsa(&self, data: &[u8]) -> GSA {
                    let mut fixmode = FixMode::NoFix;
                    let mut sat_ids = [0u8; 12];
                    let mut hdop= GPSFloat{int:0, fract:0};
                    let mut vdop= GPSFloat{int:0, fract:0};
                    let mut pdop= GPSFloat{int:0, fract:0};
            
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
                                    let (int, fract) = (field[0] as u32, field[2] as u32);
                                    pdop = GPSFloat{int:int, fract:fract};
                                },
                                // Horizontal Dilution of Precision
                                14 => {
                                    let (int, fract) = (field[0] as u32, field[2] as u32);
                                    hdop = GPSFloat{int:int, fract:fract};
                                },
                                // Vertical Dilution of Precision
                                15 => {
                                    let (int, fract) = (field[0] as u32, field[2] as u32);
                                    vdop = GPSFloat{int:int, fract:fract};
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
                        satellite_ids: sat_ids,
                    }
                }
                pub fn parse_gga(&self, data: &[u8]) -> GGA {
                    let mut gpstime = GPSTime::new();
                    let mut pos = Position::new();
                    let mut satellites = 0u8;
                    let mut fix_mode = FixType::NoFix;
            
                    for (i, field) in data.split(|c| *c == b',').enumerate() {
                        if field.len() > 0 {
                            match i {
                                // UTC time
                                0 => {
                                    let (hour, minute, seconds) = (atoi(&field[..2]) as u8, atoi(&field[2..4]) as u8, atoi(& field[4..6]) as u8);
                                    gpstime.hour = hour;
                                    gpstime.minute = minute;
                                    gpstime.second = seconds;
                                },
                                // Lattitude
                                1 => {
                                    let (lat_int, lat_fract) = (atoi(&field[..2]), atoi(&field[2..4]));
                                    pos.lattitude = GPSFloat{int:lat_int, fract:lat_fract};        
                                },
                                // N/S indicator
                                2 => {
                                    pos.ns_indicator = field[0] as char;
                                },
                                // Longitude
                                3 => {
                                    let (long_int, long_fract) = (atoi(&field[..2]), atoi(&field[2..4]));
                                    pos.longitude = GPSFloat{int: long_int, fract:long_fract};
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
                                    let mut alt_iter = field.split(|c| *c == b'.');
                                    let (alt_int, alt_frac) = (atoi(alt_iter.next().unwrap()), atoi(alt_iter.next().unwrap()));
                                    pos.altitude = GPSFloat{int:alt_int, fract:alt_frac as u32};
                                    // pos.altitude = alt_int as f32 + (alt_frac as f32)/10.0
                                },
                                _ => (),
                            }
                        }
                    }

                    GGA {
                        time: gpstime, 
                        position: pos,
                        satellites_used: satellites,
                        fix: fix_mode,
                    }
            
                }
                pub fn get_data(&self) -> GPS_Data {
                    self.gps_data
                }
                pub fn report(&mut self) {
                    use core::fmt::Write;
                    write!(self.tx, "{}\n", self.gps_data.get_time());
                    write!(self.tx, "{}\n", self.gps_data.get_date());
                    write!(self.tx, "{}\n", self.gps_data.get_position());

                }
            }
        )+
        
    }
}


#[derive(Debug, Copy, Clone)]
pub struct GPS_Data {

    // from RMC
    valid: bool,
    date: GPSDate,
    // from GGA
    position: Position,
    satellites_used: u8,
    fix: FixType,
    time: GPSTime,
    // from GSA
    hdop: GPSFloat,
    vdop: GPSFloat,
    pdop: GPSFloat,
    fix_mode: FixMode,
    satellite_ids: [u8; 12],
}

impl GPS_Data {
    pub fn new() -> Self {
        GPS_Data { 
            valid: false,
            date: GPSDate::new(),
            // from GGA
            position: Position::new(),
            satellites_used: 0,
            fix: FixType::NoFix,
            time: GPSTime::new(),
            // from GSA
            hdop: GPSFloat{int:0, fract:0},
            vdop: GPSFloat{int:0, fract:0},
            pdop: GPSFloat{int:0, fract:0},
            fix_mode: FixMode::NoFix,
            satellite_ids: [0u8; 12],
        }
    }
    pub fn is_valid(&self) -> bool {
        self.valid
    }
    pub fn get_position(&self) -> Position {
        self.position
    }
    pub fn get_time(&self) -> GPSTime {
        self.time
    }
    pub fn get_date(&self) -> GPSDate {
        self.date
    }
    pub fn satellites_no(&self) -> u8 {
        self.satellites_used
    }
    pub fn get_fix_type(&self) -> FixType {
        self.fix
    }
    pub fn get_fix_mode(&self) -> FixMode {
        self.fix_mode
    }
    pub fn update_rmc (&mut self, data: RMC) {
        self.valid = data.valid;
        self.date = data.date;
    }
    pub fn update_gga (&mut self, data: GGA) {
        self.position = data.position;
        self.satellites_used = data.satellites_used;
        self.fix = data.fix;
        self.time = data.time;
    }
    pub fn update_gsa (&mut self, data: GSA) {
        self.hdop = data.hdop;
        self.vdop = data.vdop;
        self.pdop = data.pdop;
        self.fix_mode = data.fix;
        self.satellite_ids = data.satellite_ids;
    }
}

neo! {
    NEO6: (
        USART1,
        Rx1,
        Tx1,
    ),
    NEO6: (
        USART2,
        Rx2,
        Tx2,
    ),
    NEO6: (
        USART3,     
        Rx3,
        Tx3,

    ),
    NEO6: (
        USART3,     
        Rx3,
        Tx1,

    ),
}