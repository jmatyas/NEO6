#![no_std]

extern crate embedded_hal;
use embedded_hal::serial;
use nb::block;
use cortex_m_semihosting::{hprint, hprintln};


pub struct Date {
    hour: u8,
    minute: u8,
    second: u8,
    day: u8,
    month: u8,
    year: u8,
}

impl Date {
    pub fn new() -> Self {
        Date {
            hour: 0u8,
            minute: 0u8,
            second: 0u8,
            day: 0u8,
            month: 0u8,
            year: 0u8
        }
    }
}

pub struct Position {
    lattitude: f32,
    ns_indicator: char,
    longitude: f32,
    ew_indicator: char,
}

impl Position {
    pub fn new() -> Self {
        Position{
            lattitude: 0.0f32,
            ns_indicator: 'N',
            longitude: 0.0f32,
            ew_indicator: 'E',   
        } 
    }
}

pub struct NEO6 <'a, UART> {
    uart: &'a mut UART,
    
    date: Date,


    // lattitude: f64,
    // lattitude_drection: u8,
    // longitude: f64,
    // longitude_drection: u8,
    // altitude: f64,

    // speed_knots: f64,
    // speed_km: f64,

    // sat_num: u8,
    // quality: u8,
    // fix_mode: u8,
    
}

impl <'a, UART> NEO6 <'a, UART> {
    pub fn new (uart: &'a mut UART) -> NEO6<'a, UART> {
        let date = Date::new();
        NEO6 {uart, date}
    }
    pub fn init(&mut self) -> nb::Result<(), UART::Error> 
    where 
        UART: serial::Write<u8>
    {
        block!(self.uart.write(0xFF))?;
        Ok(())
    }
    pub fn read_char(&mut self) -> nb::Result<u8, UART::Error> 
    where
        UART: serial::Read<u8>,
    {
        Ok(self.uart.read()?)
    }
    pub fn read_lines(&mut self, buff: &mut [u8; 512], lines: u8) -> nb::Result<(), UART::Error>
    where
        UART: serial::Read<u8>
    {
        let mut curr_line = 0;
        let mut received = block!(self.uart.read())?;
        while received != 0x24 {
            received = block!(self.uart.read())?;
        }
        buff[0] = received;
        for byte in buff[1..].iter_mut() {
            let received =  block!(self.uart.read())?;
            *byte = received;
            if received == 0x0A {
                curr_line += 1;
                if curr_line == lines {
                    break;
                }
            }
                    
        }
        Ok(())
    }
    pub fn parse_info(&mut self) -> nb::Result<(), UART::Error>
    where
        UART: serial::Read<u8> 
    {
        let mut curr_line = 0;
        let mut commands = [[0u8; 81]; 6];
        let mut buffer = [0u8; 512];
        self.read_lines(&mut buffer, 6)?;
        let mut i=0;

        for byte in buffer.iter() {
            
            if *byte != 0x0A {
                commands[curr_line][i] = *byte;
                // hprint!("{}", *byte as char);
                i += 1;
                // hprint!("{}", commands[curr_line][i] as char);
            } else {                
                curr_line += 1;
                if curr_line == 6 {
                    break;
                }
                // hprint!("{} {} {}", *byte as char, i, curr_line);
                i = 0;
            };
        }
        
        // for cmd in commands.iter() {
        //     for ch in cmd.iter(){
        //         hprint!("{}", *ch as char);                
        //     }
        // }
        for cmd in commands.iter() {
            match &cmd[0..6] {
                b"$GPGLL" => {hprintln!("GLL"); ()},
                b"$GPRMC" => {self.parse_rmc(&cmd); ()},
                b"GPGSA" => (),
                b"GPVTG" => (),
                b"GPGGA" => (),
                b"GPGSV" => (),
                _ => ()
            }
        }

        Ok(())
    }
    pub fn parse_rmc(&mut self, cmd: &[u8; 81]) -> () {
        let mut date = Date::new();
        let mut position = Position::new();
        for (i, field) in cmd.split(|d| *d == b',').enumerate() { 
            if field.len() > 0 {
                match i {
                1 => {
                    let (hour, minute, seconds) = ((field[0] - 0x30)*10 + field[1] - 0x30 , (field[2] - 0x30)*10 + field[3] - 0x30, (field[4] - 0x30)*10 + field[5] -0x30);
                    date.hour = hour;
                    date.minute = minute;
                    date.second = seconds;
                },
                9 => {
                    let (day, month, year) = ((field[0] - 0x30)*10 + field[1] - 0x30 , (field[2] - 0x30)*10 + field[3] - 0x30, (field[4] - 0x30)*10 + field[5] -0x30);
                    date.day = day;
                    date.month = month;
                    date.year = year;
                }
                _ => ()
            }
        }
    //        for letter in field.iter(){
    //            hprint!("{:?}", *letter as char);
    //        }
    //        hprintln!("");
       }
       hprintln!("{}:{}:{}", date.hour, date.minute, date.second);
       hprintln!("{}.{}.{}", date.day, date.month, date.year);
       
    }
    pub fn parse_gga(&mut self, cmd: &[u8; 81]) -> () {
        
    }

}

