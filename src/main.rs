#![feature(int_roundings)]

use serialport::{TTYPort, SerialPort};
use std::io::{Read, Write, ErrorKind::TimedOut};
use std::time::{Duration, Instant};
use std::fs;
use clap::{Parser, Subcommand};
use clap_num::maybe_hex;
use utils::*;

mod utils;

#[derive(Debug, Parser)]
struct App {
    #[arg(short, default_value_t = String::from("/dev/ttyUSB0"))]
    port: String,
    
    #[clap(subcommand)]
    cmd: Command
}

#[derive(Debug, Subcommand, Clone)]
enum Command {
    Write {
        #[arg(short)]
        file: String,
        #[arg(short, default_value_t = 0, value_parser=maybe_hex::<usize>)]
        offset: usize
    },
    Read {
        #[arg(short)]
        file: String,
        #[arg(short, default_value_t = 0, value_parser=maybe_hex::<usize>)]
        offset: usize,
        #[arg(short, default_value_t = FLASH_SIZE, value_parser=maybe_hex::<usize>)]
        length: usize
    }
}

const FLASH_SIZE: usize = 2usize.pow(17);

fn main() {
    let app = App::parse();
    let cmd = app.cmd;

    let port = serialport::new(&app.port, 9600)
        .timeout(Duration::from_millis(100))
        .open_native()
        .expect("failed to open port");
    let mut pgm = Programmer::new(port);

    //test(&mut pgm);
    //return;
    
    let start = Instant::now();
    match cmd {
        Command::Write { file, offset } => {
            if offset & 0xfff != 0 {
                println!("offset must be a multiple of 4096");
                return
            }
            let d = fs::read(file).expect("could not read source file");
            if offset + d.len() > FLASH_SIZE {
                println!("file too big :(");
                return
            }
            for (i, chunk) in d.chunks(4096).enumerate() {
                eprintln!("writing sector {} of {}", i, d.len().div_ceil(4096));
                pgm.overwrite_sector(((i * 4096) + offset) as u32, chunk).expect("could not write sector")
            }
            let dur = Instant::now().duration_since(start);
            eprintln!("wrote {} bytes in {}ms", d.len(), dur.as_millis())
        }
        Command::Read { file, offset, length } => {
            if offset + length > FLASH_SIZE {
                println!("requested read too big :(");
                return
            }
            let mut f = fs::OpenOptions::new().write(true).create(true).truncate(true).open(file).expect("could not open destination for writing");
            let v = pgm.dump(offset as u32, length).expect("something went wrong :(");
            let dur = Instant::now().duration_since(start);
            eprintln!("read {} bytes in {}ms", v.len(), dur.as_millis());
            f.write(&v).expect("could not write file");
        }
    }
}

struct Programmer {
    port: TTYPort,
}
impl Programmer {
    fn new(port: TTYPort) -> Self {
        Self { port }
    }

    fn enter_long_write_mode(&mut self) -> Result<()> {
        self.send_cmd_no_data(b"l\n")
    }
    fn enter_long_read_mode(&mut self) -> Result<()> {
        self.send_cmd_no_data(b"L\n")
    }
    fn exit_long_mode(&mut self) -> Result<()> {
        std::thread::sleep(Duration::from_millis(100));
        self.send_cmd_no_data(&[]) // confirm that the programmer has left long mode
    }

    fn long_write(&mut self, data: &[u8]) -> Result<()> {
        if data.len() != 16 {
            return Err(PgmErr::WrongDataLength)
        }
        self.send_cmd_no_data(data)
    }
    fn long_read(&mut self) -> Result<[u8; 16]> {
        self.port.write(&[b'!'])?;
        let mut buf = [0; 16];
        loop {
            match self.port.read_exact(&mut buf) {
                Ok(len) => break len,
                Err(e) if e.kind() == TimedOut => {
                    //println!("timeout");
                    continue
                },
                Err(e) => return Err(e.into())
            }
        };
        Ok(buf)
    }

    fn overwrite_sector(&mut self, address: u32, data: &[u8]) -> Result<()> {
        if address & 0xfff != 0 {
            return Err(PgmErr::SectorMisaligned)
        }
        self.erase_sector(address)?;
        self.enter_long_write_mode()?;
        
        let iter = data.chunks_exact(16);
        let rem = iter.remainder();
        for (i, block) in iter.enumerate() {
            if (i * 16) % 1024 == 0 {
                eprintln!("byte {} of 4096", i * 16);
            }
            self.long_write(block)?
        }
        self.exit_long_mode()?;
        if rem.len() != 0 {
            eprintln!("finishing off ({} bytes)", rem.len());
            for (i, b) in rem.iter().enumerate() {
                eprint!(".");
                self.program_byte(address + (data.len() - rem.len() + i) as u32, *b)?
            }
            eprintln!()
        }
        Ok(())
    }

    fn dump_sector(&mut self, address: u32) -> Result<Vec<u8>> {
        if address & 0xfff != 0 {
            return Err(PgmErr::SectorMisaligned)
        }
        let mut v = Vec::with_capacity(4096);
        self.set_cursor(address)?;
        self.enter_long_read_mode()?;
        for i in (0..4096).step_by(16) {
            if i % 1024 == 0 {
                eprintln!("byte {} of 4096", i)
            }
            let d = self.long_read()?;
            v.extend(d.iter())
        }
        self.exit_long_mode()?;
        Ok(v)
    }
    fn dump(&mut self, address: u32, len: usize) -> Result<Vec<u8>> {
        let mut v = Vec::with_capacity(len);
        self.set_cursor(address)?;
        eprintln!("set cursor");
        self.enter_long_read_mode()?;
        eprintln!("in long read mode");
        for i in (0..len).step_by(16) {
            if i % 1024 == 0 {
                eprintln!("byte {} of {}", i, len)
            }
            let d = self.long_read()?;
            v.extend(d.iter())
        }
        v.truncate(len);
        self.exit_long_mode()?;
        Ok(v)
    }

    fn read_byte(&mut self, address: u32) -> Result<u8> {
        let mut cmd = [b'r', b' ', 0, 0, 0, 0, 0, b'\n'];
        write_20b_hex_to_buffer(&mut cmd[2..7], address);
        //println!("sending command");
        let reply = self.send_cmd(&cmd)?;
        if reply.len() != 4 {
            dbg!(reply);
            Err(PgmErr::ExpectedNum)
        }
        else {
            std::str::from_utf8(&reply[1..3])
                .map_err(|_| PgmErr::InvalidNum)
                .and_then(|s| u8::from_str_radix(s, 16).map_err(|_| PgmErr::InvalidNum))
        }
    }

    fn program_byte(&mut self, address: u32, data: u8) -> Result<()> {
        let mut cmd = [b'w', b' ', 0, 0, 0, 0, 0, b' ', 0, 0, b'\n'];
        write_20b_hex_to_buffer(&mut cmd[2..7], address);
        write_8b_hex_to_buffer(&mut cmd[8..10], data);
        self.send_cmd_no_data(&cmd)
    }

    fn erase_sector(&mut self, address: u32) -> Result<()> {
        let mut cmd = [b'e', b' ', 0, 0, 0, 0, 0, b'\n'];
        if address & 0xfff != 0 {
            return Err(PgmErr::SectorMisaligned)
        }
        write_20b_hex_to_buffer(&mut cmd[2..7], address);
        self.send_cmd_no_data(&cmd)?;
        Ok(())
    }
    fn set_cursor(&mut self, address: u32) -> Result<()> {
        let mut cmd = [b'c', b' ', 0, 0, 0, 0, 0, b'\n'];
        if address & 0xfff != 0 {
            return Err(PgmErr::SectorMisaligned)
        }
        write_20b_hex_to_buffer(&mut cmd[2..7], address);
        self.send_cmd_no_data(&cmd)?;
        Ok(())
    }

    fn send_cmd(&mut self, cmd: &[u8]) -> Result<Vec<u8>> {
        self.port.write(cmd)?;
        let mut buf = vec![0; 16];
        let len = loop {
            match self.port.read(&mut buf) {
                Ok(len) => break len,
                Err(e) if e.kind() == TimedOut => {
                    println!("timeout");
                    continue
                },
                Err(e) => return Err(e.into())
            }
        };
        buf.truncate(len);
        if buf[0] != b'!' {
            return Err(PgmErr::Other)
        }
        Ok(buf)
    }
    fn send_cmd_no_data(&mut self, cmd: &[u8]) -> Result<()> {
        self.port.write(cmd)?;
        let mut buf = [0; 2];
        loop {
            match self.port.read_exact(&mut buf) {
                Ok(()) => break,
                Err(e) if e.kind() == TimedOut => {
                    println!("timeout");
                    continue
                },
                Err(e) => return Err(e.into())
            }
        }
        if buf[1] != b'\n' {
            dbg!(buf);
            self.port.clear(serialport::ClearBuffer::Input).expect("failed to clear buffer");
            return Err(PgmErr::UnexpectedData)
        }
        if buf[0] == b'!' { Ok(()) }
        else {
            Err(PgmErr::Other)
        }

    }
}

type Result<T> = std::result::Result<T, PgmErr>;
#[derive(Debug)]
enum PgmErr {
    /// expected hex number, found none
    ExpectedNum,
    InvalidNum,
    /// expected nothing, got something
    UnexpectedData,
    SectorMisaligned,
    WrongDataLength,
    /// something happened inside the programmer and we don't know what
    Other,
    IoErr(std::io::Error)
}
impl From<std::io::Error> for PgmErr {
    fn from(value: std::io::Error) -> Self {
        Self::IoErr(value)
    }
}

