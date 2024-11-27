/// writes the least significant 20 bits, in hex, to a buffer, starting at 0
pub fn write_20b_hex_to_buffer(buf: &mut [u8], num: u32) {
    for i in 0..5 {
        let shift = (4 - i) * 4;
        buf[i] = num_to_hex_digit((num >> shift) as u8 & 0xf)
    }
}

pub fn write_8b_hex_to_buffer(buf: &mut [u8], num: u8) {
    buf[1] = num_to_hex_digit(num & 0xf);
    buf[0] = num_to_hex_digit(num >> 4)
}

pub fn num_to_hex_digit(num: u8) -> u8 {
    if num < 0xa {
        num + b'0'
    }
    else {
        num + (b'A' - 10)
    }
}

#[test]
fn test_8b_hex() {
    let mut buf = [0; 2];

    write_8b_hex_to_buffer(&mut buf, 0xaa);
    assert_eq!(buf, [b'A', b'A'])
}

#[test]
fn test_num_to_hex() {
    assert_eq!(num_to_hex_digit(0), b'0');
    assert_eq!(num_to_hex_digit(1), b'1');
    assert_eq!(num_to_hex_digit(2), b'2');
    assert_eq!(num_to_hex_digit(3), b'3');
    assert_eq!(num_to_hex_digit(4), b'4');
    assert_eq!(num_to_hex_digit(5), b'5');
    assert_eq!(num_to_hex_digit(6), b'6');
    assert_eq!(num_to_hex_digit(7), b'7');
    assert_eq!(num_to_hex_digit(8), b'8');
    assert_eq!(num_to_hex_digit(9), b'9');
    assert_eq!(num_to_hex_digit(0xA), b'A');
    assert_eq!(num_to_hex_digit(0xB), b'B');
    assert_eq!(num_to_hex_digit(0xC), b'C');
    assert_eq!(num_to_hex_digit(0xD), b'D');
    assert_eq!(num_to_hex_digit(0xE), b'E');
    assert_eq!(num_to_hex_digit(0xF), b'F');
}
