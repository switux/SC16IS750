/*
   SC16IS1750 UART Bridge driver
   
   (c) 2023 Lesage F. 

	Revision history
		1.0.0	: Barebone version, I2C only, polled mode operation, made to work with the AstroWeatherStation's GPS.

   	This program is free software: you can redistribute it and/or modify it
	under the terms of the GNU General Public License as published by the
	Free Software Foundation, either version 3 of the License, or (at your option)
	any later version.

	This program is distributed in the hope that it will be useful, but
	WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
	or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
	more details.

	You should have received a copy of the GNU General Public License along
	with this program. If not, see <https://www.gnu.org/licenses/>.

*/

/*
 * Used the following documents
 * 		Product data sheet: https://www.nxp.com/docs/en/data-sheet/SC16IS740_750_760.pdf
 * 		
 */
 
#include <Arduino.h>
#include <Wire.h>
#include "SC16IS750.h"

SC16IS750::SC16IS750( bool i2c_protocol, uint8_t _address )
{
	if ( !i2c_protocol ) {

		Serial.printf( "[ERROR] Only I2C protocol is supported. Setting address to 0x00.\n" );
		address = 0;
	}
	address = _address >> 1;		// 8 bits addressing (address > 0x77, see Table 32 of data sheet), we have to drop the R/W bit
	has_peek = 0;
	peek_byte = -1;
}

int SC16IS750::available( void )
{
    return available_data_in_FIFO();
}

bool SC16IS750::available_data_in_FIFO( void )
{
	return (( read_register( SC16IS750_LSR ) & LSR_DATA_AVAILABLE ));	// Polled mode operation
}

bool SC16IS750::begin( uint32_t baudrate )
{
	if ( !address ) {

		Serial.printf( "[ERROR] address = 0x00, bailing out.\n" );
		return false;
	}
	
	Wire.begin();
	
	if ( !test() )
		goto fail;
	if ( !toggle_sleep( false ))
		goto fail;
	if ( !FIFO_reset( true, true ))
		goto fail;
	if ( !FIFO_enable( true ))
		goto fail;
	if ( !enable_extra_features())
		goto fail;
	if ( !enable_TCR_TLR())
		goto fail;
	if ( !set_flow_control_levels( 48, 24 ))
		goto fail;
	if ( !enable_TX_RX())
		goto fail;
	if ( !set_line( 8, 0, 1 ))
		goto fail;
	if ( !set_baudrate( baudrate ))
		goto fail;

    return true;

fail:

	Wire.end();
	return false;
}

bool SC16IS750::enable_extra_features( void )
{
	uint8_t	tmp,
			tmp2;

	tmp = read_register( SC16IS750_LCR );
	if ( !write_register( SC16IS750_LCR, LCR_ACCESS_EFR ))
		return false;
	
	tmp2 = read_register( SC16IS750_EFR );
	if ( !write_register( SC16IS750_EFR, tmp2 | EFR_ENHANCED_FUNC_ENABLE ))
		return false;

	return write_register( SC16IS750_LCR, tmp );
}

bool SC16IS750::enable_TCR_TLR( void )
{
	if ( !enable_extra_features())
		return false;

	uint8_t	tmp = read_register( SC16IS750_MCR );

	return write_register( SC16IS750_MCR, tmp | MCR_TCR_TLR_ENABLE );
}

bool SC16IS750::enable_TX_RX( void )
{
	uint8_t tmp = read_register( SC16IS750_EFCR );

	return write_register( SC16IS750_EFCR, tmp & (~( EFCR_RX_DISABLE | EFCR_TX_DISABLE )) );
}

bool SC16IS750::FIFO_enable( bool enable_fifo )
{
	uint8_t tmp = read_register( SC16IS750_FCR );

	if ( enable_fifo )
		return write_register( SC16IS750_FCR, tmp | FCR_FIFO_ENABLE );
	else
		return write_register( SC16IS750_FCR, tmp & (~FCR_FIFO_ENABLE) );  
}

bool SC16IS750::FIFO_reset( bool rx, bool tx )
{
     uint8_t tmp = read_register( SC16IS750_FCR );

    if ( rx )
        tmp |= FCR_FIFO_RX_RESET;
    if ( tx )
        tmp |= FCR_FIFO_TX_RESET;
  
    return write_register( SC16IS750_FCR, tmp );
}

bool SC16IS750::flush()
{
    if ( !FIFO_reset( true, true ))
    	return false;
	return FIFO_enable( false );
}

int SC16IS750:: peek()
{
	if ( has_peek )
		return peek_byte;

	// FIXME: set a mutex?
	peek_byte = read_byte();
	has_peek = ( peek_byte >= 0 );
	return peek_byte;
}

int8_t SC16IS750::read( void )
{
	return read_byte();
}

int8_t SC16IS750::read_byte( void )
{
	return read_register( SC16IS750_RHR );
}

int8_t SC16IS750::read_register( uint8_t register_address )
{
    uint8_t n;

	Wire.beginTransmission( address );
	Wire.write( ( register_address << SC16IS750_REG_ADDR_SHIFT ));	// Product sheet, table 33. Register address byte: use bits 3-6 and bits 2&1 must be 0, bit 0 is not used.
	switch( Wire.endTransmission( 0 )) {
		case 0:
			break;
		case 1:
			Serial.printf("ERROR: SC16IS750 transmission failed: data too long to fit in buffer\n" );
			break;
		case 2:
			Serial.printf("ERROR: SC16IS750 transmission failed: NACK on transmit of address\n" );
			break;
		case 3:
			Serial.printf("ERROR: SC16IS750 transmission failed: NACK on transmit of data\n" );
			break;
		case 4:
			Serial.printf("ERROR: SC16IS750 transmission failed: other error\n" );
			break;
		case 5:
			Serial.printf("ERROR: SC16IS750 transmission failed: timeout\n" );
			break;
	}
	if ( ( n = Wire.requestFrom( address, 1 )) != 1 )
		Serial.printf( "ERROR: SC16IS750 received %d bytes instead of 1\n", n );
	return Wire.read();
}

bool SC16IS750::set_baudrate( uint32_t baudrate )
{
    ulong		divisor;
    uint8_t		prescaler = 1,
				tmp1,
				tmp2;
	bool		x;
	
	divisor = SC16IS750_XTAL_FREQ / ( 16 * baudrate );
	if ( divisor > 0xffff ) {
		
		prescaler = 4;		// since DLL/DLH are both 8 bits, anything > 0xffff needs to be further divided
		divisor /= 4;
	}

	tmp1 = read_register( SC16IS750_LCR );
	if ( !write_register( SC16IS750_LCR, tmp1 | LCR_DIVISOR_LATCH_ENABLE )) 	// DLL & DLH are available only if LCR[7] is set
		return false;
		
    x = write_register( SC16IS750_DLL, (uint8_t)(divisor & 0xFF) );
    x &= write_register( SC16IS750_DLH, (uint8_t)(divisor >> 8) );

    x &= write_register( SC16IS750_LCR, tmp1 );

	tmp1 = read_register( SC16IS750_EFR );
	x &= write_register( SC16IS750_EFR, EFR_ENHANCED_FUNC_ENABLE );		// MCR[7] can be changed if EFR[4] is set
	
	tmp2 = read_register( SC16IS750_MCR );
	if ( prescaler  == 4 )
		x &= write_register( SC16IS750_MCR, tmp2 | MCR_CLOCK_DIVISOR );
	else
		x &= write_register( SC16IS750_MCR, tmp2 & (~MCR_CLOCK_DIVISOR ) );

	return ( x & write_register( SC16IS750_EFR, tmp1 ));
}

bool SC16IS750::set_flow_control_levels( uint8_t halt_level, uint8_t resume_level )
{
	if ( halt_level <= resume_level )
		return false;
		
	uint8_t val = (( halt_level / 4 ) & 0x0F ) + (( (resume_level / 4) & 0x0F ) << 4 );
	return write_register( SC16IS750_TCR, val );
}

bool SC16IS750::set_line( uint8_t word_length, uint8_t parity_type, uint8_t stop_bits )
{
    uint8_t tmp = read_register( SC16IS750_LCR );
    
    tmp &= ~( LCR_WORD_LENGTH0 | LCR_WORD_LENGTH1 | LCR_STOP_BITS_15_OR_2 | LCR_PARITY_ENABLE | LCR_PARITY_EVEN | LCR_PARITY_FORCE );
    
	switch ( word_length ) {           
		case 5:
			tmp |= LCR_WORD_LENGTH_5;
			break;
		case 6:
			tmp |= LCR_WORD_LENGTH_6;
			break;
		case 7:
			tmp |= LCR_WORD_LENGTH_7;
			break;
		case 8:
			tmp |= LCR_WORD_LENGTH_8;
			break;
		default:
			tmp |= LCR_WORD_LENGTH_8;
			break;
	}

	if ( stop_bits == 2 )
		tmp |= LCR_STOP_BITS_15_OR_2;

	switch ( parity_type ) {
		case NO_PARITY:
			break;
		case ODD_PARITY:
			tmp |= SET_LCR_PARITY_ODD;
			break;
		case EVEN_PARITY:
			tmp |= SET_LCR_PARITY_EVEN;
			break;
		case FORCE_1_PARITY:
			tmp |= SET_LCR_PARITY_1;
            break;
		case FORCE_0_PARITY:
			tmp |= SET_LCR_PARITY_0;
            break;
		default:
			break;
	}

    return write_register( SC16IS750_LCR, tmp );
}

bool SC16IS750::test( void )
{
	write_register( SC16IS750_SPR, 0x41 );
	return ( read_register( SC16IS750_SPR ) == 0x41 );
}

bool SC16IS750::toggle_sleep( bool sleep )
{
	uint8_t	tmp = read_register( SC16IS750_IER );

	if ( sleep )
		return write_register( SC16IS750_IER, tmp | IER_SLEEP_MODE_ENABLE );
	else
		return write_register( SC16IS750_IER, tmp & (~IER_SLEEP_MODE_ENABLE) );
}

uint8_t SC16IS750::write( uint8_t b )
{
    return write_byte( b );
}

uint8_t SC16IS750::write_byte( uint8_t b )
{
	while (( read_register( SC16IS750_LSR ) & LSR_OVERRUN_ERROR )) { };

	return write_register( SC16IS750_THR, b );
}

bool SC16IS750::write_register (uint8_t register_address, uint8_t value )
{
	Wire.beginTransmission( address );
	Wire.write( ( register_address << SC16IS750_REG_ADDR_SHIFT ));	// Product sheet, table 33. Register address byte: use bits 3-6 and bits 2&1 must be 0, bit 0 is not used.
	Wire.write( value );
	switch ( Wire.endTransmission( 1 )) {
		case 0:
			return true;
			break;
		case 1:
			Serial.printf("ERROR: SC16IS750 transmission failed: data to long to fit in buffer\n" );
			break;
		case 2:
			Serial.printf("ERROR: SC16IS750 transmission failed: NACK on transmit of address\n" );
			break;
		case 3:
			Serial.printf("ERROR: SC16IS750 transmission failed: NACK on transmit of data\n" );
			break;
		case 4:
			Serial.printf("ERROR: SC16IS750 transmission failed: other error\n" );
			break;
		case 5:
			Serial.printf("ERROR: SC16IS750 transmission failed: timeout\n" );
			break;
	}
    return false;
}


//
// GPIO handling
//

bool SC16IS750::set_IO_direction( uint8_t pin, bool write )
{
	uint8_t tmp = read_register( SC16IS750_IODIR );

	if ( write )
		return write_register( SC16IS750_IODIR, tmp  | ( 1 << pin )  );
	else
		return write_register( SC16IS750_IODIR, tmp  & ~( 1 << pin )  );
}

bool SC16IS750::pin_mode( uint8_t pin, bool write )
{
	return set_IO_direction( pin, write ); 
}

uint8_t SC16IS750::get_IO_state( void )
{
	return read_register( SC16IS750_IOSTATE );
}

bool SC16IS750::read_GPIO( uint8_t pin )
{
	uint8_t pinStates = get_IO_state();

	if ( pin > 7 )
		return false;

	return (( pinStates & (1 << pin) ) != 0 );
}

bool SC16IS750::setup_GPIO_interrupt( uint8_t pin, bool enable )
{
	uint8_t tmp = read_register( SC16IS750_IOINTENA );

	if ( enable )
		return write_register( SC16IS750_IOINTENA, tmp | (1 << pin));
	else
		return write_register( SC16IS750_IOINTENA, tmp & ~(1 << pin));

}

bool SC16IS750::write_GPIO( uint8_t pin, bool high )
{
	uint8_t tmp = get_IO_state();

	if ( high )
		return write_register( SC16IS750_IOSTATE, tmp  | (1 << pin)  );
	else
		return write_register( SC16IS750_IOSTATE, tmp  & ~(1 << pin)  );
}
