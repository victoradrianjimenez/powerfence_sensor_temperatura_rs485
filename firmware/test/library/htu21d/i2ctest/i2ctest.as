IoPin@  sdaPin = component.getPin("sda");
IoPin@  sclPin = component.getPin("scl");
IoPin@  txdPin = component.getPin("txd");

enum pinMode_t{
    undef_mode=0,
    input,
    openCo,
    output,
    source
};

enum iic_state_t {
	iic_idle = 0,
	iic_start,
	iic_stop,
	iic_write,
	iic_read,
	iic_getack,
	iic_setack,
	iic_nacksent
};

enum state_t
{
	s_idle = 0,
	s_command,
	s_data
}

enum seqstate_t
{
	seq_idle = 0,
	seq_command,
	seq_write,
	seq_read
}

bool	sclPrev, sdaPrev;
int		iic_state;
bool	iic_session;
bool	iic_ack;
bool	iic_rw;
bool	iic_inprogress;
uint8	iic_sbuf, iic_rbuf;
int		iic_bitcnt;
int		iic_delay = 5;

string	uart_buffer;
array<uint>	Sequence;
uint	seq_ptr;
bool	seq_err;
int		seq_state;
int		seq_counter;
bool	seq_session;

string converter = " !_#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";

uint8	devaddr;

void setup()
{
}

void reset()
{
	iic_state = iic_idle;
	iic_rw = false;
	iic_session = false;
	iic_inprogress = false;
	sclPrev = false;
	sdaPrev = false;
	
	sdaPin.setOutState( true );
	sclPin.setOutState( true );
	sdaPin.setPinMode( openCo );
	sclPin.setPinMode( openCo );
	txdPin.setOutState( true );
	txdPin.setPinMode( output );

	uart.setBaudRate( 9600 );
	uart_buffer = "";
	
	sclPin.changeCallBack( element, true );
	sdaPin.changeCallBack( element, true );

	seq_ptr = 0;
	seq_err = false;
	seq_state = seq_idle;
	seq_session = false;
	devaddr = 0;
}

uint tokenizer( void )
{
uint	i, len;
int		i2;
uint	state;
bool	error;
int		value;
uint	dcnt;

	Sequence = {};
	len = uart_buffer.length();
	i = 0;
	state = s_command;
	error = false;
	while( i < len )
	{
		switch( state )
		{
			case s_command:
				switch( uart_buffer[i] )
				{
					case	0x40:		// @
						Sequence.insertLast( 0x40 );
						i2 = uart_buffer.findFirstOf( " ", i+1 );
						if( i2 < 0 )
						{
							value = convert( uart_buffer.substr( i+1 ) );
							i = len;
						}
						else
						{
							value = convert( uart_buffer.substr( i+1, i2-(i+1) ) );
							i = i2;
						}
						if( value < 0 )
						{
							error = true;
						}
						else
						{
							Sequence.insertLast( value );
						}
						break;

					case	0x57:		// W
					case	0x77:
						Sequence.insertLast( 0x57 );
						i2 = uart_buffer.findFirstOf( " ", i+1 );
						if( i2 < 0 )
						{
							value = convert( uart_buffer.substr( i+1 ) );
							i = len;
						}
						else
						{
							value = convert( uart_buffer.substr( i+1, i2-(i+1) ) );
							i = i2;
						}
						if( value < 0 )
						{
							error = true;
						}
						else
						{
							Sequence.insertLast( value );
							dcnt = value;
							state = s_data;
						}
						break;

					case	0x52:		// R
					case	0x72:
						Sequence.insertLast( 0x52 );
						i2 = uart_buffer.findFirstOf( " ", i+1 );
						if( i2 < 0 )
						{
							value = convert( uart_buffer.substr( i+1 ) );
							i = len;
						}
						else
						{
							value = convert( uart_buffer.substr( i+1, i2-(i+1) ) );
							i = i2;
						}
						if( value < 0 )
						{
							error = true;
						}
						else
						{
							Sequence.insertLast( value );
						}
						break;

					case	0x20:		// space
						break;

					default:
						error = true;
				}
				break;

			case s_data:
				switch( uart_buffer[i] )
				{
					case	0x20:		// space
						break;

					default:
						i2 = uart_buffer.findFirstOf( " ", i );
						if( i2 < 0 )
						{
							value = convert( uart_buffer.substr( i ) );
							i = len;
						}
						else
						{
							value = convert( uart_buffer.substr( i, i2-i ) );
							i = i2;
						}
						if( value < 0 )
						{
							error = true;
						}
						else
						{
							Sequence.insertLast( value );
							dcnt--;
							if( dcnt == 0 )
							{
								state = s_command;
							}
						}
						break;
				}
				break;
		}
		if( error == true )
			break;
		i++;
	}
	i2 = i;
	if( error == false )
	{
		i2 = -1;
	}
	return i2;
}

int	convert( string str )
{
int		value;

	if( str.substr(0,2) == "0x" || str.substr(0,2) == "0X" )
	{
		value = parseInt( str.substr( 2 ), 16 );
	}
	else
	{
		value = parseInt( str, 10 );
	}
	return value;
}

void execute( void )
{
	if( seq_err == false )
	{
		if( seq_state == seq_idle )
		{
			seq_state = seq_command;
		}
		switch( seq_state )
		{
			case seq_command:
				if( seq_ptr < Sequence.length() )
				{
					switch( Sequence[seq_ptr] )
					{
						case 0x40:	// @
							devaddr = Sequence[seq_ptr+1];
							seq_ptr += 2;
							execute();
							break;

						case 0x57:	// W
							if( devaddr == 0 )
							{
								seq_err = true;
							}
							else
							{
								seq_counter = Sequence[seq_ptr+1];
								seq_ptr += 2;
								seq_state = seq_write;
								if( iic_writeAddress( devaddr, false ) == false )
								{
									seq_err = true;
								}
							}
							break;

						case 0x52:	// R
							if( devaddr == 0 )
							{
								seq_err = false;
							}
							else
							{
								seq_counter = Sequence[seq_ptr+1];
								seq_ptr += 2;
								seq_state = seq_read;
								if( iic_writeAddress( devaddr, true ) == false )
								{
									seq_err = true;
								}
							}
							break;
					}
					break;
				}
				else
				{
					seq_err = true;
				}

			case seq_write:
				if( seq_counter > 0 )
				{
					if( iic_writeByte( Sequence[seq_ptr] ) == true ) 
					{
						seq_counter--;
						seq_ptr++;
					}
					else
					{
						seq_err = true;
					}
				}
				else
				{
					seq_state = seq_command;
					execute();
				}
				break;

			case seq_read:
				if( seq_counter > 0 )
				{
					if( iic_readByte() == true ) 
					{
						seq_counter--;
					}
					else
					{
						seq_err = true;
					}
				}
				else
				{
					seq_state = seq_command;
					execute();
				}
				break;
		}
		if( seq_err == true )
		{
			if( seq_session == true )
			{
				iic_sendStop();
			}
		}
	}
	else
	{
		if( seq_ptr < Sequence.length() )
		{
			print( "ERROR - i2c tester - Sequence error " + seq_ptr );
		}
	}
}

void byteReceived( uint data )
{
uint i;
int i2;

	if( data == 0x0d )
	{
		i2 = tokenizer();
		uart_buffer = "";
		if( i2 < 0 )
		{
			devaddr = 0;
			seq_ptr = 0;
			seq_err = false;
			seq_session = false;
			seq_state = seq_idle;
			execute();
		}
		else
		{
			print( "ERROR - i2c tester - error in serial buffer " + i2 );
			uart_buffer = "";
		}
	}
	else
	{
		if( data >= 32 && data < 127 )
			uart_buffer += converter.substr(data-32,1);
	}
}

void frameSent( uint data )
{
}

void iic_byteSent( bool ack )
{
	if( ack == true )
	{
		seq_session = true;
		execute();
	}
	else
	{
		seq_err = true;
		seq_session = false;
		iic_sendStop();
	}

}

bool iic_byteReceived( uint8 data )
{
bool	retval = true;

	if( seq_counter == 0 )
	{
		retval = false;
	}
	return	retval;
}

void iic_ackSent( bool ack )
{
	execute();
}

bool iic_writeAddress( uint8 address, bool rw )
{
bool	retval = true;

	if( iic_inprogress == true || iic_state == iic_read )
	{
		retval = false;
	}
	else
	{
		iic_inprogress = true;
		iic_sbuf = address << 1;
		iic_rw = rw;
		if( rw == true )
			iic_sbuf |= 1;
		if( iic_session == true )
		{
			iic_state = iic_start;
			iic_session = false;
			sdaPin.scheduleState( true, 2.5e6 );
			sclPin.scheduleState( true, 5e6 );
		}
		else
		{
			iic_state = iic_start;
			sdaPin.scheduleState( false, 2.5e6 );
			sclPin.scheduleState( false, 5e6 );
		}
	}
	return	retval;
}

bool iic_sendStop( void )
{
bool	retval = true;

	if( iic_state != iic_idle && iic_state != iic_read && iic_inprogress == false )
	{
		iic_inprogress = true;
		iic_state = iic_stop;
		sdaPin.scheduleState( false, 2.5e6 );
		sclPin.scheduleState( true, 5e6 );
	}
	else
	{
		retval = false;
	}
	return retval;
}

bool iic_writeByte( uint8 data )
{
bool	retval = true;

	if( iic_state == iic_write && iic_inprogress == false )
	{
		iic_inprogress = true;
		iic_sbuf = data;
		iic_bitcnt = 8;
		if( iic_sbuf & 0x80 != 0  )
		{
			sdaPin.scheduleState( true, 2.5e6 );
		}
		else
		{
			sdaPin.scheduleState( false, 2.5e6 );
		}
		iic_sbuf <<= 1;
		iic_bitcnt--;
		sclPin.scheduleState( true, 5e6 );
	}
	else
	{
		retval = false;
	}
	return retval;
}

bool iic_readByte( void )
{
bool	retval = true;

	if( iic_state == iic_read && iic_inprogress == false )
	{
		iic_inprogress = true;
		iic_bitcnt = 8;
		sclPin.scheduleState( true, 5e6 );
	}
	else
	{
		retval = false;
	}
	return retval;
}

void voltChanged( void )
{
bool	scl, sda;

	scl = sclPin.getInpState();
	sda = sdaPin.getInpState();
	if( sda != sdaPrev )
	{
		sdaPrev = sda;
		if( sda == true && scl == true )	// STOP
		{
			iic_session = false;
			iic_inprogress = false;
			iic_state = iic_idle;
		}
	}
	if( scl != sclPrev )
	{
		sclPrev = scl;
		if( scl == true )
		{
			switch( iic_state )
			{
				case iic_start:
					sdaPin.scheduleState( false, 2.5e6 );
					sclPin.scheduleState( false, 5e6 );
					break;

				case iic_stop:
					sdaPin.scheduleState( true, 2.5e6 );
					break;

				case iic_write:
				case iic_setack:
					sclPin.scheduleState( false, 5e6 );
					break;

				case iic_read:
					if( sda == true )
					{
						iic_rbuf |= 1;
					}
					iic_rbuf <<= 1;
					iic_bitcnt--;
					sclPin.scheduleState( false, 5e6 );
					break;

				case iic_getack:
					iic_ack = !sda;
					sclPin.scheduleState( false, 5e6 );
					break;
			}
		}
		else
		{
			switch( iic_state )
			{
				case iic_start:
					iic_state = iic_write;
					iic_bitcnt = 8;
					if( iic_sbuf & 0x80 != 0  )
					{
						sdaPin.scheduleState( true, 2.5e6 );
					}
					else
					{
						sdaPin.scheduleState( false, 2.5e6 );
					}
					iic_sbuf <<= 1;
					iic_bitcnt--;
					sclPin.scheduleState( true, 5e6 );
					break;

				case iic_write:
					if( iic_bitcnt > 0 )
					{
						if( iic_sbuf & 0x80 != 0  )
						{
							sdaPin.scheduleState( true, 2.5e6 );
						}
						else
						{
							sdaPin.scheduleState( false, 2.5e6 );
						}
						iic_sbuf <<= 1;
						iic_bitcnt--;
						sclPin.scheduleState( true, 5e6 );
					}
					else
					{
						iic_state = iic_getack;
						sdaPin.scheduleState( true, 1.25e6 );
						sclPin.scheduleState( true, 5e6 );
					}
					break;

				case iic_read:
					if( iic_bitcnt == 0 )
					{
						if( iic_byteReceived( iic_rbuf ) == true )
						{
							sdaPin.scheduleState( false, 2.5e6 );
						}
						else
						{
							sdaPin.scheduleState( true, 2.5e6 );
						}
						iic_state = iic_setack;
					}
					sclPin.scheduleState( true, 5e6 );
					break;

				case iic_setack:
					if( sda == true )
					{
						iic_state = iic_nacksent;
					}
					else
					{
						iic_state = iic_read;
					}
					iic_inprogress = false;
					sdaPin.scheduleState( true, 1.25e6 );
					iic_ackSent( !sda );
					break;

				case iic_getack:
					if( iic_rw == true && iic_ack == true )
					{
						iic_state = iic_read;
					}
					else
					{
						iic_state = iic_write;
					}
					iic_session = true;
					iic_inprogress = false;
					iic_byteSent( iic_ack );
					break;
			}
		}
	}
}
