IoPin@	sclPin = component.getPin("scl");
IoPin@	sdaPin = component.getPin("sda");

enum iicState_t{
	iic_idle=0,
	iic_address,
	iic_write,
	iic_getack,
};

bool	sclPrev, sdaPrev;
int		iic_state;
bool	iic_session;
uint8	rbuf, bitcnt;
bool	rw;
bool	ack;
bool	address;

void setup()
{
}

void reset()
{
	sclPin.setPinMode( 1 );
	sdaPin.setPinMode( 1 );

	sclPin.changeCallBack( element, true );
	sdaPin.changeCallBack( element, true );

	iic_state = iic_idle;
	iic_session = false;
	sclPrev = true; sdaPrev = true;
}

void voltChanged()
{
bool	scl, sda;

	scl = sclPin.getInpState();
	sda = sdaPin.getInpState();

	if( sda != sdaPrev )
	{
		sdaPrev = sda;
		if( scl == true )
		{
			if( sda == false )
			{
				if( iic_session == true )
				{
					print( "RESTART" );
				}
				else
				{
					print( "START" );
				}
				switch( iic_state )
				{
					case iic_idle:
					case iic_write:
						iic_session == false;
						iic_state = iic_address;
						rbuf = 0; bitcnt = 8;
						break;
				}
			}
			else
			{
				print( "STOP" );
				iic_session = false;
				iic_state = iic_idle;
			}
		}
	}
	if( scl != sclPrev )
	{
		sclPrev = scl;
		if( scl == false )
		{
			switch( iic_state )
			{
				case iic_address:
					if( bitcnt == 0 )
					{
						rw = false;
						if( rbuf & 0x01 != 0 )
						{
							rw = true;
						}
						print( "Address : 0x" + formatInt( rbuf>>1, '0H', 2 ) + ", R/W = " + (rw ? "Read" : "Write" ) );
						iic_session = true;
						address = true;
						iic_state = iic_getack;
					}
					break;

				case iic_write:
					if( bitcnt == 0 )
					{
						if( rw == false )
						{
							print( "M->S : 0x" + formatInt( rbuf, '0H', 2 ) );
						}
						else
						{
							print( "S->M : 0x" + formatInt( rbuf, '0H', 2 ) );
						}
						iic_state = iic_getack;
					}
					break;

				case iic_getack:
					rbuf = 0; bitcnt = 8;
					iic_state = iic_write;
			}
		}
		else
		{
			switch( iic_state )
			{
				case iic_address:
				case iic_write:
					rbuf <<=1;
					if( sda == true )
						rbuf |= 0x1;
					bitcnt--;
					break;

				case iic_getack:
					if( sda == false )
					{
						ack = true;
						if( rw == false || address == true)
						{
							print( "S->M : ACK" );
						}
						else
						{
							print( "M->S : ACK" );
						}
					}
					else
					{
						ack = false;
						if( rw == false || address == true)
						{
							print( "S->M : NACK" );
						}
						else
						{
							print( "M->S : NACK" );
						}
					}
					address = false;
					break;
			}
		}
	}
}
