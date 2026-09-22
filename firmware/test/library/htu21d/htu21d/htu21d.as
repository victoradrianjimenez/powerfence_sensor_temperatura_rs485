IoPin@    sclPin = component.getPin("scl");
IoPin@    sdaPin = component.getPin("sda");
IoPin@    vddPin = component.getPin("vdd");
IoPin@    vtempPin = component.getPin("vtemp");
IoPin@    vhumidPin = component.getPin("vhumid");

enum iicState_t{
    iic_idle=0,
    iic_address,
    iic_write,
    iic_read,
    iic_getack,
    iic_setack,
    iic_setnack
};

enum chipState_t{
    chip_idle=0,
    chip_command,
    chip_command2,
    chip_data,
    chip_readmeasdata
};

enum command_t{
    nocommand=0,
    temphold=0xe3,
    humihold=0xe5,
    tempnohold=0xf3,
    huminohold=0xf5,
    wruser=0xe6,
    rduser=0xe7,
    softreset=0xfe,
    serial1=0xfa,    // 0x0F
    serial2=0xfc     // 0xC9
};

bool    sclPrev, sdaPrev;
int        iic_state;
bool    iic_stretch;
bool    iic_session;
uint8    myI2cAddress;
uint8    sbuf, rbuf, bitcnt;
bool    rw;
bool    ack;
uint64    delay;
uint16    result;
uint8    crc;
int        bytecount;

int        chip_state;
bool    chip_error;
uint8    command;
uint8    user_register;
array<uint8>    buffer(9);
int        bufptr;
double    vdd_min;
bool    debug = false; // Variable declarada para evitar el error

void setup()
{
}

void reset()
{
    sclPin.setPinMode( 2 );             // Open Drain
    sdaPin.setPinMode( 2 );             // Open Drain
    vddPin.setPinMode( 1 );             // Input
    vtempPin.setPinMode( 1 );        // Input
    vhumidPin.setPinMode( 1 );        // Input

    sclPin.changeCallBack( element, true );
    sdaPin.changeCallBack( element, true );

    chip_state = chip_idle;
    command = nocommand;
    chip_error = false;

    delay = 350000;                 // 350 ns
    myI2cAddress = 0x40;        // HTU21D
    iic_state = iic_idle;
    iic_stretch = false;
    iic_session = false;
    sclPrev = true; sdaPrev = true;
    sdaPin.setOutState( true );
    sclPin.setOutState( true );

    user_register = 0x02;
    vdd_min = 2.25;
}

void runEvent()
{
double    anaval;

    bufptr = 0;
    bytecount = 0;
    buffer[0] = 0xFF;
    if( chip_error == false )
    {
        switch( command )
        {
            case temphold:
            case tempnohold:
                anaval = vtempPin.getVoltage();
                anaval *= 100;        // 10mv/°C
                if( anaval <= -46.85 )
                {
                    result = 0;
                }
                else if( anaval >= 128.87 )
                {
                    result = 0xFFFF;
                }
                else
                {
                    anaval += 46.85;
                    anaval /= 175.72;
                    result = int(anaval * 65536);
                    result &= 0xFFFC;
                    switch( user_register & 0x81 )
                    {
                        case 0x00:
                            result &= 0xFFFC;
                            break;

                        case 0x01:
                            result &= 0xFFF0;
                            break;

                        case 0x80:
                            result &= 0xFFF8;
                            break;

                        case 0x81:
                            result &= 0xFFE0;
                            break;
                    }
                }
                buffer[0] = result >> 8;
                buffer[1] = result & 0xFF;
                buffer[2] = calcCRC( result );
                buffer[3] = 0xFF;
                break;

            case humihold:
            case huminohold:
                anaval = vhumidPin.getVoltage();
                anaval *= 100;        // 1V = 100%
                if( anaval <= -6 )
                {
                    result = 0;
                }
                else if( anaval >= 119 )
                {
                    result = 0xFFFF;
                }
                else
                {
                    anaval += 6;
                    anaval /= 125;
                    result = int(anaval * 65536);
                    result &= 0xFFFC;
                    switch( user_register & 0x81 )
                    {
                        case 0x00:
                            result &= 0xFFF0;
                            break;

                        case 0x01:
                            result &= 0xFF00;
                            break;

                        case 0x80:
                            result &= 0xFFC0;
                            break;

                        case 0x81:
                            result &= 0xFFE0;
                            break;
                    }
                    result |= 2;
                }
                buffer[0] = result >> 8;
                buffer[1] = result & 0xFF;
                buffer[2] = calcCRC( result );
                buffer[3] = 0xFF;
                break;
        }
        bytecount = 3;
    }
    if( vddPin.getVoltage() < vdd_min )
    {
        user_register |= 0x40;
    }
    else
    {
        user_register &= 0xBF;
    }
    command = nocommand;
    chip_state = chip_idle;
    if( iic_stretch == true )
    {
        endStretch( buffer[0] );
    }
    chip_error = false;
}

uint8    calcCRC( uint16 baseval )
{
uint8    crc;
uint16    poly = 0x9880;

    for( int i = 0; i < 16; i++ )
    {
        if( (baseval & 0x8000) > 0 ) // Paréntesis corregidos
        {
            baseval ^= poly;
        }
        baseval <<= 1;
    }
    baseval >>= 8;
    crc = baseval;
    return crc;
}

bool addressReceived( bool rw )
{
bool    ack = true;

    if( rw == true )
    {
        switch( command )
        {
            case nocommand:
                if( bytecount > 0 )
                {
                    bufptr = 0;
                    sbuf = buffer[bufptr];
                    chip_state = chip_readmeasdata;
                }
                else
                {
                    print( "ERROR - HTU21D - There's nothing to read" );
                    chip_state = chip_idle;
                    ack = false;
                }
                break;

            case temphold:    // 0xE3
            case humihold:    // 0XE5
                iic_stretch = true;
                if( chip_error == true )
                    ack = false;
                break;
            
            default:                
                print( "ERROR - HTU21D - Command while measurement is in progress" );
                chip_state = chip_idle;
                ack = false;
        }
    }
    else
    {
        switch( command )
        {
            case temphold:    // 0xE3
            case humihold:    // 0xE5
                ack = false;
                break;
        }
        if( ack == true )
        {
            bufptr = 0;
            buffer[0] = nocommand;
            chip_state = chip_command;
        }
    }
    return ack;
}

bool byteReceived( uint8 data )
{
bool    ack = true;

    if( bufptr < 3 )
    {
        buffer[bufptr] = data;
        bufptr++;
        switch( chip_state )
        {
            case chip_command:
                switch( data )
                {
                    case temphold:
                    case humihold:
                    case tempnohold:
                    case huminohold:
                    case wruser:
                    case rduser:
                        if( command != nocommand )
                        {
                            chip_error = true;
                            print( "ERROR - HTU21D - Command while measurement is in progress" );
                            ack = false;
                        }                    
                        chip_state = chip_data;
                        break;

                    case softreset:
                        chip_state = chip_data;
                        break;

                    case serial1:
                    case serial2:
                        if( command != nocommand )
                        {
                            chip_error = true;
                            print( "ERROR - HTU21D - Command while measurement is in progress" );
                            chip_state = chip_data;
                            ack = false;
                        }
                        else
                        {
                            chip_state = chip_command2;
                        }
                        break;

                    default:
                        print( "ERROR - HTU21D - Invalid command" );
                        chip_state = chip_data;
                        ack = false;
                }
                break;

            case chip_command2:
                switch( buffer[0] )
                {
                    case serial1:    // 0xfa
                        if( data != 0x0f )
                        {
                            print( "ERROR - HTU21D - Invalid command" );
                            ack = false;
                        }
                        break;

                    case serial2:    // 0xfc
                        if( data != 0xc9 )
                        {
                            print( "ERROR - HTU21D - Invalid command" );
                            ack = false;
                        }
                        break;
                }
                chip_state = chip_data;
                if( debug )
                    print( "chip_state: DATA" );
                break;
        }
    }
    else
    {
        print( "ERROR - HTU21D - Too much data" );
        ack = false;
    }
    return ack;
}

void byteSent( void )
{
    if( bytecount > 0 )
    {
        bufptr++;
        bytecount--;
    }
    sbuf = buffer[bufptr];
}

void endSession( void )
{
    iic_session = false;
    if( rw == false )
    {
        switch( buffer[0] )
        {
            case    temphold:
            case    tempnohold:
                if( command == nocommand )
                {
                    command = buffer[0];
                    if( bufptr > 1 )
                    {
                        print( "ERROR - HTU21D - extra data after command" );
                        chip_error = true;
                    }
                    switch( user_register & 0x81 )
                    {
                        case 0x00:        // 44 ms
                            component.addEvent( 44e9 );
                            break;

                        case 0x01:        // 11 ms
                            component.addEvent( 11e9 );
                            break;

                        case 0x80:        // 22 ms
                            component.addEvent( 22e9 );
                            break;

                        case 0x81:        // 6 ms
                            component.addEvent( 6e9 );
                            break;
                    }
                }
                chip_state = chip_idle;
                break;

            case    humihold:
            case    huminohold:
                if( command == nocommand )
                {
                    command = buffer[0];
                    if( bufptr > 1 )
                    {
                        print( "ERROR - HTU21D - extra data after command" );
                        chip_error = true;
                    }
                    switch( user_register & 0x81 )
                    {
                        case 0x00:        // 14 ms
                            component.addEvent( 14e9 );
                            break;

                        case 0x01:        // 2 ms
                            component.addEvent( 2e9 );
                            break;

                        case 0x80:        // 4 ms
                            component.addEvent( 4e9 );
                            break;

                        case 0x81:        // 7 ms
                            component.addEvent( 7e9 );
                            break;
                    }
                }
                chip_state = chip_idle;
                break;

            case    wruser:
                if( command == nocommand )
                {
                    if( bufptr > 1 )
                    {
                        user_register = buffer[1];
                        command = nocommand;
                    }
                }
                chip_state = chip_idle;
                break;

            case    rduser:
                if( command == nocommand )
                {
                    if( bufptr == 1 )
                    {
                        buffer[0] = user_register;
                        buffer[1] = 0x00;
                        bytecount = 1;
                    }
                    else
                    {
                        print( "ERROR - HTU21D - extra data after command" );
                        bytecount = 0;
                    }
                }
                chip_state = chip_idle;
                break;

            case    softreset:
                component.cancelEvents();
                command = nocommand;
                user_register &= 0x04;
                user_register |= 0x02;
                chip_error = false;
                chip_state = chip_idle;
                break;

            case    serial1:
                if( command == nocommand )
                {
                    if( bufptr == 2 )
                    {
                        if( buffer[1] == 0x0F )
                        {
                            buffer[0] = 0x00;
                            buffer[1] = 0x00;
                            buffer[2] = 0x18;
                            buffer[3] = 0xfa;
                            buffer[4] = 0x17;
                            buffer[5] = 0xd4;
                            buffer[6] = 0x19;
                            buffer[7] = 0xcb;
                            bytecount = 8;
                            buffer[8] = 0x00;
                        }
                        else
                        {
                            bytecount = 0;
                        }
                    }
                    else
                    {
                        print( "ERROR - HTU21D - extra data after command" );
                        bytecount = 0;
                    }
                }
                chip_state = chip_idle;
                break;

            case    serial2:
                if( command == nocommand )
                {
                    if( bufptr == 2 )
                    {
                        if( buffer[1] == 0xC9 )
                        {
                            buffer[0] = 0x32;
                            buffer[1] = 0x11;
                            buffer[2] = 0x19;
                            buffer[3] = 0x48;
                            buffer[4] = 0x54;
                            buffer[5] = 0x04;
                            bytecount = 6;
                            buffer[6] = 0x00;
                        }
                        else
                        {
                            bytecount = 0;
                        }
                    }
                    else
                    {
                        print( "ERROR - HTU21D - extra data after command" );
                        bytecount = 0;
                    }
                }
                chip_state = chip_idle;
                break;
        }
    }
    else
    {
        bytecount = 0;
        chip_state = chip_idle;
    }
}

void endStretch( uint8 data )
{
    sbuf = data;
    if( iic_stretch == true )
    {
        bitcnt = 8;
        if( (sbuf & 0x80) != 0 ) // Paréntesis corregidos
        {
            sdaPin.setOutState( true );
        }
        else
        {
            sdaPin.setOutState( false );
        }
        sbuf <<= 1;
        bitcnt--;
    }
    chip_state = chip_readmeasdata;
    iic_stretch = false;
    sclPin.scheduleState( true, delay );
}

void voltChanged()
{
bool    scl, sda;

    scl = sclPin.getInpState();
    sda = sdaPin.getInpState();

    if( sda != sdaPrev )
    {
        sdaPrev = sda;
        if( scl == true )
        {
            if( sda == false )
            {
                switch( iic_state )
                {
                    case iic_idle:
                    case iic_write:
                        if( iic_session == true )
                        {
                            endSession();
                        }
                        iic_state = iic_address;
                        rbuf = 0; bitcnt = 8;
                        break;

                    default:
                        iic_state = iic_idle;
                }
            }
            else
            {
                if( iic_session == true )
                {
                    endSession();
                }
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
                        if( rbuf >> 1 == myI2cAddress )
                        {
                            rw = false;
                            if( (rbuf & 0x01) != 0 ) // Paréntesis corregidos
                                rw = true;
                            if( addressReceived( rw ) == true )
                            {
                                iic_session = true;
                                iic_state = iic_setack;
                                sdaPin.scheduleState( false, delay );
                            }
                            else
                            {
                                iic_state = iic_setnack;
                            }
                        }
                        else
                        {
                            iic_state = iic_idle;
                        }
                    }
                    break;

                case iic_write:
                    if( bitcnt == 0 )
                    {
                        if( byteReceived( rbuf ) == true )
                        {
                            iic_state = iic_setack;
                            sdaPin.scheduleState( false, delay );
                        }
                        else
                        {
                            iic_state = iic_setnack;
                        }
                    }
                    break;

                case iic_read:
                    if( bitcnt == 0 )
                    {
                        iic_state = iic_getack;
                        sdaPin.scheduleState( true, delay );
                    }
                    else
                    {
                        if( (sbuf & 0x80) != 0 ) // Paréntesis corregidos
                        {
                            sdaPin.scheduleState( true, delay );
                        }
                        else
                        {
                            sdaPin.scheduleState( false, delay );
                        }
                        sbuf <<= 1;
                        bitcnt--;
                    }
                    break;

                case iic_setack:
                    if( rw == true )
                    {
                        iic_state = iic_read;
                        if( iic_stretch == true )
                        {
                            sclPin.scheduleState( false, delay );
                        }
                        else
                        {
                            bitcnt = 8;
                            if( (sbuf & 0x80) != 0 ) // Paréntesis corregidos
                            {
                                sdaPin.scheduleState( true, delay );
                            }
                            else
                            {
                                sdaPin.scheduleState( false, delay );
                            }
                            sbuf <<= 1;
                            bitcnt--;
                        }
                    }
                    else
                    {
                        sdaPin.scheduleState( true, delay );
                        iic_state = iic_write;
                        bitcnt = 8;
                    }
                    break;

                case iic_setnack:
                    iic_state = iic_idle;
                    if( iic_stretch == true )
                    {
                        sclPin.scheduleState( false, delay );
                    }
                    break;

                case iic_getack:
                    if( ack == true )
                    {
                        byteSent();
                        iic_state = iic_read;
                        if( iic_stretch == true )
                        {
                            sclPin.scheduleState( false, delay );
                        }
                        else
                        {
                            bitcnt = 8;
                            if( (sbuf & 0x80) != 0 ) // Paréntesis corregidos
                            {
                                sdaPin.scheduleState( true, delay );
                            }
                            else
                            {
                                sdaPin.scheduleState( false, delay );
                            }
                            sbuf <<= 1;
                            bitcnt--;
                        }
                    }
                    else
                    {
                        sdaPin.scheduleState( true, delay );
                        iic_state = iic_idle;
                    }
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
                    }
                    else
                    {
                        ack = false;
                    }
                    break;
            }
        }
    }
}
