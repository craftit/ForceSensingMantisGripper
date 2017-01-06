#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL  // 16 MHz

#include <util/delay.h>
#include "Fifo.hh"

// Controller pins

// PIN D

// PB0  X -
// PB1  X -
// PB2  O - PWM Out
// PB3  X -
// PB4  X -
// PB5  O - LED
// PB6  X -
// PB7  X -

// PC:
// ADC0 A - Current
// ADC1 A - Position 0 - 5V
// ADC2 X -
// ADC3 X -
// ADC4 X -
// ADC5 X -
// ADC6 X -
// ADC7 X -

// PD0  X -
// PD1  X -
// PD2  O - Ain1
// PD3  O - Standby
// PD4  O - Ain2
// PD5  X -
// PD6  X -
// PD7  X -

FifoC<0x1f> g_txBuff;

const uint8_t g_admuxConfg = _BV(REFS0) ; // Select 5 (VCC) Volt ref.

enum ServoModeT {
  SM_Position = 0,
  SM_Force = 1,
  SM_Free = 2
} g_servoMode = SM_Free;

int16_t g_forceLimit = 500;
int16_t g_velocityLimit = 1024;

int32_t g_velocityResidual = 0;
int32_t g_velocityEstimate = 0;

int16_t g_targetPosition = 500;
int16_t g_targetForce = g_forceLimit;

int8_t g_lastSample = 0;
int16_t g_lastPosition = 0;
int16_t g_lastCurrent = 0;
int8_t g_haveSample = 0;

uint8_t g_atGoal = 0; // Are we at the goal position ?
uint8_t g_stalled = 0;
int16_t g_stallCount = 0;

void InitIO()
{
  // ---------------------------------------
  // Lets get things going full speed

  CLKPR = _BV(CLKPCE); // Set clock change enable bit.
  CLKPR = 0;      // Set division to 1.

  // ---------------------------------------
  // Setup ports
  // IO.  DDR  1=Output 0=Input
  // See pg 107 in datasheet.

  // Port B
  //
  PORTB = _BV(PB5);
  DDRB =  _BV(DDB1) | _BV(DDB2) | _BV(DDB5);

  // Port C is all inputs.
  // Pull inputs PB4 and PB5 high.
  PORTC = _BV(PB4) | _BV(PB5);
  DDRC = 0;

  // Port D, motor control. Everything off to start with.
  PORTD = 0;
  DDRD  = _BV(DDD2) | _BV(DDD3) | _BV(DDD4);


  // ---------------------------------------
  // Setup the counter/time
#if 1
  TCNT1 = 0;    // Start at 0
  ICR1 = 512;
  OCR1A = 12;  // TOP Value
  OCR1B = 64;  // Change at.

  // Enable interrupts
  TIMSK1 =  _BV(TOIE1); // _BV(OCIE1B) | _BV(OCIE1A) |

  // Configure timer.
  // Phase correct, TOP in ICR1
  // Pg 188 in datasheet.
  TCCR1A = _BV(WGM11) | _BV(COM1B1)| _BV(COM1A1);
  TCCR1B = _BV(WGM13) | _BV(CS10);
#endif

#if 0
  // ---------------------------------------
  // Setup timer 0 for 100Hz
  OCR0A = 40;
  //OCR0A = 195; // Give a 100Hz clock at 20Mhz

  TCCR0A = _BV(WGM01);  // Enable CTC, no output.
  TCCR0B = _BV(CS02) | _BV(CS00); // Set clock to clkio/1024
  TIMSK0 = 0;//_BV(OCIE0A); // Enable interrupt

  // ---------------------------------------
  // Setup timer 2

  OCR2A = 195;
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
  TIMSK2 = 0;//_BV(OCIE0A); // Disable interrupt
#endif

  // ---------------------------------------
  // Setup UART.

  // 16MHz
  //  38400 : 25 Good
  //  57600 : 16 ok ish
  // 115200 :  8 Dodgy
  // 250000 :  3 Good
#if 1
  UBRR0H = 0;
  UBRR0L = 25; //16;

  // 8-bit data, 1-stop bit no parity.
  UCSR0A = 0;
  UCSR0C =  _BV(UCSZ01) | _BV(UCSZ00);
  UCSR0B =  _BV(RXEN0) | _BV(TXEN0); // |_BV(RXCIE0) | _BV(UDRIE0);
#endif
  // ---------------------------------------
  // Setup ADC
#if 1
  DIDR0 = _BV(ADC0D) | _BV(ADC1D) | _BV(ADC2D) | _BV(ADC3D) | _BV(PD4) | _BV(PD5) | _BV(PD6) | _BV(PD7); // Disable digital inputs.
  ADMUX = g_admuxConfg; //_BV(REFS0) | _BV(REFS1);

  ADCSRB = _BV(ADTS1) | _BV(ADTS2); // Trigger on time counter 1 overflow.
  // Divide system clock by 64, enable ADC .
  // ; Start conversion bit.
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADIE); // Trigger on Counter timer overflow event.
  //  | _BV(ADATE)
#endif
}

ISR(ADC_vect)
{
  PINB = _BV(PB5);
//  PINB = _BV(PB5);

  if(g_lastSample) {
    ADMUX = g_admuxConfg | MUX1; // MUX 0 Sample position.
    g_lastCurrent = ADC;
    g_lastSample = 0;

    if(g_servoMode == SM_Force) {
      int demand = 0;
      int16_t error = (g_targetForce - (int16_t) g_lastCurrent) * 8;
      if(g_targetForce > 0) {
        PORTD = _BV(PD4) | _BV(PD3); // Close (increase pos)
        demand = error;
      } else {
        PORTD = _BV(PD2) | _BV(PD3); // Open (decrease pos)
        demand = -error;
      }

      if(demand >= 0) {
        if(demand > g_forceLimit)
          OCR1B = g_forceLimit;
        else {
          if(demand < 1)
            demand = 1;
          OCR1B = demand;
        }
      } else {
        OCR1B = 1;
      }

    }


  } else {
    ADMUX = g_admuxConfg; // Sample current.
    int16_t newPosition = ADC;
    int16_t rawVelocity =  newPosition -  g_lastPosition;
    g_lastPosition = newPosition;
    g_lastSample = 1;

    g_velocityResidual += g_velocityEstimate;
    int32_t diff = g_velocityResidual/64;
    g_velocityResidual -= diff * 64;
    g_velocityEstimate = g_velocityEstimate + (((int32_t) rawVelocity) * 4096) - diff;

    g_haveSample = 1;


    // We have a position update.

    // Position 0=Open 1024=Closed
    if(g_servoMode == SM_Position) {

      int16_t demand = 0;
      int16_t error = (g_targetPosition - g_lastPosition);
      if(error > 0) {
        PORTD = _BV(PD4) | _BV(PD3); // Close (increase pos)
        demand = error;
      } else {
        PORTD = _BV(PD2) | _BV(PD3); // Open (decrease pos)
        demand = -error;
      }


      // At goal with some hysteresis
      g_atGoal = (demand < (g_atGoal ? 20 : 16)) ? 1 : 0;

      // We need some latency in deciding if we have stalled, the motor may only just be starting
      // to turn so our velocity could be low. To avoid this we put in a delay before deciding we're stalled.
      int32_t mag = g_velocityEstimate ;
      if(mag < 0) mag *= -1;

      bool isStalled = !g_atGoal && ((mag < (g_stalled ? 12200 : 9000)) ? 1 : 0);
      if(!isStalled) {
        // Reset stall counter.
        g_stallCount = 0;
        g_stalled = 0;
      } else {
        // Count to 100 before declaring us stalled.
        if(g_stallCount < 100) {
          g_stallCount++;
        } else {
          g_stalled = 1;
        }
      }


      if(demand >= 0) {
        demand *= 8; // Gain
        if(demand > g_targetForce) {
          OCR1B = g_targetForce;
        } else {
          if(demand < 1)
            demand = 1;
          OCR1B = demand;
        }
      } else {
        OCR1B = 1;
      }
    }

  }

  if(g_servoMode == SM_Free) {
    PORTD = 0; // Turn off motor controller.
  }


}

ISR(TIMER1_OVF_vect)
{
  ADCSRA = ADCSRA | _BV(ADSC); // Start conversion.

}



const uint8_t g_charSTX = 0x02;
const uint8_t g_charETX = 0x03;

uint8_t g_checksumFailure = 0;
uint8_t g_packetCount = 0;


void SendPacket(char *buff,uint8_t len)
{
  if((5 + len) > g_txBuff.Space()) {
    // Count dropped packets?
    return;
  }

  g_txBuff.PutNoLock(g_charSTX);
  int crc = len + 0x55;
  g_txBuff.PutNoLock(len);
  for(int i =0;i < len;i++) {
    uint8_t data = buff[i];
    g_txBuff.PutNoLock(data);
    crc += data;
  }
  g_txBuff.PutNoLock(crc);
  g_txBuff.PutNoLock(crc >> 8);
  g_txBuff.PutNoLock(g_charETX);
}


class SerialDecodeC
{
public:

  //! Accept a byte
  void AcceptByte(uint8_t sendByte);

  //! Process received packet.
  void ProcessPacket();

  int m_state = 0;
  int m_checkSum = 0;
  int m_packetLen = 0;
  uint8_t m_data[255];
  int m_at = 0;

  // Packet structure.
  // x    STX
  // x    Len - Of data excluding STX,ETX and Checksum.
  // 0    Address
  // 1    Type
  // 2..n data.
  // n    2-CRC
  // n    ETX.
};

void SerialDecodeC::AcceptByte(uint8_t sendByte)
{
  switch(m_state)
  {
  case 0: // Wait for STX.
    if(sendByte == g_charSTX)
      m_state = 1;
    // Else remain in state 0.
    break;
  case 1: // Packet length.
    m_packetLen = sendByte;
    m_at = 0;
    m_checkSum = 0x55 + m_packetLen;
    m_state = 2;
    break;
  case 2: // Data
    m_checkSum += sendByte;
    m_data[m_at] = sendByte;
    m_at++;
    if(m_at >= m_packetLen)
      m_state = 3;
    break;
  case 3: { // CRC 1
    uint8_t cs1 = (m_checkSum & 0xff);
    //RavlDebug("Checksum1 : %d %d ",(int)  cs1 , (int) sendByte);
    if(cs1 != sendByte) {
      //RavlDebug("Checksum failed. ");
      g_checksumFailure++;
      if(sendByte == g_charSTX)
        m_state = 1;
      else
        m_state = 0;
      break;
    }

    m_state = 4;
  } break;
  case 4: { // CRC 1
    uint8_t cs2 = ((m_checkSum >> 8) & 0xff);

    if(cs2 != sendByte) {
      g_checksumFailure++;
      //RavlDebug("Checksum failed. ");
      if(sendByte == g_charSTX)
        m_state = 1;
      else
        m_state = 0;
      break;
    }

    m_state = 5;
  } break;
  case 5: // ETX.
    if(sendByte == g_charETX) {
      //RavlDebug("Got packet!");
      ProcessPacket();
    } else {
      //      RavlDebug("Packet corrupted.");
    }
    m_state = 0;
    break;
  }
}

void SendError(int code,int datav)
{
  char data[16];
  uint8_t at = 0;
  data[at++] = 4; // Error
  data[at++] = code; // Error code
  data[at++] = datav;
  data[at++] = g_packetCount;
  SendPacket(data,at);
}

//! Process received packet.
void SerialDecodeC::ProcessPacket()
{
  // m_data[0] //
  g_packetCount++;
  switch(m_data[0])
  {
    case 1: { // Ping.
      char data[16];
      uint8_t at = 0;
      data[at++] = 2; // Pong.
      SendPacket(data,at);
      break;
    }
    case 2: { // Pong.
      break;
    }
    case 3: { // Servo position set.
      if(m_packetLen < 4) {
        SendError(1,m_packetLen);
        break;
      }
      switch(m_data[1])
      {
        case 0:
          g_targetPosition = ((int)m_data[2])  + ((int) m_data[3] << 8);
          g_servoMode = SM_Position;
          //myservo1.write(m_data[2]);
          break;
        default:
          SendError(2,m_data[1]);
          break;
      }
      break;
    }
    case 6: { // Position / Force mode
      if(m_packetLen < 8) {
        SendError(1,m_packetLen);
        break;
      }
      switch(m_data[1])
      {
        case 0: g_servoMode = SM_Position; break;
        case 1: g_servoMode = SM_Force; break;
        case 2: g_servoMode = SM_Free; break;
        default:
          g_servoMode = SM_Free; // Error.
          SendError(3,m_data[1]);
          break;
      }
      g_targetPosition = ((int)m_data[2])  + ((int) m_data[3] << 8);
      int16_t targetForce = ((int)m_data[4])  + ((int) m_data[5] << 8);
      g_velocityLimit = ((int)m_data[6])  + ((int) m_data[7] << 8);

      // Limit force before writing it, in case we get interrupted.
      if(targetForce > g_forceLimit)
        targetForce = g_forceLimit;
      g_targetForce = targetForce;

    } break;

  }
}


SerialDecodeC g_coms;

void SendSync()
{
  char buff[6];
  uint8_t at = 0;
  buff[at++] = 1; // Address
  buff[at++] = 3; // Type
  SendPacket(buff,at);

}

int main()
{
  InitIO();

  // Make sure interrupts are enabled.
  sei();

  OCR1B = 10;    // Change at.

  // PD2  O - Ain1
  // PD3  O - Standby
  // PD4  O - Ain2

  //g_servoMode = SM_Free;

  while(true) {

    if(g_haveSample) {
      g_haveSample = 0;

      char data[16];
      int at = 0;
      data[at++] = 5;               // 0 State
      data[at++] = g_lastPosition;  // 1
      data[at++] = g_lastPosition >> 8; // 2
      data[at++] = g_lastCurrent;   // 3
      data[at++] = g_lastCurrent >> 8; // 4
      data[at++] = g_velocityEstimate; // 5
      data[at++] = g_velocityEstimate >> 8;  // 6
      data[at++] = g_velocityEstimate >> 16; // 7
      data[at++] = g_velocityEstimate >> 24; // 8
      data[at++] = g_atGoal; // At goal.    9
      data[at++] = g_stalled; // Stalled.   10

      SendPacket(data,at);
    }

    // Ready to receive a byte?
    if ( (UCSR0A & _BV(RXC0)) ) {
      /* Get and return received data from buffer */
      g_coms.AcceptByte(UDR0);
      continue;
    }
    // Ready to transmit a byte?
    if( ( UCSR0A & _BV(UDRE0)) ) {
     /// UDR0 = 0x55;
      if(!g_txBuff.IsEmpty()) {
        /* Put data into buffer, sends the data */
        UDR0 = g_txBuff.Get();
        continue;
      }
    }

  }

  return 0;
}
