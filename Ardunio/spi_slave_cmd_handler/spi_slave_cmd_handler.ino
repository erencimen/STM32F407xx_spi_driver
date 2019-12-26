/*

 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select
 *

 */

 #include <SPI.h>

 const byte led = 3;           // Slave LED digital I/O pin.

boolean ledState = HIGH;      // LED state flag.

uint8_t dataBuff[255];

uint8_t board_id[10] = "ARDUINOUN";

#define NACK 0xA5
#define ACK 0xF5


//command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

//Initialize SPI slave.
void SPI_SlaveInit(void)
{
  // Initialize SPI pins.
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);
  //make SPI as slave

  // Enable SPI as slave.
  SPCR = (1 << SPE);
}

uint8_t SPI_SlaveReceive(void)
{
  /* Wait for reception complete */
  while(!(SPSR & (1<<SPIF)));
  /* Return Data Register */
  return SPDR;
}

void SPI_SlaveTransmit(uint8_t data)
{
  /* Start transmission */
  SPDR = data;

  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)));
}

byte checkData(byte commnad)
{
  if(commnad == COMMAND_LED_CTRL || commnad == COMMAND_SENSOR_READ
    || commnad == COMMAND_LED_READ || commnad == COMMAND_PRINT
    || commnad == COMMAND_ID_READ)
    return ACK;
  return NACK;
}

void setup()
{
  // Initialize serial for troubleshooting.
  Serial.begin(9600);

  // Initialize slave LED pin.
  pinMode(led, OUTPUT);

  digitalWrite(led,LOW);

  // Initialize SPI Slave.
  SPI_SlaveInit();

  Serial.println("Slave Initialized");
}

void loop()
{
  byte data,command,len,ackornack=NACK;

  //Wait until ss is low
  Serial.println("Slave waiting for ss to go low");
  while(digitalRead(SS) );

  //Wait until rx buffer has a byte
  command = SPI_SlaveReceive();
  Serial.println(command);
  ackornack = checkData(command);

  SPI_SlaveTransmit(ackornack);

  len = SPI_SlaveReceive(); //dummy byte

  if(command == COMMAND_LED_CTRL)
  {
    //Read 2 more bytes pin number and value
    uint8_t pin = SPI_SlaveReceive();
    uint8_t value = SPI_SlaveReceive();
    Serial.println("RCVD:COMMAND_LED_CTRL");
    if(value == (uint8_t)LED_ON)
    {
      digitalWrite(pin,HIGH);
    }else if (value == (uint8_t) LED_OFF)
    {
      digitalWrite(pin,LOW);
    }

  }else if ( command == COMMAND_SENSOR_READ)
  {
    //Read analog pin number
    uint16_t aread;
    uint8_t pin = SPI_SlaveReceive();
    //PinMode(pin+14, INPUT_PULLUP);
    uint8_t val;
    aread = analogRead(pin+14);
    val = map(aread, 0, 1023, 0, 255);

    SPI_SlaveTransmit(val);

    val = SPI_SlaveReceive(); //dummy read

    Serial.println("RCVD:COMMAND_SENSOR_READ");



  }else if ( command == COMMAND_LED_READ)
  {
    uint8_t pin = SPI_SlaveReceive();
    uint8_t val = digitalRead(pin);
    SPI_SlaveTransmit(val);
    val = SPI_SlaveReceive(); //dummy read
    Serial.println("RCVD:COMMAND_LED_READ");

  }else if ( command == COMMAND_PRINT)
  {
    uint8_t len = SPI_SlaveReceive();
    for(int i=0 ; i < len ; i++)
    {
      dataBuff[i] = SPI_SlaveReceive();
    }
    Serial.println((char*)dataBuff);

    Serial.println("RCVD:COMMAND_PRINT");

  }else if ( command == COMMAND_ID_READ)
  {
    for(int i=0 ; i < 10 ; i++)
    {
      SPI_SlaveTransmit(board_id[i]);
    }
      SPI_SlaveReceive();
    Serial.println("RCVD:COMMAND_ID_READ");
  }
}
