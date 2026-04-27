#include <Arduino.h>
// Address BUS Pins
// Used Hardware SPI
#define STCP PB2 // Pin 10 on Arduino Uno/Nano
// Pin 11 to send data
// Pin 13 the serial clock

// Enable Pins
#define WE PC0 // Pin A0 on Arduino Uno/Nano
#define OE PC1 // Pin A1 on Arduino Uno/Nano
#define CE PC2 // Pin A2 on Arduino Uno/Nano

// Data BUS Pins: D2-9

/*
 * ===========================================================================
 * Project: NitroBurner - EEPROM Programmer
 * Version: 1.0
 * Target Hardware: ATmega328P (Arduino Uno / Nano)
 * Storage Device: AT28C256 (32KB Parallel EEPROM)
 * ===========================================================================
 * * HARDWARE MAPPING:
 * -----------------
 *
 * * ADDRESS BUS (15-bit): Controlled via two 74HC595 Shift Registers.
 * - Latch (STCP): PB2 (Pin 10)
 * - Data (MOSI):  PB3 (Pin 11) -> Hardware SPI
 * - Clock (SCK):  PB5 (Pin 13) -> Hardware SPI
 *
 * * DATA BUS (8-bit): Parallel connection to EEPROM I/O 0-7.
 * - Bits 0-5: PORTD bits 2-7 (Pins 2-7)
 * - Bits 6-7: PORTB bits 0-1 (Pins 8-9)
 *
 * * CONTROL LINES:
 * - WE (Write Enable): PC0 (Pin A0)
 * - OE (Output Enable): PC1 (Pin A1)
 * - CE (Chip Enable):   PC2 (Pin A2)
 *
 * * REGISTER REFERENCE (ATmega328P):
 * --------------------------------
 * DDRx  - Data Direction Register (1 = Output, 0 = Input)
 * PORTx - Output Latch (High/Low). In Input mode, 1 enables Internal Pull-up (we will set it to zero).
 * PINx  - Input Register (Read-only). To read the data on data bus.
 *
 * * MODE ARCHITECTURE:
 * ------------------
 * - WRITE MODE: DDR set to 1. WE pulsed LOW.
 * - READ MODE:  DDR set to 0. PORT set to 0 (Hi-Z). Ensures EEPROM output
 * is the only signal on the bus, avoiding bus contention.
 * ===========================================================================
 */

// CORE FUNCTIONS
void setDataBusInput()
{
    DDRD &= ~0xFC;  // Set D2-D7 as INPUT (0b00000011)
    DDRB &= ~0x03;  // Set D8-D9 as INPUT (0b11111100)
    PORTD &= ~0xFC; // Disable pull-up resistors on D2-D7
    PORTB &= ~0x03; // Disable pull-up resistors on D8-D9
}

void setDataBusOutput()
{
    DDRD |= 0xFC; // Set D2-D7 as OUTPUT (0b11111100)
    DDRB |= 0x03; // Set D8-D9 as OUTPUT (0b00000011)
}

void setAddress(uint16_t addr)
{
    // Drop the Latch before shifting the address
    PORTB &= ~(1 << STCP);

    // Send the High Byte (A8 - A15)
    // shift right by 8-bits to get the upper half
    SPDR = (uint8_t)(addr >> 8);

    // Wait for the transfer to complete
    while (!(SPSR & (1 << SPIF)))
        ;

    // Send the Low Byte (A0 - A7)
    SPDR = (uint8_t)(addr);

    // Wait for the transfer to complete
    while (!(SPSR & (1 << SPIF)))
        ;

    // Raise the Latch to latch on the output pins of the shift registers
    PORTB |= (1 << STCP);
}

// BASIC FUNCTIONS
uint8_t readRaw(uint16_t addr)
{
    setAddress(addr);
    __asm__("nop");
    uint8_t data = 0;
    // Read the data from the Data Bus (D2-D7, D8-D9)
    data |= ((PIND >> 2) & 0x3F); // Shift D2-D7 down to bits 0-5
    data |= ((PINB << 6) & 0xC0); // Shift D8-D9 up to bits 6-7
    return data;
}

void writeRaw(uint16_t addr, uint8_t data)
{
    setAddress(addr);
    PORTD = (PORTD & 0x03) | ((data << 2) & 0xFC); // Place data on D2-D7
    PORTB = (PORTB & 0xFC) | ((data >> 6) & 0x03); // Place data on D8-D9
    __asm__("nop");
    PORTC &= ~(1 << WE); // WE LOW (Active)
    __asm__("nop");      // Short delay to ensure WE LOW
    PORTC |= (1 << WE);  // WE HIGH (Inactive)
}

int8_t pageWrite(uint16_t startAddr, uint8_t *data, uint8_t length)
{
    if (startAddr > 0x7FFF)
        return 0; // Address out of range
    if ((startAddr % 64) != 0)
        return -1; // Inalignment error: page address start of as a multiple of 64
    if (length == 0 || length > 64)
        return -2; // Invalid length

    setDataBusOutput();
    PORTC |= (1 << OE);  // OE HIGH (Inactive)
    PORTC |= (1 << WE);  // WE HIGH (Inactive)
    PORTC &= ~(1 << CE); // CE LOW (Active)

    uint8_t bytesWritten = 0;
    for (; bytesWritten < length; bytesWritten++)
        writeRaw(startAddr + bytesWritten, data[bytesWritten]);

    delay(11);          // tWC = 10ms
    PORTC |= (1 << CE); // CE HIGH (Inactive)
    return bytesWritten;
}

// UTILITY FUNCTIONS
void SDPUnlock()
{
    PORTC &= ~(1 << CE); // CE LOW (Active)
    setDataBusOutput();
    PORTC |= (1 << OE); // OE HIGH (Inactive)
    PORTC |= (1 << WE); // WE HIGH (Inactive)

    // unlock sequence
    writeRaw(0x5555, 0xAA);
    writeRaw(0x2AAA, 0x55);
    writeRaw(0x5555, 0x80);
    writeRaw(0x5555, 0xAA);
    writeRaw(0x2AAA, 0x55);
    writeRaw(0x5555, 0x20);
    delay(11);
    PORTC |= (1 << CE); // CE HIGH (Inactive)
}

void SDPlock()
{
    setDataBusInput();
    PORTC &= ~(1 << CE);  // CE LOW (Active)
    PORTC |= (1 << WE);   // WE HIGH (Inactive)
    PORTC &= ~(1 << OE);  // OE LOW (Active)
    delayMicroseconds(1); // to stabilize the bus
    uint8_t data = readRaw(0x0000);

    PORTC |= (1 << OE); // OE HIGH (Inactive)
    setDataBusOutput();

    // lock sequence
    writeRaw(0x5555, 0xAA);
    writeRaw(0x2AAA, 0x55);
    writeRaw(0x5555, 0xA0);
    writeRaw(0x0000, data);
    delay(11);          // tWC = 10ms
    PORTC |= (1 << CE); // CE HIGH (Inactive)
}

void read(uint16_t start, uint16_t stop)
{
    PORTC &= ~(1 << CE); // CE LOW (Active)
    setDataBusInput();
    PORTC |= (1 << WE);  // WE HIGH (Inactive)
    PORTC &= ~(1 << OE); // OE LOW (Active)

    bool firstRead = true;
    char format[64];
    bool patternPrinted = false;
    uint8_t prevData[16];
    uint8_t currentData[16];

    for (uint16_t address = start; address <= stop; address++)
    {
        uint8_t data = readRaw(address);
        if (firstRead)
        {
            prevData[address % 16] = data;
            if (address % 16 == 15)
            {
                firstRead = false;
                sprintf(format, "0x%04X : %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n", address - 15, prevData[0], prevData[1], prevData[2], prevData[3], prevData[4], prevData[5], prevData[6], prevData[7], prevData[8], prevData[9], prevData[10], prevData[11], prevData[12], prevData[13], prevData[14], prevData[15]);
                Serial.print(format);
            }
        }
        else
        {
            currentData[address % 16] = data;
            if (address % 16 == 15)
            {
                // Compare currentData with prevData
                bool currentMatch = memcmp(currentData, prevData, 16) == 0;
                if (currentMatch)
                {
                    patternPrinted = true;
                    if (address == stop)
                    {
                        char format[16];
                        sprintf(format, "0x%04X : *\n", address - 15);
                        Serial.print(format);
                    }
                }
                else
                {
                    if (patternPrinted)
                    {
                        patternPrinted = false;
                        char format[16];
                        sprintf(format, "0x%04X : *\n", address - 31);
                        Serial.print(format);
                    }
                    sprintf(format, "0x%04X : %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n", address - 15, currentData[0], currentData[1], currentData[2], currentData[3], currentData[4], currentData[5], currentData[6], currentData[7], currentData[8], currentData[9], currentData[10], currentData[11], currentData[12], currentData[13], currentData[14], currentData[15]);
                    Serial.print(format);
                    // Copy currentData to prevData for the next comparison
                    memcpy(prevData, currentData, 16);
                }
            }
        }
    }
    PORTC |= (1 << CE);                                                          // CE HIGH (Inactive)                                                     // CE HIGH (Inactive)
    Serial.println("======================Read Complete======================"); // Final newline after reading is done
}

void blank(uint16_t start = 0x0000, uint16_t stop = 0x7fff)
{
    PORTC &= ~(1 << CE); // CE LOW (Active)
    setDataBusOutput();
    PORTC |= (1 << OE); // OE HIGH (Inactive)
    PORTC |= (1 << WE); // WE HIGH (Inactive)

    uint16_t totalBytes = stop - start + 1;
    int barWidth = 40, pos = 0; // Blanking: [/40/] 000%
    float progress = 0;

    uint8_t page[64]; // Buffer for page writes, pre-filled with 0xFF
    memset(page, 0xFF, sizeof(page));

    if (start % 64 != 0)
    {
        // If start address is not page-aligned, we need to write bytes one by one until we reach the next page boundary
        for (; start < stop && (start % 64) != 0; start++)
        {
            writeRaw(start, 0xFF);
            delay(11); // tWC = 10ms
        }
    }

    if (stop % 64 != 63)
    {
        // If stop address is not the end of a page, we need to write bytes one by one from the last page boundary until we reach stop
        for (; stop > start && (stop % 64) != 63; stop--)
        {
            writeRaw(stop, 0xFF);
            delay(11); // tWC = 10ms
        }
    }

    for (uint16_t address = start; address <= stop; address += 64)
    {
        pageWrite(address, page, 64); // Write a full page of 0xFF
        progress = (float)(address - start) / totalBytes;
        pos = progress * barWidth;
        Serial.print("\e[2K\e[GBlanking: [");
        for (int i = 0; i < barWidth; i++)
        {
            if (i < pos)
                Serial.print("-");
            else if (i == pos)
                Serial.print(">");
            else
                Serial.print(".");
        }
        char format[8];
        sprintf(format, "] %03d%%", (int)round(progress * 100));
        Serial.print(format);
    }
    PORTC |= (1 << CE); // CE HIGH (Inactive)
    Serial.println("\n====================Blanking Complete====================");
}

void write(uint16_t addr, uint8_t data)
{
    PORTC &= ~(1 << CE); // CE LOW (Active)
    setDataBusOutput();
    PORTC |= (1 << OE); // OE HIGH (Inactive)
    PORTC |= (1 << WE); // WE HIGH (Inactive)
    writeRaw(addr, data);
    delay(11);          // tWC = 10ms
    PORTC |= (1 << CE); // CE HIGH (Inactive)
}

uint8_t read(uint16_t addr)
{
    PORTC &= ~(1 << CE); // CE LOW (Active)
    setDataBusInput();
    PORTC |= (1 << WE);   // WE HIGH (Inactive)
    PORTC &= ~(1 << OE);  // OE LOW (Active)
    delayMicroseconds(1); // to stabilize the bus
    uint8_t data = readRaw(addr);
    PORTC |= (1 << CE) | (1 << OE); // CE & OE HIGH (Inactive)
    return data;
}

void writeCmd()
{
    uint8_t buffer[64];
    uint8_t checksum = 0;
    uint32_t startTime = millis();

    // Read the 16-bit Address (High byte first)
    while (Serial.available() < 2)
        if (millis() - startTime > 1000)
        {
            Serial.println("ERR: ADDR_TIMEOUT");
            return;
        }

    uint16_t addr = (uint16_t)Serial.read() << 8;
    addr |= Serial.read();

    // Read 64 bytes of raw binary data
    for (int i = 0; i < 64; i++)
    {
        while (Serial.available() == 0)
            if (millis() - startTime > 2000)
            {
                Serial.println("ERR: DATA_TIMEOUT");
                return;
            }
        buffer[i] = Serial.read();
        checksum += buffer[i]; // Additive checksum
    }

    // Read the Checksum byte
    while (Serial.available() == 0)
        if (millis() - startTime > 1000)
        {
            Serial.println("ERR: CHK_TIMEOUT");
            return;
        }
    uint8_t receivedChecksum = Serial.read();

    // Verify and Commit
    if (checksum == receivedChecksum)
    {
        int8_t result = pageWrite(addr, buffer, 64);
        if (result == 64)
        {
            Serial.println("OK ACK");
        }
        else
        {
            Serial.print("ERR: PAGEWRITE_CODE_");
            Serial.println(result);
            if (result == 0)
                Serial.println("ERR: ADDR_OUT_OF_RANGE");
            else if (result == -1)
                Serial.println("ERR: ADDR_NOT_PAGE_ALIGNED");
            else if (result == -2)
                Serial.println("ERR: INVALID_LENGTH");
        }
    }
    else
    {
        Serial.print("ERR: CHK_MISMATCH (Calc:");
        Serial.print(checksum, HEX);
        Serial.print(" Recv:");
        Serial.print(receivedChecksum, HEX);
        Serial.println(")");
    }
}

/* * ===========================================================================
 * SPI SPEED CONFIGURATION CHART (F_osc = 16MHz)
 * ===========================================================================
 * To change speed, update SPCR and SPSR as shown below:
 * * SPEED  |  DIVIDER  |   SPCR (SPR1, SPR0)   |  SPSR (SPI2X) |  STABILITY
 * ---------|-----------|-----------------------|---------------|------------
 * 8 MHz    |    /2     |   0b01010000 (0,0)    |      1        |  PCB Only
 * 4 MHz    |    /4     |   0b01010000 (0,0)    |      0        |  Short Wire
 * 2 MHz    |    /8     |   0b01010001 (0,1)    |      1        |  Reliable
 * 1 MHz    |    /16    |   0b01010001 (0,1)    |      0        |  ★ DEFAULT
 * 500 kHz  |    /32    |   0b01010010 (1,0)    |      1        |  Very Safe
 * 250 kHz  |    /64    |   0b01010010 (1,0)    |      0        |  Ultra Safe
 * ---------------------------------------------------------------------------
 * Note: Bit 6 (SPE) and Bit 4 (MSTR) must ALWAYS be 1 in SPCR.
 */

void setup()
{
    // Control Pins
    PORTC |= 0x07; // Inactive State for Control PINS
    DDRC |= 0x07;  // Set A0-A2 as OUTPUT (0b00000111) Control PINS

    // setting up serial communication
    Serial.begin(115200);
    while (!Serial)
        ;
    delay(2000); // Give the user a moment to open the Serial Monitor
    Serial.println("System Starting...");

    // Setting Up Hardware SPI
    DDRB |= 0b00101100; // Set D10 (SS), D11 (MOSI), D13 (SCK) as OUTPUT ADDR PINS
    SPCR = 0b01010001;  // SPE = 1 (Enable SPI), MSTR = 1 (Master Mode), Clock = F_osc/16 = 1MHz
    SPSR = 0x00;        // SPI2X = 0

    setAddress(0x0000); // Clear any garbage on the shift registers

    PORTC &= ~(1 << CE); // CE LOW (Active)
    PORTC |= (1 << WE);  // WE HIGH (Inactive)
    PORTC &= ~(1 << OE); // OE LOW (Active)
    Serial.println("Awaiting Commands...");
}

void loop()
{
    while (Serial.available() > 0)
    {
        char cmd = Serial.read();
        // v-> version, b -> blank, r -> read, w -> write, R -> sys read, d -> disable, e-> enable
        switch (cmd)
        {
        case 'v':
            Serial.println("NitroBurner v1.0 --- Active");
            break;
        case 'b':
            blank();
            break;
        case 'r':
            read(0x0000, 0x7fff);
            break;
        case 'w':
            writeCmd();
            break;
        case 'R': // R for system read
            setDataBusInput();
            PORTC &= ~(1 << CE);
            PORTC &= ~(1 << OE);
            PORTC |= (1 << WE);
            for (uint32_t i = 0; i <= 0x7FFF; i++)
            {
                Serial.write(readRaw(i)); // Send raw binary byte
                delayMicroseconds(1);     // Short delay to prevent overwhelming the serial buffer
            }
            PORTC |= (1 << CE);
            PORTC |= (1 << OE);
            break;
        case 'd':
            SDPUnlock();
            break;
        case 'e':
            SDPlock();
            break;
        default:
            Serial.println("Unknown command.");
            break;
        }
    }
}