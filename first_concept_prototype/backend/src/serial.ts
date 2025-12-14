import { SerialPort } from 'serialport';
import { ReadlineParser } from '@serialport/parser-readline';

// Serial Port Configuration
const serialPort = new SerialPort({
  path: 'COM3', // Adjust this to your actual port
  baudRate: 9600,
});

// Parser Configuration to Read Full Lines
const parser = serialPort.pipe(new ReadlineParser({ delimiter: '\n' }));

// Export the serial port and parser for use in other files
export { serialPort, parser };