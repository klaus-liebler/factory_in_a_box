import { WebSocketServer, WebSocket } from 'ws';
import { SerialPort } from 'serialport';
import { ReadlineParser } from '@serialport/parser-readline';
import express, { Request, Response } from 'express'; // Import Request and Response types

// Serial Port Configuration
const serialPort = new SerialPort({
  path: 'COM3', // Adjust this to your actual port
  baudRate: 9600,
});

// Parser Configuration to Read Full Lines
const parser = serialPort.pipe(new ReadlineParser({ delimiter: '\n' }));

// Block Inventory
interface Inventory {
  [key: string]: number;
}

let inventory: Inventory = {
  Red: 3,
  Green: 3,
  Blue: 3,
};

// Factory State
let isFactoryRunning = false; // Track if the factory is running

// Handle WebSocket Connection
export function setupWebSocket(app: express.Application) {
  const wss = new WebSocketServer({ port: 8080 });
  console.log('WebSocket server running at ws://localhost:8080');

  // Define a route to get the factory status
  app.get('/status', (_req: Request, res: Response) => { // Use underscore to indicate unused parameter
    res.json({ isFactoryRunning, inventory });
  });

  wss.on('connection', (ws: WebSocket) => {
    console.log('Client connected to WebSocket');

    // Send initial inventory and factory state to the client
    ws.send(JSON.stringify({ inventory, isFactoryRunning }));

    // Handle incoming messages from the client
    ws.on('message', (message: string) => {
      handleClientMessage(message, ws, wss); // Pass wss to the handler
    });
  });

  // Read Data from Serial Port (Arduino)
  parser.on('data', (data: string) => {
    handleArduinoData(data, wss); // Pass wss to the handler
  });

  // Handle Serial Port Errors
  serialPort.on('error', (error) => {
    console.error('Serial Port Error:', error);
  });
}

// Function to handle client messages
function handleClientMessage(message: string, ws: WebSocket, wss: WebSocketServer) {
  try {
    const towerConfig = JSON.parse(message);

    switch (towerConfig.action) {
      case "startFactory":
        isFactoryRunning = true;
        console.log('Factory started.');
        ws.send(JSON.stringify({ status: 'Factory started', isFactoryRunning }));
        break;

      case "stopFactory":
        isFactoryRunning = false;
        console.log('Factory stopped.');
        ws.send(JSON.stringify({ status: 'Factory stopped', isFactoryRunning }));
        break;

      case "buildTower":
        buildTower(towerConfig.blocks, ws, wss); // Pass wss to the buildTower function
        break;

      default:
        console.log('Invalid action in message');
        ws.send(JSON.stringify({ error: 'Invalid action' }));
    }
  } catch (error) {
    console.error('Error processing client message:', error);
    ws.send(JSON.stringify({ error: 'Invalid message format' }));
  }
}

// Function to build a tower
function buildTower(blocks: string[], ws: WebSocket, wss: WebSocketServer) {
  if (!isFactoryRunning) {
    console.log('Factory is not running. Cannot build tower.');
    ws.send(JSON.stringify({ error: 'Factory is not running. Cannot build tower.' }));
    return;
  }

  if (blocks.length < 3) {
    console.log('Not enough blocks provided');
    ws.send(JSON.stringify({ error: 'Not enough blocks provided' }));
    return;
  }

  const [block1, block2, block3] = blocks;

  if (inventory[block1] > 0 && inventory[block2] > 0 && inventory[block3] > 0) {
    inventory[block1]--;
    inventory[block2]--;
    inventory[block3]--;

    const commandToArduino = `TOWER,${block1.charAt(0)},${block2.charAt(0)},${ block3.charAt(0)}\n`;
    serialPort.write(commandToArduino, (err) => {
      if (err) {
        console.error('Error sending command to Arduino:', err);
      }
    });

    // Notify all clients about updated inventory
    notifyClients(wss);
    console.log('Tower being constructed:', { block1, block2, block3 });
  } else {
    console.log('Insufficient blocks to construct the tower');
    ws.send(JSON.stringify({ error: 'Insufficient blocks to construct the tower' }));
  }
}

// Function to handle data received from Arduino
function handleArduinoData(data: string, wss: WebSocketServer) {
  console.log('Data received from Arduino:', data);
  try {
    const parsedData = JSON.parse(data);
    inventory.Red = parsedData.red || inventory.Red;
    inventory.Green = parsedData.green || inventory.Green;
    inventory.Blue = parsedData.blue || inventory.Blue;

    // Notify all clients about updated inventory
    notifyClients(wss);
  } catch (error) {
    console.error('Error processing Arduino data:', error instanceof Error ? error.message : 'Unknown error');
  }
}

// Function to notify all WebSocket clients about inventory updates
function notifyClients(wss: WebSocketServer) {
  wss.clients.forEach((client: WebSocket) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify({ inventory, isFactoryRunning }));
    }
  });
}