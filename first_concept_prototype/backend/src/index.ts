import express from 'express';
import cors from 'cors';
import dotenv from 'dotenv';
import { WebSocketServer, WebSocket } from 'ws';
import { SerialPort } from 'serialport';
import { ReadlineParser } from '@serialport/parser-readline';
import sqlite3 from 'sqlite3';

// Initialize SQLite Database
const db = new sqlite3.Database('./orders.db');

// // Reset the database
// db.serialize(() => {
//   db.run('DELETE FROM orders;');
//   db.run('DELETE FROM inventory;');
//   db.run('INSERT INTO inventory (color, quantity) VALUES (?, ?)', ['Red', 5]);
//   db.run('INSERT INTO inventory (color, quantity) VALUES (?, ?)', ['Green', 5]);
//   db.run('INSERT INTO inventory (color, quantity) VALUES (?, ?)', ['Blue', 5]);
//   db.run('UPDATE sqlite_sequence SET seq = 1 WHERE name = \'orders\';');
// });

// Create a table to store orders if it doesn't exist
db.serialize(() => {
  // cSpell:disable
  db.run(`
    CREATE TABLE IF NOT EXISTS orders (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      block1 TEXT,
      block2 TEXT,
      block3 TEXT,
      built BOOLEAN DEFAULT FALSE
    );
  `, (err) => {
    if (err) {
      console.error('Error creating table:', err);
    } else {
      console.log('Table "orders" created or already exists.');
    }
  });
  // cSpell:disable
});

// Create a table to store inventory if it doesn't exist
db.serialize(() => {
  db.run(`
    CREATE TABLE IF NOT EXISTS inventory (
      color TEXT PRIMARY KEY,
      quantity INTEGER
    );
  `, (err) => {
    if (err) {
      console.error('Error creating table:', err);
    } else {
      console.log('Table "inventory" created or already exists.');
    }
  });
});

// Initialize inventory in the database if it's empty
db.all('SELECT * FROM inventory;', (err, rows) => {
  if (err) {
    console.error('Error loading inventory from database:', err);
  } else {
    if (rows.length === 0) {
      const initialInventory = [
        { color: 'Red', quantity: 3 },
        { color: 'Green', quantity: 3 },
        { color: 'Blue', quantity: 3 },
      ];
      initialInventory.forEach((item) => {
        db.run('INSERT INTO inventory (color, quantity) VALUES (?, ?)', [item.color, item.quantity], (err) => {
          if (err) {
            console.error('Error initializing inventory:', err);
          }
        });
      });
      console.log('Inventory initialized successfully');
    }
  }
});

// Load initial queue from the database
db.all('SELECT * FROM orders WHERE built = 0;', (err, rows) => {
  if (err) {
    console.error('Error loading queue from database:', err);
  } else {
    rows.forEach((row: unknown) => {
      const rowObject = row as { [key: string]: string };
      const existingOrder = towerQueue.find((order) => order.id === parseInt(rowObject.id));
      if (!existingOrder) {
        towerQueue.push({
          id: parseInt(rowObject.id),
          block1: rowObject.block1,
          block2: rowObject.block2,
          block3: rowObject.block3,
        });
      }
    });
    console.log('Initial queue loaded from database:', towerQueue);
  }
});

// Update inventory in the database
const updateInventory = (color: string, quantity: number) => {
  db.run('UPDATE inventory SET quantity = ? WHERE color = ?', [quantity, color], (err) => {
    if (err) {
      console.error('Error updating inventory:', err);
    }
  });
};

// Process Queue
const processQueue = async () => {
  if (!isFactoryRunning || towerQueue.length === 0) return; // Pause if factory is stopped or queue is empty

  const nextTower = towerQueue[0]; // Get the first tower in the queue
  const blocks = [nextTower.block1, nextTower.block2, nextTower.block3];

  // Check if there are enough blocks
  if (inventory[nextTower.block1] > 0 && inventory[nextTower.block2] > 0 && inventory[nextTower.block3] > 0) {
    // Deduct blocks used
    inventory[nextTower.block1]--;
    inventory[nextTower.block2]--;
    inventory[nextTower.block3]--;

import { setupWebSocket } from './websocket'; // Import the WebSocket setup function


    // Update inventory in the database
    updateInventory(nextTower.block1, inventory[nextTower.block1]);
    updateInventory(nextTower.block2, inventory[nextTower.block2]);
    updateInventory(nextTower.block3, inventory[nextTower.block3]);

    // Send information to the Arduino about the blocks used
    const commandToArduino = `TOWER, ${nextTower.block1.charAt(0)},${nextTower.block2.charAt(0)},${nextTower.block3.charAt(0)}\n`;
    serialPort.write(commandToArduino);

    // Notify all clients about updated inventory
    wss.clients.forEach((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(JSON.stringify(inventory));
      }
    });
    console.log('Tower being constructed:', nextTower);

// Update the order as built
db.run('UPDATE orders SET built = 1 WHERE id = ?', [nextTower.id], (err) => {
  if (err) {
    console.error('Error updating order:', err);
  } else {
    console.log('Order updated as built');
  }
});

// Remove the tower from the queue
const existingTowerIndex = towerQueue.findIndex((tower) => tower.id === nextTower.id);
if (existingTowerIndex !== -1) {
  towerQueue.splice(existingTowerIndex, 1);
}

    // Process the next tower in the queue
    processQueue();
  } else {
    console.log('Insufficient blocks to construct the tower');
    // Wait for 1 second before checking again
    setTimeout(() => {
      processQueue();
    }, 1000);
  }
};
// Express server setup
const app = express();

// Enable CORS
app.use(cors({
  origin: 'http://localhost:3000', // Allow requests from your frontend server
  methods: ['GET', 'POST', 'PUT', 'DELETE'], // Allow specific HTTP methods
  credentials: true, // Allow credentials (e.g., cookies) to be sent with requests
}));

app.use(express.json()); // Parse JSON request bodies

// Load environment variables
dotenv.config();

// WebSocket server setup
const wss = new WebSocketServer({ port: 8080 });
console.log('WebSocket server running at ws://localhost:8080');

// Serial Port Configuration
const serialPort = new SerialPort({
  path: 'COM3',
  baudRate: 9600,
});

// Parser Configuration to Read Full Lines
const parser = serialPort.pipe(new ReadlineParser({ delimiter: '\n' }));

// Block Inventory
interface Inventory {
  [key: string]: number;
  Red: number;
  Green: number;
  Blue: number;
}

let inventory: Inventory = {
  Red: 3,
  Green: 3,
  Blue: 3,
};

// Tower Queue
let towerQueue: any[] = [];

// Factory State
let isFactoryRunning = false;

// Load initial queue from the database
db.all('SELECT * FROM orders WHERE built = 0;', (err, rows) => {
  if (err) {
    console.error('Error loading queue from database:', err);
  } else {
    rows.forEach((row: unknown) => {
      const rowObject = row as { [key: string]: string };
      towerQueue.push({
        id: parseInt(rowObject.id),
        block1: rowObject.block1,
        block2: rowObject.block2,
        block3: rowObject.block3,
      });
    });
    console.log('Initial queue loaded from database:', towerQueue);
  }
});

// Handle WebSocket Connection
wss.on('connection', (ws: WebSocket) => {
  console.log('Client connected to WebSocket');

  // Send initial inventory to the client
  ws.send(JSON.stringify(inventory));

  // Handle incoming messages from the client
  ws.on('message', (message: string) => {
    try {
      const towerConfig = JSON.parse(message);

      // Ensure the action is correct
      if (towerConfig.action === "buildTower") {
        // Send INIT command to Arduino
        serialPort.write("INIT");

        // Access the block array
        const blocks = towerConfig.blocks;

        // Validate blocks
        if (!blocks || !Array.isArray(blocks) || blocks.length < 3) {
          console.log('Invalid message format or not enough blocks provided');
          return; // Exit if the message format is invalid
        }

        // Assign blocks to block 1-2-3
        const [block1, block2, block3] = blocks;

        // Add the tower to the queue
        towerQueue.push({
          id: Date.now(), // Use timestamp as a unique ID
          block1: block1,
          block2: block2,
          block3: block3,
        });

        // Save the order to the database
        db.run(
          'INSERT INTO orders (block1, block2, block3, built) VALUES (?, ?, ?, ?)',
          [block1, block2, block3, 0],
          function (err) {
            if (err) {
              console.error('Error saving order to database:', err);
            } else {
              console.log(`Order saved to database with ID: ${this.lastID}`);
            }
          }
        );

        // Send the update message to all clients
        wss.clients.forEach((client) => {
          if (client.readyState === WebSocket.OPEN) {
            client.send(JSON.stringify({ action: 'updateQueue', queue: towerQueue }));
          }
        });

        // Check if enough blocks are available
        if (inventory[block1] > 0 && inventory[block2] > 0 && inventory[block3] > 0) {
          // Deduct blocks used
          inventory[block1]--;
          inventory[block2]--;
          inventory[block3]--;

          // Update inventory in the database
          updateInventory(block1, inventory[block1]);
          updateInventory(block2, inventory[block2]);
          updateInventory(block3, inventory[block3]);

          // Send information to the Arduino about the blocks used
          const commandToArduino = `TOWER,${block1.charAt(0)},${block2.charAt(0)},${block3.charAt(0)}\n`;
          serialPort.write(commandToArduino);

          // Notify all clients about updated inventory
          wss.clients.forEach((client) => {
            if (client.readyState === WebSocket.OPEN) {
              client.send(JSON.stringify(inventory));
            }
          });

          console.log('Tower being constructed:', towerConfig);

          // Update the order as built
          db.run('UPDATE orders SET built = 1 WHERE id = ?', [towerQueue[0].id], (err) => {
            if (err) {
              console.error('Error updating order:', err);
            } else {
              console.log('Order updated as built');
            }
          });

          // Remove the tower from the queue
          towerQueue.shift();

          // Process the next tower in the queue
          if (towerQueue.length > 0) {
            processQueue();
          }
        } else {
          console.log('Insufficient blocks to construct the tower');
          isFactoryRunning = false;
          // Do not remove the tower from the queue
        }
      } else {
        console.log('Invalid action in message');
      }
    } catch (error) {
      console.error('Error processing client message:', error);
    }
  });
});

// Endpoint to get the queue
app.get('/queue', (req, res) => {
  db.all('SELECT * FROM orders WHERE built = 0;', (err, rows) => {
    if (err) {
      console.error('Error fetching queue:', err);
      res.status(500).json({ message: 'Error fetching queue' });
    } else {
      res.json(rows);
    }
  });
});

// Endpoint to update the queue
app.post('/queue', (req, res) => {
  const queue = req.body.queue;
  db.run('DELETE FROM orders;', (err) => {
    if (err) {
      console.error('Error deleting queue:', err);
      res.status(500).json({ message: 'Error deleting queue' });
    } else {
      queue.forEach((tower: any) => {
        db.run('INSERT INTO orders (block1, block2, block3, built) VALUES (?, ?, ?, ?)', [tower.block1, tower.block2, tower.block3, 0], (err) => {
          if (err) {
            console.error('Error saving queue:', err);
            res.status(500).json({ message: 'Error saving queue' });
          }
        });
      });
      res.json({ message: 'Queue updated successfully' });
    }
  });
});

// Endpoint to get the inventory
app.get('/inventory', (req, res) => {
  res.json(inventory);
});

// Endpoint to start the factory
app.post('/start-factory', (req, res) => {
  isFactoryRunning = true;
  serialPort.write('START\n', (err) => {
    if (err) {
      console.error('Error sending START command to Arduino:', err);
      return res.status(500).json({ message: 'Failed to start factory' });
    } else {
      console.log('START command sent to Arduino successfully');
      return res.status(200).json({ message: 'Factory started' });
    }
  });
});

// Endpoint to stop the factory
app.post('/stop-factory', (req, res) => {
  isFactoryRunning = false;
  serialPort.write('STOP\n', (err) => {
    if (err) {
      console.error('Error sending STOP command to Arduino:', err);
      return res.status(500).json({ message: 'Failed to stop factory' });
    } else {
      console.log('STOP command sent to Arduino successfully');
      towerQueue = [];
      wss.clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
          client.send(JSON.stringify({ action: 'STOP' }));
        }
      });
      return res.status(200).json({ message: 'Factory stopped' });
    }
  });
});

// Setup WebSocket server
setupWebSocket(app); // Pass the Express app to the WebSocket setup

// Express API endpoint
app.get('/api', (req, res) => {
  console.log('Received a request to /api'); // Log the request
  console.log('Query parameters:', req.query); // Example of using req to access query parameters
  res.send('Hello from the backend!');
});

// Start the Express server
const PORT = process.env.PORT || 5000;
app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
});