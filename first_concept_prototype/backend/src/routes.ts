import { Router } from 'express';
import { serialPort } from './serial'; // Import the serial port if needed

const router = Router();

// New endpoint for factory control
router.post('/command', (req, res) => {
  const { command } = req.body;
  console.log(`Received command: ${command}`);

  try {
    switch (command) {
      case 'START':
        serialPort.write('START\n', (err) => {
          if (err) {
            console.error('Error sending START command:', err);
            return res.status(500).json({ error: 'Failed to send START command' });
          }
          res.json({ message: 'Factory process started' });
        });
        break;

      case 'STOP':
        serialPort.write('STOP\n', (err) => {
          if (err) {
            console.error('Error sending STOP command:', err);
            return res.status(500).json({ error: 'Failed to send STOP command' });
          }
          res.json({ message: 'Factory process stopped' });
        });
        break;

      default:
        res.status(400).json({ error: 'Invalid command' });
    }
  } catch (error) {
    console.error('Error processing command:', error);
    res.status(500).json({ error: 'Internal server error' });
  }
});

// Export the router
export default router;