import { useEffect, useState, useRef, forwardRef, useImperativeHandle, useCallback } from 'react';

interface Inventory {
  Red: number;
  Green: number;
  Blue: number;
}

const WebSocketManager = forwardRef<{ sendMessage: (message: any) => void; isConnected: boolean }, { setInventory: (inventory: Inventory) => void; setStatus: (status: string) => void; }>(({ setInventory, setStatus }, ref) => {
  const socketRef = useRef<WebSocket | null>(null);
  const [isConnected, setIsConnected] = useState<boolean>(false);

  const connectWebSocket = useCallback(() => {
    if (socketRef.current) {
      console.log('WebSocket connection already established');
      return;
    }

    const socket = new WebSocket('ws://localhost:8080');

    socket.onopen = () => {
      console.log('WebSocket connection established');
      setIsConnected(true);
      setStatus('WebSocket connection established');
    };

    socket.onmessage = (event) => {
      try {
        const updatedInventory = JSON.parse(event.data);
        setInventory(updatedInventory);
      } catch (error) {
        console.error("Received data is not valid JSON:", event.data);
        setStatus(`Received non-JSON message: ${event.data}`);
      }
    };

    socket.onclose = () => {
      console.log('WebSocket connection closed');
      setIsConnected(false);
      setStatus('WebSocket connection closed. Attempting to reconnect...');
      setTimeout(connectWebSocket, 3000); // Reconnect after 3 seconds
    };

    socket.onerror = (error) => {
      console.error('WebSocket error:', error);
      setStatus('WebSocket error occurred. Check console for details.');
      setIsConnected(false);
      setTimeout(connectWebSocket, 3000); // Reconnect after 3 seconds
    };

    socketRef.current = socket;
  }, [setInventory, setStatus]);

  useEffect(() => {
    connectWebSocket();

    return () => {
      if (socketRef.current) {
        socketRef.current.close();
      }
    };
  }, [connectWebSocket]);

  const sendMessage = (message: any) => {
    if (socketRef.current && isConnected) {
      socketRef.current.send(JSON.stringify(message));
    } else {
      console.error('WebSocket is not connected. Cannot send message:', message);
      setStatus('WebSocket is not connected. Cannot send message.');
      setTimeout(() => sendMessage(message), 1000); // Retry sending the message after 1 second
    }
  };

  useImperativeHandle(ref, () => ({
    sendMessage,
    isConnected,
  }));

  return null;
});

export default WebSocketManager;