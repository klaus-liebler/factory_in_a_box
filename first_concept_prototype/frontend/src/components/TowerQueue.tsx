// src/hooks/useTowerQueue.tsx
import { useState, useEffect } from 'react';

export const useTowerQueue = (wsManagerRef: React.RefObject<{ sendMessage: (message: any) => void }>) => {
    const [queue, setQueue] = useState<{ block1: string; block2: string; block3: string }[]>([]);
    const [isBuilding, setIsBuilding] = useState<boolean>(false);

    const buildTower = (blocks: string[]) => {
        const towerConfig = { block1: blocks[0], block2: blocks[1], block3: blocks[2] };
        setQueue((prevQueue) => [...prevQueue, towerConfig]);
        processQueue();
    };

    const processQueue = () => {
        if (isBuilding || queue.length === 0) return;

        const nextTowerConfig = queue[0];
        if (wsManagerRef.current) {
            setIsBuilding(true);
            wsManagerRef.current.sendMessage(nextTowerConfig);

            // Simulate the building process (this will be replaced by the robotic arm logic)
            setTimeout(() => {
                completeTowerBuild(); // Simulate completion of the tower building
            }, 2000); // Simulate a delay for building the tower
        }
    };

    const completeTowerBuild = () => {
        // Remove the processed configuration from the queue
        setQueue((prevQueue) => prevQueue.slice(1)); // Remove the processed configuration
        setIsBuilding(false);
        processQueue(); // Process the next configuration
    };

    // Log the queue to the console whenever it changes
    useEffect(() => {
        console.log('Current Queue:', queue);
    }, [queue]);

    return { buildTower, queue, completeTowerBuild }; // Return the completeTowerBuild function
};