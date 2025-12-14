import React from 'react';

interface Inventory {
    Red: number;
    Green: number;
    Blue: number;
}

const InventoryDisplay: React.FC<{ inventory: Inventory }> = ({ inventory }) => (
    <div className="inventory-section">
        <h2>Current Inventory</h2>
        <p>Red: {inventory.Red}</p>
        <p>Green: {inventory.Green}</p>
        <p>Blue: {inventory.Blue}</p>
    </div>
);

export default InventoryDisplay;