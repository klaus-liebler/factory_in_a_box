import React from 'react';

const StatusDisplay: React.FC<{ status: string }> = ({ status }) => (
    <div className="status-section">
        <h2>Status</h2>
        <p>{status}</p>
    </div>
);

export default StatusDisplay;