import React from 'react';

interface Tower {
  id: number;
  block1: string;
  block2: string;
  block3: string;
}

const QueueDisplay: React.FC<{ queue: Tower[] }> = ({ queue }) => {
  return (
    <div className="queue-display">
      <h2>Building Queue</h2>
      {queue.length === 0 ? (
        <p>No towers in the queue.</p>
      ) : (
        <ul>
          {queue.map((tower) => (
            <li key={tower.id}>
              Tower {tower.id}: {tower.block1}, {tower.block2}, {tower.block3}
            </li>
          ))}
        </ul>
      )}
    </div>
  );
};

export default QueueDisplay;