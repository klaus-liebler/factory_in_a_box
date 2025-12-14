import React from 'react';

interface BlockSelectorProps {
  block: string;
  setBlock: (block: string) => void;
}

const BlockSelector: React.FC<BlockSelectorProps> = ({ block, setBlock }) => {
  return (
    <label>
      Block:
      <select value={block} onChange={(e) => setBlock(e.target.value)}>
        <option value="Red">Red</option>
        <option value="Green">Green</option>
        <option value="Blue">Blue</option>
      </select>
    </label>
  );
};

export default BlockSelector;