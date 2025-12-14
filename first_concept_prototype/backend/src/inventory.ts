// Inventory management logic
interface Inventory {
    [key: string]: number;
  }
  
  let inventory: Inventory = {
    Red: 3,
    Green: 3,
    Blue: 3,
  };
  
  function updateInventory(block: string) {
    if (inventory[block] > 0) {
      inventory[block]--;
    }
  }
  
  function getInventory() {
    return inventory;
  }
  
  export { updateInventory, getInventory };