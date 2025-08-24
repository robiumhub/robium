// Simple event system for notifying components when filter data changes
type FilterEventType = 'categories' | 'values' | 'both';

interface FilterEvent {
  type: FilterEventType;
  timestamp: number;
}

type FilterEventListener = (event: FilterEvent) => void;

class FilterEventManager {
  private listeners: FilterEventListener[] = [];

  subscribe(listener: FilterEventListener): () => void {
    this.listeners.push(listener);

    // Return unsubscribe function
    return () => {
      const index = this.listeners.indexOf(listener);
      if (index > -1) {
        this.listeners.splice(index, 1);
      }
    };
  }

  notify(event: FilterEvent): void {
    this.listeners.forEach((listener) => {
      try {
        listener(event);
      } catch (error) {
        console.error('Error in filter event listener:', error);
      }
    });
  }

  notifyCategoriesChanged(): void {
    this.notify({
      type: 'categories',
      timestamp: Date.now(),
    });
  }

  notifyValuesChanged(): void {
    this.notify({
      type: 'values',
      timestamp: Date.now(),
    });
  }

  notifyBothChanged(): void {
    this.notify({
      type: 'both',
      timestamp: Date.now(),
    });
  }
}

// Export singleton instance
export const filterEventManager = new FilterEventManager();

// Export types for use in other files
export type { FilterEventType, FilterEvent, FilterEventListener };
