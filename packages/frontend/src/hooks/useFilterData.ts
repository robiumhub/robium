import { useState, useEffect, useCallback } from 'react';
import { FilterCategory, FilterValue } from '@robium/shared';
import { ApiService } from '../services/api';
import { filterEventManager, FilterEventType } from '../utils/filterEvents';

interface UseFilterDataOptions {
  isTemplate?: boolean;
  autoRefresh?: boolean;
  refreshInterval?: number;
  retryAttempts?: number;
  retryDelay?: number;
}

interface UseFilterDataReturn {
  categories: FilterCategory[];
  filterValues: FilterValue[];
  stats: Record<string, Record<string, number>>;
  loading: boolean;
  error: string | null;
  retryCount: number;
  refresh: () => Promise<void>;
  refreshCategories: () => Promise<void>;
  refreshValues: () => Promise<void>;
  refreshStats: () => Promise<void>;
  clearError: () => void;
}

export const useFilterData = (options: UseFilterDataOptions = {}): UseFilterDataReturn => {
  const {
    isTemplate = false,
    autoRefresh = false,
    refreshInterval = 30000,
    retryAttempts = 3,
    retryDelay = 1000,
  } = options;

  const [categories, setCategories] = useState<FilterCategory[]>([]);
  const [filterValues, setFilterValues] = useState<FilterValue[]>([]);
  const [stats, setStats] = useState<Record<string, Record<string, number>>>({});
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [retryCount, setRetryCount] = useState(0);

  // Utility function for retrying API calls
  const retryApiCall = useCallback(
    async <T>(
      apiCall: () => Promise<T>,
      maxAttempts: number = retryAttempts,
      delay: number = retryDelay
    ): Promise<T> => {
      let lastError: Error;

      for (let attempt = 1; attempt <= maxAttempts; attempt++) {
        try {
          return await apiCall();
        } catch (err) {
          lastError = err instanceof Error ? err : new Error('Unknown error');

          if (attempt === maxAttempts) {
            throw lastError;
          }

          // Wait before retrying
          await new Promise((resolve) => setTimeout(resolve, delay * attempt));
        }
      }

      throw lastError!;
    },
    [retryAttempts, retryDelay]
  );

  const refreshCategories = useCallback(async () => {
    try {
      const response = await retryApiCall(() => ApiService.getFilterCategories());
      if (response.success && response.data) {
        setCategories(response.data.categories);
        setError(null);
        setRetryCount(0);
      } else {
        throw new Error(response.error || 'Failed to fetch categories');
      }
    } catch (err) {
      console.error('Error refreshing categories:', err);
      const errorMessage = err instanceof Error ? err.message : 'Failed to refresh categories';
      setError(errorMessage);
      setRetryCount((prev) => prev + 1);
    }
  }, [retryApiCall]);

  const refreshValues = useCallback(async () => {
    try {
      const response = await retryApiCall(() => ApiService.getFilterValues());
      if (response.success && response.data) {
        setFilterValues(response.data.values);
        setError(null);
        setRetryCount(0);
      } else {
        throw new Error(response.error || 'Failed to fetch filter values');
      }
    } catch (err) {
      console.error('Error refreshing filter values:', err);
      const errorMessage = err instanceof Error ? err.message : 'Failed to refresh filter values';
      setError(errorMessage);
      setRetryCount((prev) => prev + 1);
    }
  }, [retryApiCall]);

  const refreshStats = useCallback(async () => {
    try {
      const response = await retryApiCall(() => ApiService.getFilterStats(isTemplate));
      if (response.success && response.data) {
        setStats(response.data.stats);
        setError(null);
        setRetryCount(0);
      } else {
        throw new Error(response.error || 'Failed to fetch filter stats');
      }
    } catch (err) {
      console.error('Error refreshing filter stats:', err);
      const errorMessage = err instanceof Error ? err.message : 'Failed to refresh filter stats';
      setError(errorMessage);
      setRetryCount((prev) => prev + 1);
    }
  }, [retryApiCall, isTemplate]);

  const refresh = useCallback(async () => {
    setLoading(true);
    setError(null);

    try {
      await Promise.all([refreshCategories(), refreshValues(), refreshStats()]);
    } catch (err) {
      console.error('Error refreshing filter data:', err);
      const errorMessage = err instanceof Error ? err.message : 'Failed to refresh filter data';
      setError(errorMessage);
    } finally {
      setLoading(false);
    }
  }, [refreshCategories, refreshValues, refreshStats]);

  const clearError = useCallback(() => {
    setError(null);
    setRetryCount(0);
  }, []);

  // Initial load
  useEffect(() => {
    refresh();
  }, [refresh]);

  // Auto-refresh on interval
  useEffect(() => {
    if (!autoRefresh) return;

    const interval = setInterval(() => {
      refresh();
    }, refreshInterval);

    return () => clearInterval(interval);
  }, [autoRefresh, refreshInterval, refresh]);

  // Listen for filter data change events
  useEffect(() => {
    const unsubscribe = filterEventManager.subscribe((event) => {
      if (event.type === 'categories' || event.type === 'both') {
        refreshCategories();
      }
      if (event.type === 'values' || event.type === 'both') {
        refreshValues();
      }
      if (event.type === 'both') {
        refreshStats();
      }
    });

    return unsubscribe;
  }, [refreshCategories, refreshValues, refreshStats]);

  // Refresh on page visibility change
  useEffect(() => {
    const handleVisibilityChange = () => {
      if (!document.hidden) {
        refresh();
      }
    };

    document.addEventListener('visibilitychange', handleVisibilityChange);
    return () => document.removeEventListener('visibilitychange', handleVisibilityChange);
  }, [refresh]);

  return {
    categories,
    filterValues,
    stats,
    loading,
    error,
    retryCount,
    refresh,
    refreshCategories,
    refreshValues,
    refreshStats,
    clearError,
  };
};
