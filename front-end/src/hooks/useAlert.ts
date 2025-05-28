import { useState, useCallback } from 'react';

interface AlertState {
  open: boolean;
  message: string;
  severity: 'success' | 'info' | 'warning' | 'error';
  autoHideDuration?: number;
}

interface UseAlertReturn {
  alertState: AlertState;
  showAlert: (message: string, severity?: AlertState['severity'], duration?: number) => void;
  hideAlert: () => void;
}

export const useAlert = (defaultDuration = 6000): UseAlertReturn => {
  const [alertState, setAlertState] = useState<AlertState>({
    open: false,
    message: '',
    severity: 'info',
    autoHideDuration: defaultDuration,
  });

  const showAlert = useCallback((
    message: string,
    severity: AlertState['severity'] = 'info',
    duration?: number
  ) => {
    setAlertState({
      open: true,
      message,
      severity,
      autoHideDuration: duration || defaultDuration,
    });
  }, [defaultDuration]);

  const hideAlert = useCallback(() => {
    setAlertState(prev => ({ ...prev, open: false }));
  }, []);

  return {
    alertState,
    showAlert,
    hideAlert,
  };
}; 