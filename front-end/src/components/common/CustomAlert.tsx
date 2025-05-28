import React from 'react';
import type { ReactNode } from 'react';
import type { AlertProps } from '@mui/material';
import { Alert as MuiAlert, Snackbar } from '@mui/material';

export interface CustomAlertProps extends Omit<AlertProps, 'onClose'> {
  open?: boolean;
  message?: string;
  autoHideDuration?: number;
  onClose?: () => void;
  position?: {
    vertical: 'top' | 'bottom';
    horizontal: 'left' | 'center' | 'right';
  };
  children?: ReactNode;
}

export const CustomAlert: React.FC<CustomAlertProps> = ({
  open = false,
  message,
  autoHideDuration = 6000,
  onClose,
  position = { vertical: 'top', horizontal: 'right' },
  children,
  ...alertProps
}) => {
  const handleClose = (event?: React.SyntheticEvent | Event, reason?: string) => {
    if (reason === 'clickaway') {
      return;
    }
    onClose?.();
  };

  if (open) {
    return (
      <Snackbar
        open={open}
        autoHideDuration={autoHideDuration}
        onClose={handleClose}
        anchorOrigin={position}
      >
        <MuiAlert
          elevation={6}
          variant="filled"
          onClose={handleClose}
          {...alertProps}
        >
          {message || children}
        </MuiAlert>
      </Snackbar>
    );
  }

  return (
    <MuiAlert
      elevation={6}
      variant="filled"
      {...alertProps}
    >
      {message || children}
    </MuiAlert>
  );
}; 