import { createTheme } from '@mui/material/styles';

// 自定义主题配置
export const theme = createTheme({
  palette: {
    mode: 'dark',
    primary: {
      main: '#00bcd4', // 青色 - 主色调
      light: '#4dd0e1',
      dark: '#00838f',
      contrastText: '#ffffff',
    },
    secondary: {
      main: '#ff9800', // 橙色 - 辅助色
      light: '#ffb74d',
      dark: '#e65100',
      contrastText: '#ffffff',
    },
    error: {
      main: '#f44336',
      light: '#e57373',
      dark: '#d32f2f',
    },
    warning: {
      main: '#ff9800',
      light: '#ffb74d',
      dark: '#e65100',
    },
    info: {
      main: '#2196f3',
      light: '#64b5f6',
      dark: '#1976d2',
    },
    success: {
      main: '#4caf50',
      light: '#81c784',
      dark: '#388e3c',
    },
    background: {
      default: '#121212',
      paper: '#1e1e1e',
    },
    text: {
      primary: '#ffffff',
      secondary: '#b0b0b0',
    },
  },
  typography: {
    fontFamily: '"Roboto", "Helvetica", "Arial", sans-serif',
    h4: {
      fontWeight: 600,
      fontSize: '1.5rem',
    },
    h5: {
      fontWeight: 500,
      fontSize: '1.25rem',
    },
    h6: {
      fontWeight: 500,
      fontSize: '1.1rem',
    },
    body1: {
      fontSize: '0.95rem',
    },
    body2: {
      fontSize: '0.85rem',
    },
  },
  components: {
    MuiCard: {
      styleOverrides: {
        root: {
          backgroundImage: 'none',
          backgroundColor: 'rgba(30, 30, 30, 0.9)',
          backdropFilter: 'blur(10px)',
          border: '1px solid rgba(255, 255, 255, 0.1)',
        },
      },
    },
    MuiButton: {
      styleOverrides: {
        root: {
          textTransform: 'none',
          borderRadius: 8,
          padding: '8px 16px',
        },
      },
    },
    MuiChip: {
      styleOverrides: {
        root: {
          borderRadius: 16,
        },
      },
    },
  },
});

// 连接状态颜色映射
export const connectionStatusColors = {
  connecting: '#ff9800', // 橙色
  connected: '#4caf50',  // 绿色
  disconnected: '#757575', // 灰色
  error: '#f44336',      // 红色
};

// 车辆状态颜色映射
export const vehicleStatusColors = {
  idle: '#2196f3',       // 蓝色
  moving: '#4caf50',     // 绿色
  parking: '#ff9800',    // 橙色
  emergency: '#f44336',  // 红色
};
