import React from 'react';
import {
  Box,
  Typography,
  Card,
  CardContent,
  Switch,
  FormControlLabel,
  Divider,
  Button,
} from '@mui/material';
import { Save as SaveIcon } from '@mui/icons-material';

const Settings: React.FC = () => {
  const [settings, setSettings] = React.useState({
    notifications: true,
    highContrast: false,
    autoSave: true,
    darkMode: false,
  });

  const handleSettingChange =
    (setting: string) => (event: React.ChangeEvent<HTMLInputElement>) => {
      setSettings({
        ...settings,
        [setting]: event.target.checked,
      });
    };

  return (
    <Box>
      <Typography variant="h4" component="h1" gutterBottom>
        Settings
      </Typography>

      <Card sx={{ mb: 3 }}>
        <CardContent>
          <Typography variant="h6" gutterBottom>
            Notifications
          </Typography>
          <FormControlLabel
            control={
              <Switch
                checked={settings.notifications}
                onChange={handleSettingChange('notifications')}
              />
            }
            label="Enable notifications"
          />
        </CardContent>
      </Card>

      <Card sx={{ mb: 3 }}>
        <CardContent>
          <Typography variant="h6" gutterBottom>
            Accessibility
          </Typography>
          <FormControlLabel
            control={
              <Switch
                checked={settings.highContrast}
                onChange={handleSettingChange('highContrast')}
              />
            }
            label="High contrast mode"
          />
          <Divider sx={{ my: 2 }} />
          <FormControlLabel
            control={
              <Switch
                checked={settings.darkMode}
                onChange={handleSettingChange('darkMode')}
              />
            }
            label="Dark mode"
          />
        </CardContent>
      </Card>

      <Card sx={{ mb: 3 }}>
        <CardContent>
          <Typography variant="h6" gutterBottom>
            Application
          </Typography>
          <FormControlLabel
            control={
              <Switch
                checked={settings.autoSave}
                onChange={handleSettingChange('autoSave')}
              />
            }
            label="Auto-save changes"
          />
        </CardContent>
      </Card>

      <Box sx={{ display: 'flex', justifyContent: 'flex-end' }}>
        <Button variant="contained" startIcon={<SaveIcon />}>
          Save Settings
        </Button>
      </Box>
    </Box>
  );
};

export default Settings;
