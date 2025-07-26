import React from 'react';
import {
  Box,
  Typography,
  Card,
  CardContent,
  Avatar,
  Button,
  TextField,
  Divider,
} from '@mui/material';
import {
  Edit as EditIcon,
  Save as SaveIcon,
  Person as PersonIcon,
} from '@mui/icons-material';

const Profile: React.FC = () => {
  const [isEditing, setIsEditing] = React.useState(false);
  const [profile, setProfile] = React.useState({
    firstName: 'John',
    lastName: 'Doe',
    email: 'john.doe@example.com',
    role: 'User',
    company: 'Robium Corp',
  });

  const handleInputChange =
    (field: string) => (event: React.ChangeEvent<HTMLInputElement>) => {
      setProfile({
        ...profile,
        [field]: event.target.value,
      });
    };

  return (
    <Box>
      <Typography variant="h4" component="h1" gutterBottom>
        Profile
      </Typography>

      <Card sx={{ mb: 3 }}>
        <CardContent>
          <Box sx={{ display: 'flex', alignItems: 'center', mb: 3 }}>
            <Avatar
              sx={{ mr: 3, bgcolor: 'primary.main', width: 80, height: 80 }}
            >
              <PersonIcon sx={{ fontSize: 40 }} />
            </Avatar>
            <Box sx={{ flexGrow: 1 }}>
              <Typography variant="h5" gutterBottom>
                {profile.firstName} {profile.lastName}
              </Typography>
              <Typography variant="body1" color="text.secondary">
                {profile.role} • {profile.company}
              </Typography>
            </Box>
            <Button
              variant="outlined"
              startIcon={isEditing ? <SaveIcon /> : <EditIcon />}
              onClick={() => setIsEditing(!isEditing)}
            >
              {isEditing ? 'Save' : 'Edit'}
            </Button>
          </Box>

          <Divider sx={{ my: 2 }} />

          <Box sx={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 2 }}>
            <TextField
              label="First Name"
              value={profile.firstName}
              onChange={handleInputChange('firstName')}
              disabled={!isEditing}
              fullWidth
            />
            <TextField
              label="Last Name"
              value={profile.lastName}
              onChange={handleInputChange('lastName')}
              disabled={!isEditing}
              fullWidth
            />
            <TextField
              label="Email"
              value={profile.email}
              onChange={handleInputChange('email')}
              disabled={!isEditing}
              fullWidth
            />
            <TextField
              label="Company"
              value={profile.company}
              onChange={handleInputChange('company')}
              disabled={!isEditing}
              fullWidth
            />
          </Box>
        </CardContent>
      </Card>
    </Box>
  );
};

export default Profile;
