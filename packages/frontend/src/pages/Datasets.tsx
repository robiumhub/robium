import React, { useState, useEffect } from 'react';
import {
  Box,
  Typography,
  Grid,
  Card,
  CardContent,
  CardActions,
  Button,
  Chip,
  Avatar,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  TextField,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Alert,
  CircularProgress,
  Tooltip,
  IconButton,
  Badge,
  LinearProgress,
} from '@mui/material';
import {
  Download as DownloadIcon,
  Visibility as ViewIcon,
  Storage as StorageIcon,
  Image as ImageIcon,
  VideoLibrary as VideoIcon,
  TextFields as TextIcon,
  Sensors as SensorIcon,
  CloudDownload as CloudIcon,
  Info as InfoIcon,
} from '@mui/icons-material';
import { useNavigate } from 'react-router-dom';
import { ApiService } from '../services/api';

interface Dataset {
  id: string;
  name: string;
  description: string;
  category: string;
  modalities: string[];
  size: string;
  samples: number;
  format: string;
  license: string;
  tags: string[];
  downloadUrl?: string;
  previewUrl?: string;
  documentation?: string;
  usage: string;
  requirements: string[];
}

const Datasets: React.FC = () => {
  const [datasets, setDatasets] = useState<Dataset[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [selectedDataset, setSelectedDataset] = useState<Dataset | null>(null);
  const [downloadDialogOpen, setDownloadDialogOpen] = useState(false);
  const [downloading, setDownloading] = useState(false);
  const [filterCategory, setFilterCategory] = useState<string>('all');
  const navigate = useNavigate();

  // Mock datasets data - in real app, this would come from API
  const mockDatasets: Dataset[] = [
    {
      id: '1',
      name: 'RoboVision Dataset',
      description:
        'Large-scale dataset for robot vision tasks including object detection, segmentation, and pose estimation.',
      category: 'Computer Vision',
      modalities: ['RGB Images', 'Depth Maps', 'Point Clouds'],
      size: '2.5 GB',
      samples: 50000,
      format: 'COCO, YOLO, PASCAL VOC',
      license: 'MIT',
      tags: ['Object Detection', 'Segmentation', 'Pose Estimation', 'Robotics'],
      usage: 'Training computer vision models for robotic applications',
      requirements: ['Python 3.8+', 'OpenCV', 'PyTorch/TensorFlow'],
    },
    {
      id: '2',
      name: 'Navigation Sensor Data',
      description:
        'Multi-sensor dataset for autonomous navigation including lidar, IMU, and GPS data.',
      category: 'Navigation',
      modalities: [
        'Lidar Point Clouds',
        'IMU Data',
        'GPS Coordinates',
        'Odometry',
      ],
      size: '1.8 GB',
      samples: 25000,
      format: 'ROS Bags, CSV, JSON',
      license: 'Apache 2.0',
      tags: ['SLAM', 'Localization', 'Path Planning', 'Sensor Fusion'],
      usage: 'Developing and testing navigation algorithms',
      requirements: ['ROS2', 'PCL', 'NumPy', 'Pandas'],
    },
    {
      id: '3',
      name: 'Manipulation Trajectories',
      description:
        'Robotic arm manipulation trajectories for pick-and-place, assembly, and manipulation tasks.',
      category: 'Manipulation',
      modalities: [
        'Joint Trajectories',
        'End-Effector Poses',
        'Force/Torque Data',
      ],
      size: '850 MB',
      samples: 15000,
      format: 'MoveIt Trajectories, CSV, JSON',
      license: 'BSD-3-Clause',
      tags: ['Trajectory Planning', 'Force Control', 'Grasping', 'Assembly'],
      usage: 'Training manipulation models and trajectory optimization',
      requirements: ['MoveIt', 'ROS2', 'NumPy', 'SciPy'],
    },
    {
      id: '4',
      name: 'Human-Robot Interaction',
      description:
        'Dataset of human-robot interaction scenarios including speech, gestures, and collaborative tasks.',
      category: 'HRI',
      modalities: ['Audio', 'Video', 'Gesture Data', 'Interaction Logs'],
      size: '3.2 GB',
      samples: 8000,
      format: 'WAV, MP4, JSON',
      license: 'Creative Commons',
      tags: [
        'Speech Recognition',
        'Gesture Recognition',
        'Collaboration',
        'Safety',
      ],
      usage: 'Developing human-robot interaction systems',
      requirements: ['Speech Recognition API', 'OpenCV', 'PyAudio'],
    },
    {
      id: '5',
      name: 'Simulation Environments',
      description:
        'Collection of simulation environments and scenarios for testing robotic algorithms.',
      category: 'Simulation',
      modalities: ['3D Environments', 'Physics Data', 'Sensor Simulations'],
      size: '5.1 GB',
      samples: 100,
      format: 'Gazebo Worlds, Unity Assets, Blender Files',
      license: 'GPL v3',
      tags: ['Simulation', 'Testing', 'Validation', 'Benchmarking'],
      usage:
        'Testing algorithms in simulated environments before real-world deployment',
      requirements: ['Gazebo', 'ROS2', 'Unity/Blender'],
    },
    {
      id: '6',
      name: 'Multi-Modal Robot Data',
      description:
        'Comprehensive dataset combining vision, audio, and sensor data for multi-modal robot learning.',
      category: 'Multi-Modal',
      modalities: [
        'RGB-D Images',
        'Audio',
        'IMU',
        'Force Sensors',
        'Temperature',
      ],
      size: '8.7 GB',
      samples: 12000,
      format: 'ROS Bags, HDF5, JSON',
      license: 'MIT',
      tags: ['Multi-Modal Learning', 'Sensor Fusion', 'AI', 'Machine Learning'],
      usage: 'Training multi-modal AI models for robotics',
      requirements: ['PyTorch', 'TensorFlow', 'ROS2', 'NumPy', 'Pandas'],
    },
  ];

  useEffect(() => {
    // Simulate API call
    const loadDatasets = async () => {
      try {
        setLoading(true);
        // In real app: const response = await ApiService.get('/datasets');
        // Simulate API delay
        await new Promise((resolve) => setTimeout(resolve, 1000));
        setDatasets(mockDatasets);
      } catch (err) {
        setError('Failed to load datasets');
        console.error('Error loading datasets:', err);
      } finally {
        setLoading(false);
      }
    };

    loadDatasets();
  }, []);

  const handleDownloadDataset = (dataset: Dataset) => {
    setSelectedDataset(dataset);
    setDownloadDialogOpen(true);
  };

  const handleDownloadConfirm = async () => {
    if (!selectedDataset) return;

    try {
      setDownloading(true);
      // In real app: await ApiService.post('/datasets/download', { datasetId: selectedDataset.id });

      // Simulate download
      await new Promise((resolve) => setTimeout(resolve, 3000));

      setDownloadDialogOpen(false);
      setSelectedDataset(null);

      // Show success message or navigate to downloads
    } catch (err) {
      setError('Failed to download dataset');
      console.error('Error downloading dataset:', err);
    } finally {
      setDownloading(false);
    }
  };

  const getCategoryIcon = (category: string) => {
    switch (category.toLowerCase()) {
      case 'computer vision':
        return <ImageIcon />;
      case 'navigation':
        return <SensorIcon />;
      case 'manipulation':
        return <StorageIcon />;
      case 'hri':
        return <TextIcon />;
      case 'simulation':
        return <VideoIcon />;
      case 'multi-modal':
        return <CloudIcon />;
      default:
        return <StorageIcon />;
    }
  };

  const getModalityIcon = (modality: string) => {
    if (
      modality.toLowerCase().includes('image') ||
      modality.toLowerCase().includes('rgb')
    ) {
      return <ImageIcon />;
    } else if (modality.toLowerCase().includes('video')) {
      return <VideoIcon />;
    } else if (
      modality.toLowerCase().includes('audio') ||
      modality.toLowerCase().includes('speech')
    ) {
      return <TextIcon />;
    } else if (
      modality.toLowerCase().includes('sensor') ||
      modality.toLowerCase().includes('lidar')
    ) {
      return <SensorIcon />;
    } else {
      return <StorageIcon />;
    }
  };

  const filteredDatasets =
    filterCategory === 'all'
      ? datasets
      : datasets.filter(
          (dataset) =>
            dataset.category.toLowerCase() === filterCategory.toLowerCase()
        );

  const categories = [
    'all',
    ...Array.from(new Set(datasets.map((d) => d.category))),
  ];

  if (loading) {
    return (
      <Box
        display="flex"
        justifyContent="center"
        alignItems="center"
        minHeight="400px"
      >
        <CircularProgress />
      </Box>
    );
  }

  if (error) {
    return (
      <Box
        display="flex"
        justifyContent="center"
        alignItems="center"
        minHeight="400px"
      >
        <Alert severity="error">{error}</Alert>
      </Box>
    );
  }

  return (
    <Box>
      <Typography variant="h4" gutterBottom>
        Datasets
      </Typography>
      <Typography variant="body1" color="text.secondary" sx={{ mb: 3 }}>
        Browse and download datasets for your robotics projects. Each dataset
        includes detailed information about modalities, usage, and requirements.
      </Typography>

      {/* Filter */}
      <Box sx={{ mb: 3 }}>
        <FormControl sx={{ minWidth: 200 }}>
          <InputLabel>Filter by Category</InputLabel>
          <Select
            value={filterCategory}
            label="Filter by Category"
            onChange={(e) => setFilterCategory(e.target.value)}
          >
            {categories.map((category) => (
              <MenuItem key={category} value={category}>
                {category === 'all' ? 'All Categories' : category}
              </MenuItem>
            ))}
          </Select>
        </FormControl>
      </Box>

      <Grid container spacing={3}>
        {filteredDatasets.map((dataset) => (
          <Grid item xs={12} sm={6} md={4} key={dataset.id}>
            <Card
              sx={{
                height: '100%',
                display: 'flex',
                flexDirection: 'column',
              }}
            >
              <CardContent sx={{ flexGrow: 1 }}>
                <Box display="flex" alignItems="center" mb={2}>
                  <Avatar sx={{ mr: 1, bgcolor: 'primary.main' }}>
                    {getCategoryIcon(dataset.category)}
                  </Avatar>
                  <Box>
                    <Typography variant="h6" component="h2">
                      {dataset.name}
                    </Typography>
                    <Chip
                      label={dataset.category}
                      size="small"
                      variant="outlined"
                    />
                  </Box>
                </Box>

                <Typography
                  variant="body2"
                  color="text.secondary"
                  sx={{ mb: 2 }}
                >
                  {dataset.description}
                </Typography>

                <Box sx={{ mb: 2 }}>
                  <Typography variant="body2" sx={{ mb: 1 }}>
                    <strong>Modalities:</strong>
                  </Typography>
                  <Box display="flex" flexWrap="wrap" gap={0.5}>
                    {dataset.modalities.map((modality) => (
                      <Chip
                        key={modality}
                        icon={getModalityIcon(modality)}
                        label={modality}
                        size="small"
                        variant="outlined"
                      />
                    ))}
                  </Box>
                </Box>

                <Box sx={{ mb: 2 }}>
                  <Typography variant="body2" sx={{ mb: 1 }}>
                    <strong>Size:</strong> {dataset.size} •{' '}
                    <strong>Samples:</strong> {dataset.samples.toLocaleString()}
                  </Typography>
                  <Typography variant="body2" color="text.secondary">
                    <strong>Format:</strong> {dataset.format}
                  </Typography>
                </Box>

                <Box sx={{ mb: 2 }}>
                  {dataset.tags.map((tag) => (
                    <Chip
                      key={tag}
                      label={tag}
                      size="small"
                      variant="outlined"
                      sx={{ mr: 0.5, mb: 0.5 }}
                    />
                  ))}
                </Box>

                <Typography variant="body2" color="text.secondary">
                  <strong>License:</strong> {dataset.license}
                </Typography>
              </CardContent>

              <CardActions sx={{ justifyContent: 'space-between', p: 2 }}>
                <Tooltip title="View dataset details">
                  <IconButton size="small">
                    <ViewIcon />
                  </IconButton>
                </Tooltip>
                <Button
                  variant="contained"
                  startIcon={<DownloadIcon />}
                  onClick={() => handleDownloadDataset(dataset)}
                  size="small"
                >
                  Download
                </Button>
              </CardActions>
            </Card>
          </Grid>
        ))}
      </Grid>

      {/* Download Dialog */}
      <Dialog
        open={downloadDialogOpen}
        onClose={() => setDownloadDialogOpen(false)}
        maxWidth="sm"
        fullWidth
      >
        <DialogTitle>Download Dataset</DialogTitle>
        <DialogContent>
          {selectedDataset && (
            <Box>
              <Typography variant="body1" sx={{ mb: 2 }}>
                You're about to download <strong>{selectedDataset.name}</strong>
                .
              </Typography>

              <Box sx={{ mb: 2 }}>
                <Typography variant="body2" sx={{ mb: 1 }}>
                  <strong>Dataset Information:</strong>
                </Typography>
                <Typography variant="body2" color="text.secondary">
                  Size: {selectedDataset.size} • Samples:{' '}
                  {selectedDataset.samples.toLocaleString()}
                </Typography>
                <Typography variant="body2" color="text.secondary">
                  Format: {selectedDataset.format}
                </Typography>
                <Typography variant="body2" color="text.secondary">
                  License: {selectedDataset.license}
                </Typography>
              </Box>

              <Box sx={{ mb: 2 }}>
                <Typography variant="body2" sx={{ mb: 1 }}>
                  <strong>Usage:</strong>
                </Typography>
                <Typography variant="body2" color="text.secondary">
                  {selectedDataset.usage}
                </Typography>
              </Box>

              <Box sx={{ mb: 2 }}>
                <Typography variant="body2" sx={{ mb: 1 }}>
                  <strong>Requirements:</strong>
                </Typography>
                <Box component="ul" sx={{ pl: 2 }}>
                  {selectedDataset.requirements.map((req, index) => (
                    <Typography
                      key={index}
                      variant="body2"
                      component="li"
                      color="text.secondary"
                    >
                      {req}
                    </Typography>
                  ))}
                </Box>
              </Box>

              {downloading && (
                <Box sx={{ mt: 2 }}>
                  <Typography variant="body2" sx={{ mb: 1 }}>
                    Downloading dataset...
                  </Typography>
                  <LinearProgress />
                </Box>
              )}
            </Box>
          )}
        </DialogContent>
        <DialogActions>
          <Button
            onClick={() => setDownloadDialogOpen(false)}
            disabled={downloading}
          >
            Cancel
          </Button>
          <Button
            onClick={handleDownloadConfirm}
            variant="contained"
            disabled={downloading}
            startIcon={
              downloading ? <CircularProgress size={16} /> : <DownloadIcon />
            }
          >
            {downloading ? 'Downloading...' : 'Download Dataset'}
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default Datasets;
