import React, { useState, useEffect, useRef } from 'react';
import {
  Box,
  Typography,
  Card,
  CardContent,
  Grid,
  Tabs,
  Tab,
  Button,
  TextField,
  Chip,
  Avatar,
  List,
  ListItem,
  ListItemText,
  ListItemAvatar,
  IconButton,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  Alert,
  CircularProgress,
  Tooltip,
  Paper,
  Divider,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Switch,
  FormControlLabel,
  InputAdornment,
  Badge,
} from '@mui/material';
import {
  DragIndicator as DragIcon,
  Add as AddIcon,
  Edit as EditIcon,
  Delete as DeleteIcon,
  Folder as FolderIcon,
  Assignment as TaskIcon,
  Workflow as WorkflowIcon,
  ViewKanban as KanbanIcon,
  Timeline as TimelineIcon,
  Settings as SettingsIcon,
  Save as SaveIcon,
  Undo as UndoIcon,
  Redo as RedoIcon,
  ExpandMore as ExpandMoreIcon,
  CheckCircle as CheckCircleIcon,
  Warning as WarningIcon,
  Error as ErrorIcon,
  Info as InfoIcon,
  CloudUpload as CloudUploadIcon,
  CloudDownload as CloudDownloadIcon,
  Archive as ArchiveIcon,
  RestoreFromTrash as RestoreIcon,
  GetApp as GetAppIcon,
  Send as SendIcon,
  Block as BlockIcon,
  Unblock as UnblockIcon,
  Timeline as TimelineIcon,
  Code as CodeIcon,
  BugReport as BugReportIcon,
  NewReleases as NewReleasesIcon,
  Security as SecurityIcon,
  DragHandle as DragHandleIcon,
  ContentCopy as CopyIcon,
  ContentCut as CutIcon,
  ContentPaste as PasteIcon,
  ZoomIn as ZoomInIcon,
  ZoomOut as ZoomOutIcon,
  Fullscreen as FullscreenIcon,
  FullscreenExit as FullscreenExitIcon,
} from '@mui/icons-material';

// Types
interface DraggableItem {
  id: string;
  type: 'project' | 'task' | 'module' | 'workflow';
  title: string;
  description: string;
  status: 'active' | 'inactive' | 'completed' | 'archived';
  priority: 'low' | 'medium' | 'high' | 'critical';
  assignee?: string;
  dueDate?: string;
  tags: string[];
  position: { x: number; y: number };
  size: { width: number; height: number };
  color: string;
  parentId?: string;
  children: string[];
}

interface DropZone {
  id: string;
  name: string;
  type: 'folder' | 'board' | 'timeline' | 'workflow';
  items: string[];
  position: { x: number; y: number };
  size: { width: number; height: number };
  color: string;
  acceptTypes: string[];
  maxItems?: number;
}

interface WorkflowStep {
  id: string;
  name: string;
  type: 'start' | 'process' | 'decision' | 'end';
  position: { x: number; y: number };
  connections: string[];
  data: any;
}

interface TabPanelProps {
  children?: React.ReactNode;
  index: number;
  value: number;
}

function TabPanel(props: TabPanelProps) {
  const { children, value, index, ...other } = props;

  return (
    <div
      role="tabpanel"
      hidden={value !== index}
      id={`dragdrop-tabpanel-${index}`}
      aria-labelledby={`dragdrop-tab-${index}`}
      {...other}
    >
      {value === index && <Box sx={{ p: 3 }}>{children}</Box>}
    </div>
  );
}

const DragDropOrganization: React.FC = () => {
  const [tabValue, setTabValue] = useState(0);
  const [loading, setLoading] = useState(false);
  const [items, setItems] = useState<DraggableItem[]>([]);
  const [dropZones, setDropZones] = useState<DropZone[]>([]);
  const [workflowSteps, setWorkflowSteps] = useState<WorkflowStep[]>([]);
  const [selectedItem, setSelectedItem] = useState<DraggableItem | null>(null);
  const [draggedItem, setDraggedItem] = useState<DraggableItem | null>(null);
  const [showItemDialog, setShowItemDialog] = useState(false);
  const [showZoneDialog, setShowZoneDialog] = useState(false);
  const [showWorkflowDialog, setShowWorkflowDialog] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);
  const [zoom, setZoom] = useState(1);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const canvasRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    loadDragDropData();
  }, []);

  const loadDragDropData = async () => {
    setLoading(true);
    try {
      // Mock data
      const mockItems: DraggableItem[] = [
        {
          id: '1',
          type: 'project',
          title: 'Autonomous Navigation',
          description: 'Self-driving robot navigation system',
          status: 'active',
          priority: 'high',
          assignee: 'alice@robium.com',
          dueDate: '2024-02-15',
          tags: ['navigation', 'autonomous', 'robotics'],
          position: { x: 100, y: 100 },
          size: { width: 200, height: 150 },
          color: '#2196F3',
          children: ['2', '3'],
        },
        {
          id: '2',
          type: 'task',
          title: 'Path Planning Algorithm',
          description: 'Implement A* pathfinding algorithm',
          status: 'active',
          priority: 'medium',
          assignee: 'bob@robium.com',
          dueDate: '2024-01-30',
          tags: ['algorithm', 'pathfinding'],
          position: { x: 350, y: 100 },
          size: { width: 180, height: 120 },
          color: '#4CAF50',
          parentId: '1',
          children: [],
        },
        {
          id: '3',
          type: 'module',
          title: 'Sensor Integration',
          description: 'Integrate LIDAR and camera sensors',
          status: 'inactive',
          priority: 'high',
          tags: ['sensors', 'hardware'],
          position: { x: 100, y: 300 },
          size: { width: 180, height: 120 },
          color: '#FF9800',
          parentId: '1',
          children: [],
        },
      ];

      const mockDropZones: DropZone[] = [
        {
          id: 'zone1',
          name: 'Active Projects',
          type: 'folder',
          items: ['1'],
          position: { x: 50, y: 50 },
          size: { width: 300, height: 200 },
          color: '#E3F2FD',
          acceptTypes: ['project', 'task'],
          maxItems: 10,
        },
        {
          id: 'zone2',
          name: 'Completed Tasks',
          type: 'board',
          items: [],
          position: { x: 400, y: 50 },
          size: { width: 300, height: 200 },
          color: '#E8F5E8',
          acceptTypes: ['task'],
          maxItems: 20,
        },
        {
          id: 'zone3',
          name: 'Archived Items',
          type: 'folder',
          items: [],
          position: { x: 750, y: 50 },
          size: { width: 300, height: 200 },
          color: '#F5F5F5',
          acceptTypes: ['project', 'task', 'module'],
          maxItems: 50,
        },
      ];

      const mockWorkflowSteps: WorkflowStep[] = [
        {
          id: 'step1',
          name: 'Start',
          type: 'start',
          position: { x: 100, y: 100 },
          connections: ['step2'],
          data: {},
        },
        {
          id: 'step2',
          name: 'Data Collection',
          type: 'process',
          position: { x: 300, y: 100 },
          connections: ['step3'],
          data: { duration: '2 hours' },
        },
        {
          id: 'step3',
          name: 'Processing',
          type: 'process',
          position: { x: 500, y: 100 },
          connections: ['step4'],
          data: { duration: '1 hour' },
        },
        {
          id: 'step4',
          name: 'Decision',
          type: 'decision',
          position: { x: 700, y: 100 },
          connections: ['step5', 'step6'],
          data: { condition: 'Quality Check' },
        },
        {
          id: 'step5',
          name: 'Success',
          type: 'end',
          position: { x: 700, y: 250 },
          connections: [],
          data: {},
        },
        {
          id: 'step6',
          name: 'Retry',
          type: 'process',
          position: { x: 700, y: 350 },
          connections: ['step2'],
          data: { maxRetries: 3 },
        },
      ];

      setItems(mockItems);
      setDropZones(mockDropZones);
      setWorkflowSteps(mockWorkflowSteps);
      setLoading(false);
    } catch (err) {
      setError('Failed to load drag and drop data');
      setLoading(false);
    }
  };

  const handleDragStart = (item: DraggableItem) => {
    setDraggedItem(item);
  };

  const handleDragEnd = () => {
    setDraggedItem(null);
  };

  const handleDrop = (zoneId: string, itemId: string) => {
    try {
      // Update item position and parent
      setItems(items.map(item => 
        item.id === itemId 
          ? { ...item, parentId: zoneId, position: { x: 0, y: 0 } }
          : item
      ));

      // Update drop zone items
      setDropZones(dropZones.map(zone => 
        zone.id === zoneId 
          ? { ...zone, items: [...zone.items, itemId] }
          : zone
      ));

      setSuccess('Item moved successfully');
    } catch (err) {
      setError('Failed to move item');
    }
  };

  const handleCreateItem = async () => {
    try {
      // TODO: Implement API call
      console.log('Creating new item');
      setShowItemDialog(false);
      setSuccess('Item created successfully');
    } catch (err) {
      setError('Failed to create item');
    }
  };

  const handleCreateZone = async () => {
    try {
      // TODO: Implement API call
      console.log('Creating new drop zone');
      setShowZoneDialog(false);
      setSuccess('Drop zone created successfully');
    } catch (err) {
      setError('Failed to create drop zone');
    }
  };

  const handleZoomIn = () => {
    setZoom(Math.min(zoom * 1.2, 3));
  };

  const handleZoomOut = () => {
    setZoom(Math.max(zoom / 1.2, 0.3));
  };

  const handleResetZoom = () => {
    setZoom(1);
    setPan({ x: 0, y: 0 });
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'active':
        return 'success';
      case 'inactive':
        return 'default';
      case 'completed':
        return 'primary';
      case 'archived':
        return 'error';
      default:
        return 'default';
    }
  };

  const getPriorityColor = (priority: string) => {
    switch (priority) {
      case 'low':
        return 'success';
      case 'medium':
        return 'warning';
      case 'high':
        return 'error';
      case 'critical':
        return 'error';
      default:
        return 'default';
    }
  };

  const getTypeIcon = (type: string) => {
    switch (type) {
      case 'project':
        return <FolderIcon />;
      case 'task':
        return <TaskIcon />;
      case 'module':
        return <CodeIcon />;
      case 'workflow':
        return <WorkflowIcon />;
      default:
        return <InfoIcon />;
    }
  };

  const formatDate = (dateString: string) => {
    return new Date(dateString).toLocaleDateString();
  };

  if (loading) {
    return (
      <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '50vh' }}>
        <CircularProgress />
      </Box>
    );
  }

  return (
    <Box>
      {/* Header */}
      <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', mb: 3 }}>
        <Typography variant="h4" component="h1">
          Drag & Drop Organization
        </Typography>
        <Box sx={{ display: 'flex', gap: 2 }}>
          <Button
            variant="outlined"
            startIcon={<ZoomOutIcon />}
            onClick={handleZoomOut}
          >
            Zoom Out
          </Button>
          <Button
            variant="outlined"
            startIcon={<ZoomInIcon />}
            onClick={handleZoomIn}
          >
            Zoom In
          </Button>
          <Button
            variant="outlined"
            startIcon={<FullscreenIcon />}
            onClick={handleResetZoom}
          >
            Reset
          </Button>
          <Button
            variant="contained"
            startIcon={<AddIcon />}
            onClick={() => setShowItemDialog(true)}
          >
            Add Item
          </Button>
        </Box>
      </Box>

      {/* Alerts */}
      {error && (
        <Alert severity="error" sx={{ mb: 3 }} onClose={() => setError(null)}>
          {error}
        </Alert>
      )}
      {success && (
        <Alert severity="success" sx={{ mb: 3 }} onClose={() => setSuccess(null)}>
          {success}
        </Alert>
      )}

      {/* Main Tabs */}
      <Card>
        <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
          <Tabs value={tabValue} onChange={(_, newValue) => setTabValue(newValue)}>
            <Tab icon={<KanbanIcon />} label="Kanban Board" />
            <Tab icon={<TimelineIcon />} label="Timeline View" />
            <Tab icon={<WorkflowIcon />} label="Workflow Designer" />
            <Tab icon={<SettingsIcon />} label="Settings" />
          </Tabs>
        </Box>

        {/* Kanban Board Tab */}
        <TabPanel value={tabValue} index={0}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
            <Typography variant="h6">
              Kanban Board
            </Typography>
            <Button
              variant="contained"
              startIcon={<AddIcon />}
              onClick={() => setShowZoneDialog(true)}
            >
              Add Column
            </Button>
          </Box>

          <Box
            ref={canvasRef}
            sx={{
              position: 'relative',
              width: '100%',
              height: '600px',
              overflow: 'auto',
              backgroundColor: '#f5f5f5',
              transform: `scale(${zoom})`,
              transformOrigin: 'top left',
            }}
          >
            {/* Drop Zones */}
            {dropZones.map((zone) => (
              <Card
                key={zone.id}
                sx={{
                  position: 'absolute',
                  left: zone.position.x,
                  top: zone.position.y,
                  width: zone.size.width,
                  height: zone.size.height,
                  backgroundColor: zone.color,
                  border: '2px dashed #ccc',
                  '&:hover': {
                    border: '2px dashed #2196F3',
                  },
                }}
                onDrop={(e) => {
                  e.preventDefault();
                  const itemId = e.dataTransfer.getData('text/plain');
                  handleDrop(zone.id, itemId);
                }}
                onDragOver={(e) => e.preventDefault()}
              >
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    {zone.name}
                  </Typography>
                  <Typography variant="body2" color="textSecondary">
                    {zone.items.length} items
                  </Typography>
                  
                  {/* Items in this zone */}
                  <Box sx={{ mt: 2 }}>
                    {items
                      .filter(item => zone.items.includes(item.id))
                      .map((item) => (
                        <Card
                          key={item.id}
                          sx={{
                            mb: 1,
                            cursor: 'grab',
                            backgroundColor: item.color,
                            color: 'white',
                            '&:hover': {
                              cursor: 'grabbing',
                            },
                          }}
                          draggable
                          onDragStart={(e) => {
                            e.dataTransfer.setData('text/plain', item.id);
                            handleDragStart(item);
                          }}
                          onDragEnd={handleDragEnd}
                        >
                          <CardContent sx={{ p: 1 }}>
                            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                              <DragHandleIcon fontSize="small" />
                              <Typography variant="body2" noWrap>
                                {item.title}
                              </Typography>
                            </Box>
                            <Box sx={{ display: 'flex', gap: 0.5, mt: 1 }}>
                              <Chip
                                label={item.status}
                                size="small"
                                color={getStatusColor(item.status) as any}
                              />
                              <Chip
                                label={item.priority}
                                size="small"
                                color={getPriorityColor(item.priority) as any}
                              />
                            </Box>
                          </CardContent>
                        </Card>
                      ))}
                  </Box>
                </CardContent>
              </Card>
            ))}

            {/* Floating Items */}
            {items
              .filter(item => !item.parentId)
              .map((item) => (
                <Card
                  key={item.id}
                  sx={{
                    position: 'absolute',
                    left: item.position.x,
                    top: item.position.y,
                    width: item.size.width,
                    height: item.size.height,
                    cursor: 'grab',
                    backgroundColor: item.color,
                    color: 'white',
                    '&:hover': {
                      cursor: 'grabbing',
                    },
                  }}
                  draggable
                  onDragStart={(e) => {
                    e.dataTransfer.setData('text/plain', item.id);
                    handleDragStart(item);
                  }}
                  onDragEnd={handleDragEnd}
                >
                  <CardContent>
                    <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 1 }}>
                      {getTypeIcon(item.type)}
                      <Typography variant="h6" noWrap>
                        {item.title}
                      </Typography>
                    </Box>
                    <Typography variant="body2" sx={{ mb: 1 }}>
                      {item.description}
                    </Typography>
                    <Box sx={{ display: 'flex', gap: 0.5, mb: 1 }}>
                      <Chip
                        label={item.status}
                        size="small"
                        color={getStatusColor(item.status) as any}
                      />
                      <Chip
                        label={item.priority}
                        size="small"
                        color={getPriorityColor(item.priority) as any}
                      />
                    </Box>
                    {item.assignee && (
                      <Typography variant="caption">
                        Assigned to: {item.assignee}
                      </Typography>
                    )}
                    {item.dueDate && (
                      <Typography variant="caption" display="block">
                        Due: {formatDate(item.dueDate)}
                      </Typography>
                    )}
                  </CardContent>
                </Card>
              ))}
          </Box>
        </TabPanel>

        {/* Timeline View Tab */}
        <TabPanel value={tabValue} index={1}>
          <Typography variant="h6" gutterBottom>
            Timeline View
          </Typography>
          
          <Box sx={{ height: '400px', overflow: 'auto' }}>
            <Timeline>
              {items.map((item, index) => (
                <TimelineItem key={item.id}>
                  <TimelineSeparator>
                    <TimelineDot sx={{ backgroundColor: item.color }}>
                      {getTypeIcon(item.type)}
                    </TimelineDot>
                    {index < items.length - 1 && <TimelineConnector />}
                  </TimelineSeparator>
                  <TimelineContent>
                    <Card>
                      <CardContent>
                        <Typography variant="h6">{item.title}</Typography>
                        <Typography variant="body2" color="textSecondary">
                          {item.description}
                        </Typography>
                        <Box sx={{ display: 'flex', gap: 1, mt: 1 }}>
                          <Chip
                            label={item.status}
                            size="small"
                            color={getStatusColor(item.status) as any}
                          />
                          <Chip
                            label={item.priority}
                            size="small"
                            color={getPriorityColor(item.priority) as any}
                          />
                        </Box>
                      </CardContent>
                    </Card>
                  </TimelineContent>
                </TimelineItem>
              ))}
            </Timeline>
          </Box>
        </TabPanel>

        {/* Workflow Designer Tab */}
        <TabPanel value={tabValue} index={2}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
            <Typography variant="h6">
              Workflow Designer
            </Typography>
            <Button
              variant="contained"
              startIcon={<AddIcon />}
              onClick={() => setShowWorkflowDialog(true)}
            >
              Add Step
            </Button>
          </Box>

          <Box
            sx={{
              position: 'relative',
              width: '100%',
              height: '500px',
              overflow: 'auto',
              backgroundColor: '#f5f5f5',
              transform: `scale(${zoom})`,
              transformOrigin: 'top left',
            }}
          >
            {workflowSteps.map((step) => (
              <Card
                key={step.id}
                sx={{
                  position: 'absolute',
                  left: step.position.x,
                  top: step.position.y,
                  width: 150,
                  height: 80,
                  cursor: 'grab',
                  backgroundColor: step.type === 'start' ? '#4CAF50' : 
                                   step.type === 'end' ? '#F44336' : 
                                   step.type === 'decision' ? '#FF9800' : '#2196F3',
                  color: 'white',
                  '&:hover': {
                    cursor: 'grabbing',
                  },
                }}
                draggable
              >
                <CardContent sx={{ textAlign: 'center', p: 1 }}>
                  <Typography variant="body2" noWrap>
                    {step.name}
                  </Typography>
                  <Typography variant="caption">
                    {step.type}
                  </Typography>
                </CardContent>
              </Card>
            ))}
          </Box>
        </TabPanel>

        {/* Settings Tab */}
        <TabPanel value={tabValue} index={3}>
          <Typography variant="h6" gutterBottom>
            Organization Settings
          </Typography>
          
          <Grid container spacing={3}>
            <Grid item xs={12} md={6}>
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    Display Settings
                  </Typography>
                  
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Show item details on hover"
                  />
                  
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Enable snap to grid"
                  />
                  
                  <FormControlLabel
                    control={<Switch />}
                    label="Auto-save changes"
                  />
                  
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Show connection lines"
                  />
                </CardContent>
              </Card>
            </Grid>

            <Grid item xs={12} md={6}>
              <Card>
                <CardContent>
                  <Typography variant="h6" gutterBottom>
                    Interaction Settings
                  </Typography>
                  
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Enable drag and drop"
                  />
                  
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Enable resizing"
                  />
                  
                  <FormControlLabel
                    control={<Switch />}
                    label="Enable multi-select"
                  />
                  
                  <FormControlLabel
                    control={<Switch defaultChecked />}
                    label="Show drop zones"
                  />
                </CardContent>
              </Card>
            </Grid>
          </Grid>
        </TabPanel>
      </Card>

      {/* Create Item Dialog */}
      <Dialog open={showItemDialog} onClose={() => setShowItemDialog(false)} maxWidth="md" fullWidth>
        <DialogTitle>Create New Item</DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 2 }}>
            <Grid container spacing={2}>
              <Grid item xs={12} md={6}>
                <TextField
                  fullWidth
                  label="Title"
                  margin="normal"
                />
              </Grid>
              <Grid item xs={12} md={6}>
                <FormControl fullWidth margin="normal">
                  <InputLabel>Type</InputLabel>
                  <Select label="Type">
                    <MenuItem value="project">Project</MenuItem>
                    <MenuItem value="task">Task</MenuItem>
                    <MenuItem value="module">Module</MenuItem>
                    <MenuItem value="workflow">Workflow</MenuItem>
                  </Select>
                </FormControl>
              </Grid>
              <Grid item xs={12}>
                <TextField
                  fullWidth
                  label="Description"
                  multiline
                  rows={3}
                  margin="normal"
                />
              </Grid>
              <Grid item xs={12} md={6}>
                <FormControl fullWidth margin="normal">
                  <InputLabel>Status</InputLabel>
                  <Select label="Status">
                    <MenuItem value="active">Active</MenuItem>
                    <MenuItem value="inactive">Inactive</MenuItem>
                    <MenuItem value="completed">Completed</MenuItem>
                    <MenuItem value="archived">Archived</MenuItem>
                  </Select>
                </FormControl>
              </Grid>
              <Grid item xs={12} md={6}>
                <FormControl fullWidth margin="normal">
                  <InputLabel>Priority</InputLabel>
                  <Select label="Priority">
                    <MenuItem value="low">Low</MenuItem>
                    <MenuItem value="medium">Medium</MenuItem>
                    <MenuItem value="high">High</MenuItem>
                    <MenuItem value="critical">Critical</MenuItem>
                  </Select>
                </FormControl>
              </Grid>
            </Grid>
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowItemDialog(false)}>Cancel</Button>
          <Button variant="contained" onClick={handleCreateItem}>
            Create Item
          </Button>
        </DialogActions>
      </Dialog>

      {/* Create Drop Zone Dialog */}
      <Dialog open={showZoneDialog} onClose={() => setShowZoneDialog(false)} maxWidth="sm" fullWidth>
        <DialogTitle>Create New Drop Zone</DialogTitle>
        <DialogContent>
          <Box sx={{ pt: 2 }}>
            <TextField
              fullWidth
              label="Zone Name"
              margin="normal"
            />
            
            <FormControl fullWidth margin="normal">
              <InputLabel>Zone Type</InputLabel>
              <Select label="Zone Type">
                <MenuItem value="folder">Folder</MenuItem>
                <MenuItem value="board">Board</MenuItem>
                <MenuItem value="timeline">Timeline</MenuItem>
                <MenuItem value="workflow">Workflow</MenuItem>
              </Select>
            </FormControl>
            
            <TextField
              fullWidth
              label="Max Items"
              type="number"
              margin="normal"
            />
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowZoneDialog(false)}>Cancel</Button>
          <Button variant="contained" onClick={handleCreateZone}>
            Create Zone
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default DragDropOrganization; 