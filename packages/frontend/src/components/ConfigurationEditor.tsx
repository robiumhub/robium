import React, { useState, useEffect, useCallback } from 'react';
import { useConfigurationValidation } from '../hooks/useValidation';
import { ValidationFeedback } from './ValidationFeedback';
import {
  Box,
  Typography,
  Card,
  CardContent,
  Tabs,
  Tab,
  Button,
  Alert,
  CircularProgress,
  Chip,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  TextField,
  IconButton,
  Tooltip,
  Paper,
  Divider,
  List,
  ListItem,
  ListItemText,
  ListItemIcon,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Grid,
} from '@mui/material';
import {
  Code as CodeIcon,
  CheckCircle as CheckCircleIcon,
  Error as ErrorIcon,
  Warning as WarningIcon,
  Info as InfoIcon,
  Save as SaveIcon,
  Refresh as RefreshIcon,
  FormatIndentIncrease as FormatIcon,
  Settings as SettingsIcon,
  Help as HelpIcon,
  ExpandMore as ExpandMoreIcon,
  PlayArrow as PlayIcon,
  Stop as StopIcon,
  Download as DownloadIcon,
  Upload as UploadIcon,
} from '@mui/icons-material';

// Types
interface ValidationError {
  path: string;
  message: string;
  severity: 'error' | 'warning' | 'info';
  line?: number;
  column?: number;
}

interface SchemaField {
  name: string;
  type: string;
  required: boolean;
  description: string;
  default?: any;
  enum?: string[];
  pattern?: string;
  minimum?: number;
  maximum?: number;
}

interface Schema {
  type: string;
  properties: Record<string, SchemaField>;
  required: string[];
  title: string;
  description: string;
}

interface ConfigurationTemplate {
  id: string;
  name: string;
  description: string;
  category: string;
  content: string;
  schema: Schema;
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
      id={`config-tabpanel-${index}`}
      aria-labelledby={`config-tab-${index}`}
      {...other}
    >
      {value === index && <Box sx={{ p: 3 }}>{children}</Box>}
    </div>
  );
}

interface ConfigurationEditorProps {
  initialValue?: string;
  schema?: Schema;
  onSave?: (content: string, format: 'json' | 'yaml') => void;
  onValidate?: (content: string, format: 'json' | 'yaml') => ValidationError[];
  templates?: ConfigurationTemplate[];
  readOnly?: boolean;
}

const ConfigurationEditor: React.FC<ConfigurationEditorProps> = ({
  initialValue = '',
  schema,
  onSave,
  onValidate,
  templates = [],
  readOnly = false,
}) => {
  const [tabValue, setTabValue] = useState(0);
  const [content, setContent] = useState(initialValue);
  const [format, setFormat] = useState<'json' | 'yaml'>('json');
  const [validationErrors, setValidationErrors] = useState<ValidationError[]>([]);
  const [isValidating, setIsValidating] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [showTemplateDialog, setShowTemplateDialog] = useState(false);
  const [showSchemaDialog, setShowSchemaDialog] = useState(false);
  const [selectedTemplate, setSelectedTemplate] = useState<ConfigurationTemplate | null>(null);
  const [autoComplete, setAutoComplete] = useState(true);
  const [syntaxHighlighting, setSyntaxHighlighting] = useState(true);
  const [lineNumbers, setLineNumbers] = useState(true);
  const [wordWrap, setWordWrap] = useState(false);
  const [fontSize, setFontSize] = useState(14);
  const [theme, setTheme] = useState<'light' | 'dark'>('light');

  // Real-time validation
  const validation = useConfigurationValidation(content, format, schema, {
    debounceDelay: 500,
    validateOnChange: true
  });

  // Mock validation function - in real app this would use a proper JSON schema validator
  const validateContent = useCallback(async (content: string, format: 'json' | 'yaml'): Promise<ValidationError[]> => {
    const errors: ValidationError[] = [];
    
    try {
      if (format === 'json') {
        JSON.parse(content);
      } else {
        // In real app, would use a YAML parser
        // For now, just basic validation
        if (!content.trim()) {
          errors.push({
            path: '',
            message: 'Empty content',
            severity: 'warning',
            line: 1,
            column: 1,
          });
        }
      }
    } catch (error) {
      errors.push({
        path: '',
        message: `Invalid ${format.toUpperCase()}: ${error instanceof Error ? error.message : 'Unknown error'}`,
        severity: 'error',
        line: 1,
        column: 1,
      });
    }

    // Mock schema validation
    if (schema) {
      try {
        const parsed = format === 'json' ? JSON.parse(content) : content;
        // In real app, would validate against schema
        // For now, just check for required fields
        schema.required?.forEach(field => {
          if (!parsed[field]) {
            errors.push({
              path: field,
              message: `Required field '${field}' is missing`,
              severity: 'error',
              line: 1,
              column: 1,
            });
          }
        });
      } catch (error) {
        // Ignore parsing errors as they're already caught above
      }
    }

    return errors;
  }, [schema]);

  // Auto-format content
  const formatContent = useCallback(() => {
    if (format === 'json') {
      try {
        const parsed = JSON.parse(content);
        setContent(JSON.stringify(parsed, null, 2));
      } catch (error) {
        // Don't format if invalid JSON
      }
    } else {
      // In real app, would use a YAML formatter
      // For now, just trim whitespace
      setContent(content.trim());
    }
  }, [content, format]);

  // Validate content on change
  useEffect(() => {
    if (content && autoComplete) {
      const timeoutId = setTimeout(async () => {
        setIsValidating(true);
        const errors = await validateContent(content, format);
        setValidationErrors(errors);
        setIsValidating(false);
      }, 500);

      return () => clearTimeout(timeoutId);
    }
  }, [content, format, validateContent, autoComplete]);

  // Handle save
  const handleSave = async () => {
    setIsSaving(true);
    try {
      const errors = await validateContent(content, format);
      if (errors.some(e => e.severity === 'error')) {
        setValidationErrors(errors);
        return;
      }
      
      if (onSave) {
        await onSave(content, format);
      }
    } catch (error) {
      console.error('Save failed:', error);
    } finally {
      setIsSaving(false);
    }
  };

  // Handle template selection
  const handleTemplateSelect = (template: ConfigurationTemplate) => {
    setSelectedTemplate(template);
    setContent(template.content);
    setFormat('json'); // Templates are typically JSON
    setShowTemplateDialog(false);
  };

  // Get error count by severity
  const getErrorCount = (severity: 'error' | 'warning' | 'info') => {
    return validationErrors.filter(e => e.severity === severity).length;
  };

  // Mock templates
  const defaultTemplates: ConfigurationTemplate[] = [
    {
      id: 'ros2-basic',
      name: 'ROS2 Basic Configuration',
      description: 'Basic ROS2 node configuration with common parameters',
      category: 'ROS2',
      content: JSON.stringify({
        node_name: 'my_ros2_node',
        namespace: '/robot',
        parameters: {
          rate: 10,
          timeout: 5.0,
          debug: false
        },
        topics: {
          input: '/sensor/data',
          output: '/processed/data'
        }
      }, null, 2),
      schema: {
        type: 'object',
        properties: {
          node_name: { name: 'node_name', type: 'string', required: true, description: 'Name of the ROS2 node' },
          namespace: { name: 'namespace', type: 'string', required: true, description: 'Namespace for the node' },
          parameters: { name: 'parameters', type: 'object', required: false, description: 'Node parameters' },
          topics: { name: 'topics', type: 'object', required: false, description: 'Topic configuration' }
        },
        required: ['node_name', 'namespace'],
        title: 'ROS2 Node Configuration',
        description: 'Configuration for a ROS2 node'
      }
    },
    {
      id: 'docker-compose',
      name: 'Docker Compose Configuration',
      description: 'Docker Compose configuration for ROS2 development environment',
      category: 'Docker',
      content: JSON.stringify({
        version: '3.8',
        services: {
          ros2_node: {
            image: 'ros:humble',
            container_name: 'ros2_development',
            volumes: ['./src:/workspace/src'],
            ports: ['11311:11311'],
            environment: {
              ROS_DOMAIN_ID: '0'
            }
          }
        }
      }, null, 2),
      schema: {
        type: 'object',
        properties: {
          version: { name: 'version', type: 'string', required: true, description: 'Docker Compose version' },
          services: { name: 'services', type: 'object', required: true, description: 'Service definitions' }
        },
        required: ['version', 'services'],
        title: 'Docker Compose Configuration',
        description: 'Docker Compose configuration for development environment'
      }
    },
    {
      id: 'gazebo-world',
      name: 'Gazebo World Configuration',
      description: 'Gazebo world configuration for robot simulation',
      category: 'Simulation',
      content: JSON.stringify({
        world_name: 'empty_world',
        physics: {
          engine: 'ode',
          max_step_size: 0.001,
          real_time_factor: 1.0
        },
        models: [
          {
            name: 'robot',
            file: 'package://my_robot_description/urdf/robot.urdf',
            pose: [0, 0, 0, 0, 0, 0]
          }
        ]
      }, null, 2),
      schema: {
        type: 'object',
        properties: {
          world_name: { name: 'world_name', type: 'string', required: true, description: 'Name of the Gazebo world' },
          physics: { name: 'physics', type: 'object', required: false, description: 'Physics engine configuration' },
          models: { name: 'models', type: 'array', required: false, description: 'List of models in the world' }
        },
        required: ['world_name'],
        title: 'Gazebo World Configuration',
        description: 'Configuration for Gazebo simulation world'
      }
    }
  ];

  const allTemplates = [...templates, ...defaultTemplates];

  return (
    <Box>
      {/* Header */}
      <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', mb: 3 }}>
        <Typography variant="h5" component="h2">
          Configuration Editor
        </Typography>
        <Box sx={{ display: 'flex', gap: 2, alignItems: 'center' }}>
          {/* Real-time Validation Status */}
          <ValidationFeedback
            errors={validation.configErrors}
            warnings={validation.configWarnings}
            info={validation.configInfo}
            isValid={validation.validationState.isValid}
            isValidationInProgress={validation.validationState.isValidationInProgress}
            compact={true}
            showSummary={true}
            maxErrors={3}
            maxWarnings={2}
            maxInfo={1}
          />
          
          {/* Format Selector */}
          <FormControl size="small">
            <Select
              value={format}
              onChange={(e) => setFormat(e.target.value as 'json' | 'yaml')}
              sx={{ minWidth: 100 }}
            >
              <MenuItem value="json">JSON</MenuItem>
              <MenuItem value="yaml">YAML</MenuItem>
            </Select>
          </FormControl>
          
          {/* Action Buttons */}
          <Button
            variant="outlined"
            startIcon={<FormatIcon />}
            onClick={formatContent}
            disabled={!content}
          >
            Format
          </Button>
          <Button
            variant="outlined"
            startIcon={<SettingsIcon />}
            onClick={() => setShowSchemaDialog(true)}
          >
            Settings
          </Button>
          <Button
            variant="contained"
            startIcon={<SaveIcon />}
            onClick={handleSave}
            disabled={isSaving || readOnly}
          >
            {isSaving ? <CircularProgress size={20} /> : 'Save'}
          </Button>
        </Box>
      </Box>

      {/* Main Editor */}
      <Card>
        <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
          <Tabs value={tabValue} onChange={(_, newValue) => setTabValue(newValue)}>
            <Tab icon={<CodeIcon />} label="Editor" />
            <Tab icon={<InfoIcon />} label="Schema" />
            <Tab icon={<HelpIcon />} label="Templates" />
            <Tab icon={<ErrorIcon />} label="Validation" />
          </Tabs>
        </Box>

        {/* Editor Tab */}
        <TabPanel value={tabValue} index={0}>
          <Box sx={{ position: 'relative' }}>
            {/* Editor Toolbar */}
            <Box sx={{ display: 'flex', gap: 2, mb: 2, alignItems: 'center' }}>
              <Button
                variant="outlined"
                size="small"
                onClick={() => setShowTemplateDialog(true)}
              >
                Load Template
              </Button>
              <Button
                variant="outlined"
                size="small"
                startIcon={<DownloadIcon />}
                onClick={() => {
                  const blob = new Blob([content], { type: 'text/plain' });
                  const url = URL.createObjectURL(blob);
                  const a = document.createElement('a');
                  a.href = url;
                  a.download = `config.${format}`;
                  a.click();
                  URL.revokeObjectURL(url);
                }}
              >
                Export
              </Button>
              <Button
                variant="outlined"
                size="small"
                startIcon={<UploadIcon />}
                onClick={() => {
                  const input = document.createElement('input');
                  input.type = 'file';
                  input.accept = `.${format}`;
                  input.onchange = (e) => {
                    const file = (e.target as HTMLInputElement).files?.[0];
                    if (file) {
                      const reader = new FileReader();
                      reader.onload = (e) => {
                        setContent(e.target?.result as string);
                      };
                      reader.readAsText(file);
                    }
                  };
                  input.click();
                }}
              >
                Import
              </Button>
            </Box>

            {/* Code Editor */}
            <Paper
              variant="outlined"
              sx={{
                position: 'relative',
                backgroundColor: theme === 'dark' ? '#1e1e1e' : '#fafafa',
                minHeight: '400px',
                maxHeight: '600px',
                overflow: 'auto',
              }}
            >
              <TextField
                multiline
                fullWidth
                value={content}
                onChange={(e) => setContent(e.target.value)}
                disabled={readOnly}
                sx={{
                  '& .MuiInputBase-root': {
                    fontFamily: 'monospace',
                    fontSize: `${fontSize}px`,
                    lineHeight: 1.5,
                    backgroundColor: 'transparent',
                  },
                  '& .MuiInputBase-input': {
                    padding: 2,
                    whiteSpace: wordWrap ? 'normal' : 'pre',
                    wordBreak: wordWrap ? 'break-word' : 'keep-all',
                  },
                }}
                InputProps={{
                  disableUnderline: true,
                }}
                placeholder={`Enter your ${format.toUpperCase()} configuration here...`}
              />
              
              {/* Line Numbers (if enabled) */}
              {lineNumbers && (
                <Box
                  sx={{
                    position: 'absolute',
                    left: 0,
                    top: 0,
                    bottom: 0,
                    width: '40px',
                    backgroundColor: theme === 'dark' ? '#2d2d2d' : '#f0f0f0',
                    borderRight: 1,
                    borderColor: 'divider',
                    fontFamily: 'monospace',
                    fontSize: `${fontSize}px`,
                    lineHeight: 1.5,
                    padding: 2,
                    color: 'text.secondary',
                    userSelect: 'none',
                  }}
                >
                  {content.split('\n').map((_, index) => (
                    <Box key={index} sx={{ textAlign: 'right' }}>
                      {index + 1}
                    </Box>
                  ))}
                </Box>
              )}
            </Paper>

            {/* Validation Status */}
            {isValidating && (
              <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mt: 1 }}>
                <CircularProgress size={16} />
                <Typography variant="body2" color="text.secondary">
                  Validating...
                </Typography>
              </Box>
            )}
          </Box>
        </TabPanel>

        {/* Schema Tab */}
        <TabPanel value={tabValue} index={1}>
          {schema ? (
            <Box>
              <Typography variant="h6" gutterBottom>
                {schema.title}
              </Typography>
              <Typography variant="body2" color="text.secondary" paragraph>
                {schema.description}
              </Typography>
              
              <Typography variant="h6" gutterBottom>
                Schema Fields
              </Typography>
              <List>
                {Object.entries(schema.properties).map(([key, field]) => (
                  <ListItem key={key} divider>
                    <ListItemIcon>
                      {field.required ? <ErrorIcon color="error" /> : <InfoIcon color="info" />}
                    </ListItemIcon>
                    <ListItemText
                      primary={
                        <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                          <Typography variant="subtitle1">{field.name}</Typography>
                          <Chip label={field.type} size="small" />
                          {field.required && <Chip label="Required" size="small" color="error" />}
                        </Box>
                      }
                      secondary={
                        <Box>
                          <Typography variant="body2">{field.description}</Typography>
                          {field.default !== undefined && (
                            <Typography variant="body2" color="text.secondary">
                              Default: {JSON.stringify(field.default)}
                            </Typography>
                          )}
                          {field.enum && (
                            <Typography variant="body2" color="text.secondary">
                              Allowed values: {field.enum.join(', ')}
                            </Typography>
                          )}
                        </Box>
                      }
                    />
                  </ListItem>
                ))}
              </List>
            </Box>
          ) : (
            <Alert severity="info">
              No schema is available for this configuration. Validation will be limited to syntax checking.
            </Alert>
          )}
        </TabPanel>

        {/* Templates Tab */}
        <TabPanel value={tabValue} index={2}>
          <Typography variant="h6" gutterBottom>
            Configuration Templates
          </Typography>
          <Typography variant="body2" color="text.secondary" paragraph>
            Choose a template to get started with a pre-configured setup.
          </Typography>
          
          <Grid container spacing={2}>
            {allTemplates.map((template) => (
              <Grid item xs={12} md={6} lg={4} key={template.id}>
                <Card variant="outlined">
                  <CardContent>
                    <Typography variant="h6" gutterBottom>
                      {template.name}
                    </Typography>
                    <Typography variant="body2" color="text.secondary" paragraph>
                      {template.description}
                    </Typography>
                    <Chip label={template.category} size="small" sx={{ mb: 2 }} />
                    <Button
                      variant="contained"
                      fullWidth
                      onClick={() => handleTemplateSelect(template)}
                    >
                      Use Template
                    </Button>
                  </CardContent>
                </Card>
              </Grid>
            ))}
          </Grid>
        </TabPanel>

        {/* Validation Tab */}
        <TabPanel value={tabValue} index={3}>
          <ValidationFeedback
            errors={validation.configErrors}
            warnings={validation.configWarnings}
            info={validation.configInfo}
            isValid={validation.validationState.isValid}
            isValidationInProgress={validation.validationState.isValidationInProgress}
            showSuggestions={true}
            maxErrors={10}
            maxWarnings={5}
            maxInfo={3}
            title="Configuration Validation Results"
            onErrorClick={(error) => {
              // TODO: Navigate to error location in editor
              console.log('Error clicked:', error);
            }}
            onWarningClick={(warning) => {
              // TODO: Navigate to warning location in editor
              console.log('Warning clicked:', warning);
            }}
            onInfoClick={(info) => {
              // TODO: Navigate to info location in editor
              console.log('Info clicked:', info);
            }}
          />
        </TabPanel>
      </Card>

      {/* Template Selection Dialog */}
      <Dialog
        open={showTemplateDialog}
        onClose={() => setShowTemplateDialog(false)}
        maxWidth="md"
        fullWidth
      >
        <DialogTitle>Select Template</DialogTitle>
        <DialogContent>
          <Grid container spacing={2}>
            {allTemplates.map((template) => (
              <Grid item xs={12} md={6} key={template.id}>
                <Card variant="outlined" sx={{ cursor: 'pointer' }} onClick={() => handleTemplateSelect(template)}>
                  <CardContent>
                    <Typography variant="h6" gutterBottom>
                      {template.name}
                    </Typography>
                    <Typography variant="body2" color="text.secondary" paragraph>
                      {template.description}
                    </Typography>
                    <Chip label={template.category} size="small" />
                  </CardContent>
                </Card>
              </Grid>
            ))}
          </Grid>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowTemplateDialog(false)}>Cancel</Button>
        </DialogActions>
      </Dialog>

      {/* Settings Dialog */}
      <Dialog
        open={showSchemaDialog}
        onClose={() => setShowSchemaDialog(false)}
        maxWidth="sm"
        fullWidth
      >
        <DialogTitle>Editor Settings</DialogTitle>
        <DialogContent>
          <Grid container spacing={2}>
            <Grid item xs={12}>
              <Typography variant="h6" gutterBottom>
                Editor Options
              </Typography>
            </Grid>
            <Grid item xs={12} md={6}>
              <FormControlLabel
                control={
                  <Switch
                    checked={autoComplete}
                    onChange={(e) => setAutoComplete(e.target.checked)}
                  />
                }
                label="Auto-completion"
              />
            </Grid>
            <Grid item xs={12} md={6}>
              <FormControlLabel
                control={
                  <Switch
                    checked={syntaxHighlighting}
                    onChange={(e) => setSyntaxHighlighting(e.target.checked)}
                  />
                }
                label="Syntax highlighting"
              />
            </Grid>
            <Grid item xs={12} md={6}>
              <FormControlLabel
                control={
                  <Switch
                    checked={lineNumbers}
                    onChange={(e) => setLineNumbers(e.target.checked)}
                  />
                }
                label="Line numbers"
              />
            </Grid>
            <Grid item xs={12} md={6}>
              <FormControlLabel
                control={
                  <Switch
                    checked={wordWrap}
                    onChange={(e) => setWordWrap(e.target.checked)}
                  />
                }
                label="Word wrap"
              />
            </Grid>
            <Grid item xs={12} md={6}>
              <TextField
                fullWidth
                label="Font Size"
                type="number"
                value={fontSize}
                onChange={(e) => setFontSize(Number(e.target.value))}
                inputProps={{ min: 10, max: 24 }}
              />
            </Grid>
            <Grid item xs={12} md={6}>
              <FormControl fullWidth>
                <InputLabel>Theme</InputLabel>
                <Select
                  value={theme}
                  onChange={(e) => setTheme(e.target.value as 'light' | 'dark')}
                  label="Theme"
                >
                  <MenuItem value="light">Light</MenuItem>
                  <MenuItem value="dark">Dark</MenuItem>
                </Select>
              </FormControl>
            </Grid>
          </Grid>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowSchemaDialog(false)}>Close</Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default ConfigurationEditor; 