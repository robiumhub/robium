import React, { useState, useRef, useEffect } from 'react';
import {
  Box,
  Drawer,
  AppBar,
  Toolbar,
  Typography,
  IconButton,
  Paper,
  Tabs,
  Tab,
  Divider,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  Collapse,
  Chip,
  Badge,
  Tooltip,
  Fab,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  Button,
  TextField,
  Menu,
  MenuItem,
} from '@mui/material';
import {
  Folder as FolderIcon,
  Chat as ChatIcon,
  Terminal as TerminalIcon,
  Dashboard as DashboardIcon,
  Settings as SettingsIcon,
  Extension as ExtensionIcon,
  Code as CodeIcon,
  Visibility as VisibilityIcon,
  PlayArrow as PlayIcon,
  Close as CloseIcon,
  Menu as MenuIcon,
  ChevronLeft as ChevronLeftIcon,
  ChevronRight as ChevronRightIcon,
  ExpandMore as ExpandMoreIcon,
  ExpandLess as ExpandLessIcon,
  Add as AddIcon,
  MoreVert as MoreIcon,
  Refresh as RefreshIcon,
  Save as SaveIcon,
  Build as BuildIcon,
  BugReport as DebugIcon,
  CloudUpload as DeployIcon,
} from '@mui/icons-material';
import { useParams, useNavigate } from 'react-router-dom';
import { ApiService } from '../services/api';

// Types
interface FileNode {
  id: string;
  name: string;
  type: 'file' | 'directory';
  path: string;
  children?: FileNode[];
  isOpen?: boolean;
}

interface ChatMessage {
  id: string;
  type: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

interface TerminalSession {
  id: string;
  title: string;
  isActive: boolean;
}

interface TabItem {
  id: string;
  title: string;
  type: 'overview' | 'simulation' | 'rviz' | 'editor';
  content?: string;
  isDirty?: boolean;
}

// Main Component
const ProjectWorkspace: React.FC = () => {
  const { projectId } = useParams<{ projectId: string }>();
  const navigate = useNavigate();
  
  // State for panes
  const [leftPaneWidth, setLeftPaneWidth] = useState(300);
  const [rightPaneWidth, setRightPaneWidth] = useState(350);
  const [bottomPaneHeight, setBottomPaneHeight] = useState(200);
  
  // State for collapsible panes
  const [leftPanes, setLeftPanes] = useState({
    fileManager: { isOpen: true, isCollapsed: false },
    dashboard: { isOpen: true, isCollapsed: false },
    modules: { isOpen: true, isCollapsed: false },
    settings: { isOpen: false, isCollapsed: false },
  });
  
  const [rightPanes, setRightPanes] = useState({
    chat: { isOpen: true, isCollapsed: false },
    extensions: { isOpen: false, isCollapsed: false },
  });
  
  // State for main content
  const [activeTab, setActiveTab] = useState(0);
  const [tabs, setTabs] = useState<TabItem[]>([
    { id: 'overview', title: 'Project Overview', type: 'overview' },
    { id: 'simulation', title: 'Simulation', type: 'simulation' },
    { id: 'rviz', title: 'RViz Visualization', type: 'rviz' },
  ]);
  
  // State for file manager
  const [fileTree, setFileTree] = useState<FileNode[]>([
    {
      id: '1',
      name: 'src',
      type: 'directory',
      path: '/src',
      isOpen: true,
      children: [
        { id: '2', name: 'main.py', type: 'file', path: '/src/main.py' },
        { id: '3', name: 'config.yaml', type: 'file', path: '/src/config.yaml' },
      ],
    },
    {
      id: '4',
      name: 'packages',
      type: 'directory',
      path: '/packages',
      isOpen: false,
      children: [
        { id: '5', name: 'navigation', type: 'directory', path: '/packages/navigation' },
        { id: '6', name: 'perception', type: 'directory', path: '/packages/perception' },
      ],
    },
    { id: '7', name: 'README.md', type: 'file', path: '/README.md' },
    { id: '8', name: 'package.xml', type: 'file', path: '/package.xml' },
  ]);
  
  // State for chat
  const [chatMessages, setChatMessages] = useState<ChatMessage[]>([
    {
      id: '1',
      type: 'assistant',
      content: 'Hello! I\'m your AI assistant for this Robium project. How can I help you today?',
      timestamp: new Date(),
    },
  ]);
  const [chatInput, setChatInput] = useState('');
  
  // State for terminal
  const [terminalSessions, setTerminalSessions] = useState<TerminalSession[]>([
    { id: '1', title: 'Terminal 1', isActive: true },
  ]);
  const [activeTerminal, setActiveTerminal] = useState('1');
  
  // Refs for resizing
  const leftResizeRef = useRef<HTMLDivElement>(null);
  const rightResizeRef = useRef<HTMLDivElement>(null);
  const bottomResizeRef = useRef<HTMLDivElement>(null);
  
  // Resize handlers
  const handleLeftResize = (e: React.MouseEvent) => {
    const startX = e.clientX;
    const startWidth = leftPaneWidth;
    
    const handleMouseMove = (e: MouseEvent) => {
      const newWidth = startWidth + (e.clientX - startX);
      setLeftPaneWidth(Math.max(200, Math.min(500, newWidth)));
    };
    
    const handleMouseUp = () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };
    
    document.addEventListener('mousemove', handleMouseMove);
    document.addEventListener('mouseup', handleMouseUp);
  };
  
  const handleRightResize = (e: React.MouseEvent) => {
    const startX = e.clientX;
    const startWidth = rightPaneWidth;
    
    const handleMouseMove = (e: MouseEvent) => {
      const newWidth = startWidth - (e.clientX - startX);
      setRightPaneWidth(Math.max(250, Math.min(600, newWidth)));
    };
    
    const handleMouseUp = () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };
    
    document.addEventListener('mousemove', handleMouseMove);
    document.addEventListener('mouseup', handleMouseUp);
  };
  
  const handleBottomResize = (e: React.MouseEvent) => {
    const startY = e.clientY;
    const startHeight = bottomPaneHeight;
    
    const handleMouseMove = (e: MouseEvent) => {
      const newHeight = startHeight - (e.clientY - startY);
      setBottomPaneHeight(Math.max(150, Math.min(400, newHeight)));
    };
    
    const handleMouseUp = () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };
    
    document.addEventListener('mousemove', handleMouseMove);
    document.addEventListener('mouseup', handleMouseUp);
  };
  
  // Toggle pane visibility
  const togglePane = (paneType: string, side: 'left' | 'right') => {
    if (side === 'left') {
      setLeftPanes(prev => ({
        ...prev,
        [paneType]: { ...prev[paneType as keyof typeof prev], isOpen: !prev[paneType as keyof typeof prev].isOpen }
      }));
    } else {
      setRightPanes(prev => ({
        ...prev,
        [paneType]: { ...prev[paneType as keyof typeof prev], isOpen: !prev[paneType as keyof typeof prev].isOpen }
      }));
    }
  };
  
  // Toggle pane collapse
  const togglePaneCollapse = (paneType: string, side: 'left' | 'right') => {
    if (side === 'left') {
      setLeftPanes(prev => ({
        ...prev,
        [paneType]: { ...prev[paneType as keyof typeof prev], isCollapsed: !prev[paneType as keyof typeof prev].isCollapsed }
      }));
    } else {
      setRightPanes(prev => ({
        ...prev,
        [paneType]: { ...prev[paneType as keyof typeof prev], isCollapsed: !prev[paneType as keyof typeof prev].isCollapsed }
      }));
    }
  };
  
  // File tree handlers
  const toggleFileNode = (nodeId: string) => {
    const updateNode = (nodes: FileNode[]): FileNode[] => {
      return nodes.map(node => {
        if (node.id === nodeId) {
          return { ...node, isOpen: !node.isOpen };
        }
        if (node.children) {
          return { ...node, children: updateNode(node.children) };
        }
        return node;
      });
    };
    setFileTree(updateNode(fileTree));
  };
  
  // Chat handlers
  const sendChatMessage = () => {
    if (!chatInput.trim()) return;
    
    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      type: 'user',
      content: chatInput,
      timestamp: new Date(),
    };
    
    setChatMessages(prev => [...prev, userMessage]);
    setChatInput('');
    
    // Simulate AI response
    setTimeout(() => {
      const aiMessage: ChatMessage = {
        id: (Date.now() + 1).toString(),
        type: 'assistant',
        content: 'I understand your request. Let me help you with that...',
        timestamp: new Date(),
      };
      setChatMessages(prev => [...prev, aiMessage]);
    }, 1000);
  };
  
  // Tab handlers
  const handleTabChange = (event: React.SyntheticEvent, newValue: number) => {
    setActiveTab(newValue);
  };
  
  const closeTab = (tabId: string) => {
    setTabs(prev => prev.filter(tab => tab.id !== tabId));
  };
  
  // Render file tree
  const renderFileTree = (nodes: FileNode[], level = 0) => {
    return nodes.map(node => (
      <Box key={node.id}>
        <ListItem
          sx={{ pl: level * 2 + 2 }}
          onClick={() => node.type === 'directory' && toggleFileNode(node.id)}
        >
          <ListItemIcon sx={{ minWidth: 32 }}>
            {node.type === 'directory' ? (
              node.isOpen ? <ExpandMoreIcon fontSize="small" /> : <ChevronRightIcon fontSize="small" />
            ) : (
              <CodeIcon fontSize="small" />
            )}
          </ListItemIcon>
          <ListItemText 
            primary={node.name}
            primaryTypographyProps={{ fontSize: '0.875rem' }}
          />
        </ListItem>
        {node.type === 'directory' && node.isOpen && node.children && (
          <Box>{renderFileTree(node.children, level + 1)}</Box>
        )}
      </Box>
    ));
  };
  
  // Render pane content
  const renderPaneContent = (paneType: string, side: 'left' | 'right') => {
    switch (paneType) {
      case 'fileManager':
        return (
          <Box>
            <Box sx={{ p: 1, borderBottom: 1, borderColor: 'divider' }}>
              <Typography variant="subtitle2">EXPLORER</Typography>
            </Box>
            <List dense sx={{ py: 0 }}>
              {renderFileTree(fileTree)}
            </List>
          </Box>
        );
      
      case 'dashboard':
        return (
          <Box>
            <Box sx={{ p: 1, borderBottom: 1, borderColor: 'divider' }}>
              <Typography variant="subtitle2">DASHBOARD</Typography>
            </Box>
            <Box sx={{ p: 2 }}>
              <Typography variant="body2" color="text.secondary">
                Project Status: Active
              </Typography>
              <Chip label="Running" color="success" size="small" sx={{ mt: 1 }} />
            </Box>
          </Box>
        );
      
      case 'modules':
        return (
          <Box>
            <Box sx={{ p: 1, borderBottom: 1, borderColor: 'divider' }}>
              <Typography variant="subtitle2">MODULES</Typography>
            </Box>
            <List dense>
              <ListItem>
                <ListItemIcon><ExtensionIcon fontSize="small" /></ListItemIcon>
                <ListItemText primary="Navigation Core" />
              </ListItem>
              <ListItem>
                <ListItemIcon><ExtensionIcon fontSize="small" /></ListItemIcon>
                <ListItemText primary="Perception Basic" />
              </ListItem>
            </List>
          </Box>
        );
      
      case 'chat':
        return (
          <Box sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
            <Box sx={{ p: 1, borderBottom: 1, borderColor: 'divider' }}>
              <Typography variant="subtitle2">AI ASSISTANT</Typography>
            </Box>
            <Box sx={{ flex: 1, overflow: 'auto', p: 1 }}>
              {chatMessages.map(message => (
                <Box
                  key={message.id}
                  sx={{
                    mb: 1,
                    p: 1,
                    borderRadius: 1,
                    bgcolor: message.type === 'user' ? 'primary.light' : 'grey.100',
                    alignSelf: message.type === 'user' ? 'flex-end' : 'flex-start',
                  }}
                >
                  <Typography variant="body2">{message.content}</Typography>
                </Box>
              ))}
            </Box>
            <Box sx={{ p: 1, borderTop: 1, borderColor: 'divider' }}>
              <TextField
                fullWidth
                size="small"
                placeholder="Ask me anything..."
                value={chatInput}
                onChange={(e) => setChatInput(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && sendChatMessage()}
                InputProps={{
                  endAdornment: (
                    <IconButton size="small" onClick={sendChatMessage}>
                      <ChatIcon />
                    </IconButton>
                  ),
                }}
              />
            </Box>
          </Box>
        );
      
      default:
        return <Typography>Content for {paneType}</Typography>;
    }
  };
  
  // Render collapsible pane
  const renderCollapsiblePane = (paneType: string, side: 'left' | 'right') => {
    const panes = side === 'left' ? leftPanes : rightPanes;
    const pane = panes[paneType as keyof typeof panes] as { isOpen: boolean; isCollapsed: boolean };
    
    if (!pane.isOpen) return null;
    
         return (
       <Paper
         sx={{
           height: '100%',
           display: 'flex',
           flexDirection: 'column',
           overflow: 'hidden',
           boxShadow: 'none',
           borderRadius: 0,
           border: 'none',
         }}
       >
                 <Box
           sx={{
             display: 'flex',
             alignItems: 'center',
             p: 0.5,
             borderBottom: 1,
             borderColor: 'divider',
             bgcolor: 'grey.100',
             minHeight: 32,
           }}
         >
          <Typography variant="caption" sx={{ flex: 1, fontWeight: 'bold' }}>
            {paneType.toUpperCase()}
          </Typography>
          <IconButton
            size="small"
            onClick={() => togglePaneCollapse(paneType, side)}
          >
            {pane.isCollapsed ? <ExpandMoreIcon /> : <ExpandLessIcon />}
          </IconButton>
          <IconButton
            size="small"
            onClick={() => togglePane(paneType, side)}
          >
            <CloseIcon fontSize="small" />
          </IconButton>
        </Box>
        {!pane.isCollapsed && (
          <Box sx={{ flex: 1, overflow: 'auto' }}>
            {renderPaneContent(paneType, side)}
          </Box>
        )}
      </Paper>
    );
  };
  
  return (
    <Box sx={{ height: '100vh', display: 'flex', flexDirection: 'column', bgcolor: 'grey.100' }}>
      {/* Top App Bar */}
      <AppBar position="static" elevation={1}>
        <Toolbar variant="dense">
          <IconButton edge="start" color="inherit" onClick={() => navigate('/projects')}>
            <ChevronLeftIcon />
          </IconButton>
          <Typography variant="h6" sx={{ flex: 1 }}>
            Robium Project: {projectId}
          </Typography>
          <Box sx={{ display: 'flex', gap: 1 }}>
            <Tooltip title="Back to Projects">
              <IconButton color="inherit" size="small" onClick={() => navigate('/projects')}>
                <FolderIcon />
              </IconButton>
            </Tooltip>
            <Tooltip title="Build">
              <IconButton color="inherit" size="small">
                <BuildIcon />
              </IconButton>
            </Tooltip>
            <Tooltip title="Debug">
              <IconButton color="inherit" size="small">
                <DebugIcon />
              </IconButton>
            </Tooltip>
            <Tooltip title="Deploy">
              <IconButton color="inherit" size="small">
                <DeployIcon />
              </IconButton>
            </Tooltip>
          </Box>
        </Toolbar>
      </AppBar>
      
             {/* Main Content */}
       <Box sx={{ flex: 1, display: 'flex', overflow: 'hidden', bgcolor: 'white' }}>
                 {/* Left Sidebar */}
         <Box sx={{ 
           width: leftPaneWidth, 
           display: 'flex', 
           flexDirection: 'column',
           bgcolor: 'grey.50',
           borderRight: 1,
           borderColor: 'divider'
         }}>
           {Object.keys(leftPanes).map(paneType => (
             <Box key={paneType} sx={{ flex: 1, mb: 1 }}>
               {renderCollapsiblePane(paneType, 'left')}
             </Box>
           ))}
         </Box>
        
        {/* Left Resize Handle */}
        <Box
          ref={leftResizeRef}
          sx={{
            width: 4,
            bgcolor: 'grey.300',
            cursor: 'col-resize',
            '&:hover': { bgcolor: 'grey.400' },
          }}
          onMouseDown={handleLeftResize}
        />
        
        {/* Center Content */}
        <Box sx={{ flex: 1, display: 'flex', flexDirection: 'column' }}>
          {/* Main Tabs */}
          <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
            <Tabs value={activeTab} onChange={handleTabChange} variant="scrollable">
              {tabs.map((tab, index) => (
                <Tab
                  key={tab.id}
                  label={
                    <Box sx={{ display: 'flex', alignItems: 'center' }}>
                      {tab.title}
                      {tab.isDirty && <Chip label="•" size="small" sx={{ ml: 1 }} />}
                      <IconButton
                        size="small"
                        onClick={(e) => {
                          e.stopPropagation();
                          closeTab(tab.id);
                        }}
                        sx={{ ml: 1 }}
                      >
                        <CloseIcon fontSize="small" />
                      </IconButton>
                    </Box>
                  }
                />
              ))}
            </Tabs>
          </Box>
          
          {/* Tab Content */}
          <Box sx={{ flex: 1, overflow: 'auto', p: 2 }}>
            {tabs[activeTab] && (
              <Box>
                {tabs[activeTab].type === 'overview' && (
                  <Typography variant="h5">Project Overview</Typography>
                )}
                {tabs[activeTab].type === 'simulation' && (
                  <Typography variant="h5">Simulation Pane</Typography>
                )}
                {tabs[activeTab].type === 'rviz' && (
                  <Typography variant="h5">RViz Visualization</Typography>
                )}
                {tabs[activeTab].type === 'editor' && (
                  <Typography variant="h5">File Editor</Typography>
                )}
              </Box>
            )}
          </Box>
          
          {/* Bottom Resize Handle */}
          <Box
            ref={bottomResizeRef}
            sx={{
              height: 4,
              bgcolor: 'grey.300',
              cursor: 'row-resize',
              '&:hover': { bgcolor: 'grey.400' },
            }}
            onMouseDown={handleBottomResize}
          />
          
                     {/* Terminal */}
           <Box sx={{ height: bottomPaneHeight, borderTop: 1, borderColor: 'divider', bgcolor: 'black' }}>
             <Box sx={{ borderBottom: 1, borderColor: 'grey.700', bgcolor: 'grey.900' }}>
               <Tabs 
                 value={activeTerminal} 
                 onChange={(e, v) => setActiveTerminal(v)}
                 sx={{
                   '& .MuiTab-root': {
                     color: 'grey.400',
                     '&.Mui-selected': {
                       color: 'white',
                     },
                   },
                   '& .MuiTabs-indicator': {
                     backgroundColor: 'primary.main',
                   },
                 }}
               >
                 {terminalSessions.map(session => (
                   <Tab key={session.id} value={session.id} label={session.title} />
                 ))}
               </Tabs>
             </Box>
             <Box sx={{ height: 'calc(100% - 48px)', bgcolor: 'black', color: 'white', p: 1 }}>
               <Typography variant="body2" fontFamily="monospace" sx={{ color: 'green.400' }}>
                 $ Welcome to Robium Terminal
               </Typography>
               <Typography variant="body2" fontFamily="monospace" sx={{ color: 'grey.400' }}>
                 Type 'help' for available commands
               </Typography>
             </Box>
           </Box>
        </Box>
        
        {/* Right Resize Handle */}
        <Box
          ref={rightResizeRef}
          sx={{
            width: 4,
            bgcolor: 'grey.300',
            cursor: 'col-resize',
            '&:hover': { bgcolor: 'grey.400' },
          }}
          onMouseDown={handleRightResize}
        />
        
                 {/* Right Sidebar */}
         <Box sx={{ 
           width: rightPaneWidth, 
           display: 'flex', 
           flexDirection: 'column',
           bgcolor: 'grey.50',
           borderLeft: 1,
           borderColor: 'divider'
         }}>
           {Object.keys(rightPanes).map(paneType => (
             <Box key={paneType} sx={{ flex: 1, mb: 1 }}>
               {renderCollapsiblePane(paneType, 'right')}
             </Box>
           ))}
         </Box>
      </Box>
    </Box>
  );
};

export default ProjectWorkspace; 