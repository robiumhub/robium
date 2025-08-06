import React from 'react';
import {
  Box,
  Alert,
  AlertTitle,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  Chip,
  Typography,
  Collapse,
  IconButton,
  Tooltip,
  Paper,
  Divider,
  Accordion,
  AccordionSummary,
  AccordionDetails,
} from '@mui/material';
import {
  Error as ErrorIcon,
  Warning as WarningIcon,
  Info as InfoIcon,
  CheckCircle as CheckCircleIcon,
  ExpandMore as ExpandMoreIcon,
  Help as HelpIcon,
  Close as CloseIcon,
  Lightbulb as LightbulbIcon,
} from '@mui/icons-material';
import { ValidationError, getErrorSuggestions } from '../utils/validation';

export interface ValidationFeedbackProps {
  errors: ValidationError[];
  warnings: ValidationError[];
  info: ValidationError[];
  isValid: boolean;
  isValidationInProgress?: boolean;
  showSuggestions?: boolean;
  maxErrors?: number;
  maxWarnings?: number;
  maxInfo?: number;
  onErrorClick?: (error: ValidationError) => void;
  onWarningClick?: (warning: ValidationError) => void;
  onInfoClick?: (info: ValidationError) => void;
  onDismiss?: () => void;
  compact?: boolean;
  showSummary?: boolean;
  title?: string;
}

export const ValidationFeedback: React.FC<ValidationFeedbackProps> = ({
  errors,
  warnings,
  info,
  isValid,
  isValidationInProgress = false,
  showSuggestions = true,
  maxErrors = 5,
  maxWarnings = 3,
  maxInfo = 2,
  onErrorClick,
  onWarningClick,
  onInfoClick,
  onDismiss,
  compact = false,
  showSummary = true,
  title = 'Validation Results'
}) => {
  const [expanded, setExpanded] = React.useState<string | false>(false);
  const [showAllErrors, setShowAllErrors] = React.useState(false);
  const [showAllWarnings, setShowAllWarnings] = React.useState(false);
  const [showAllInfo, setShowAllInfo] = React.useState(false);

  const displayedErrors = showAllErrors ? errors : errors.slice(0, maxErrors);
  const displayedWarnings = showAllWarnings ? warnings : warnings.slice(0, maxWarnings);
  const displayedInfo = showAllInfo ? info : info.slice(0, maxInfo);

  const hasErrors = errors.length > 0;
  const hasWarnings = warnings.length > 0;
  const hasInfo = info.length > 0;
  const hasIssues = hasErrors || hasWarnings || hasInfo;

  const handleAccordionChange = (panel: string) => (
    event: React.SyntheticEvent,
    isExpanded: boolean
  ) => {
    setExpanded(isExpanded ? panel : false);
  };

  const renderErrorItem = (error: ValidationError, index: number) => {
    const suggestions = showSuggestions ? getErrorSuggestions(error) : [];
    
    return (
      <ListItem
        key={`${error.path}-${index}`}
        sx={{
          borderLeft: '4px solid',
          borderColor: 'error.main',
          backgroundColor: 'error.light',
          mb: 1,
          borderRadius: 1,
          cursor: onErrorClick ? 'pointer' : 'default',
          '&:hover': onErrorClick ? {
            backgroundColor: 'error.main',
            color: 'error.contrastText'
          } : {}
        }}
        onClick={() => onErrorClick?.(error)}
      >
        <ListItemIcon sx={{ color: 'inherit', minWidth: 40 }}>
          <ErrorIcon />
        </ListItemIcon>
        <ListItemText
          primary={
            <Typography variant="body2" fontWeight="medium">
              {error.message}
            </Typography>
          }
          secondary={
            <Box>
              <Typography variant="caption" color="inherit" sx={{ opacity: 0.8 }}>
                Path: {error.path}
                {error.line && ` • Line: ${error.line}`}
                {error.column && ` • Column: ${error.column}`}
              </Typography>
              {suggestions.length > 0 && (
                <Box mt={1}>
                  <Typography variant="caption" fontWeight="medium" display="flex" alignItems="center" gap={0.5}>
                    <LightbulbIcon fontSize="small" />
                    Suggestions:
                  </Typography>
                  <List dense sx={{ py: 0 }}>
                    {suggestions.map((suggestion, idx) => (
                      <ListItem key={idx} sx={{ py: 0.5 }}>
                        <ListItemText
                          primary={
                            <Typography variant="caption" sx={{ opacity: 0.9 }}>
                              • {suggestion}
                            </Typography>
                          }
                        />
                      </ListItem>
                    ))}
                  </List>
                </Box>
              )}
            </Box>
          }
        />
      </ListItem>
    );
  };

  const renderWarningItem = (warning: ValidationError, index: number) => {
    const suggestions = showSuggestions ? getErrorSuggestions(warning) : [];
    
    return (
      <ListItem
        key={`${warning.path}-${index}`}
        sx={{
          borderLeft: '4px solid',
          borderColor: 'warning.main',
          backgroundColor: 'warning.light',
          mb: 1,
          borderRadius: 1,
          cursor: onWarningClick ? 'pointer' : 'default',
          '&:hover': onWarningClick ? {
            backgroundColor: 'warning.main',
            color: 'warning.contrastText'
          } : {}
        }}
        onClick={() => onWarningClick?.(warning)}
      >
        <ListItemIcon sx={{ color: 'inherit', minWidth: 40 }}>
          <WarningIcon />
        </ListItemIcon>
        <ListItemText
          primary={
            <Typography variant="body2" fontWeight="medium">
              {warning.message}
            </Typography>
          }
          secondary={
            <Box>
              <Typography variant="caption" color="inherit" sx={{ opacity: 0.8 }}>
                Path: {warning.path}
                {warning.line && ` • Line: ${warning.line}`}
                {warning.column && ` • Column: ${warning.column}`}
              </Typography>
              {suggestions.length > 0 && (
                <Box mt={1}>
                  <Typography variant="caption" fontWeight="medium" display="flex" alignItems="center" gap={0.5}>
                    <LightbulbIcon fontSize="small" />
                    Suggestions:
                  </Typography>
                  <List dense sx={{ py: 0 }}>
                    {suggestions.map((suggestion, idx) => (
                      <ListItem key={idx} sx={{ py: 0.5 }}>
                        <ListItemText
                          primary={
                            <Typography variant="caption" sx={{ opacity: 0.9 }}>
                              • {suggestion}
                            </Typography>
                          }
                        />
                      </ListItem>
                    ))}
                  </List>
                </Box>
              )}
            </Box>
          }
        />
      </ListItem>
    );
  };

  const renderInfoItem = (info: ValidationError, index: number) => {
    const suggestions = showSuggestions ? getErrorSuggestions(info) : [];
    
    return (
      <ListItem
        key={`${info.path}-${index}`}
        sx={{
          borderLeft: '4px solid',
          borderColor: 'info.main',
          backgroundColor: 'info.light',
          mb: 1,
          borderRadius: 1,
          cursor: onInfoClick ? 'pointer' : 'default',
          '&:hover': onInfoClick ? {
            backgroundColor: 'info.main',
            color: 'info.contrastText'
          } : {}
        }}
        onClick={() => onInfoClick?.(info)}
      >
        <ListItemIcon sx={{ color: 'inherit', minWidth: 40 }}>
          <InfoIcon />
        </ListItemIcon>
        <ListItemText
          primary={
            <Typography variant="body2" fontWeight="medium">
              {info.message}
            </Typography>
          }
          secondary={
            <Box>
              <Typography variant="caption" color="inherit" sx={{ opacity: 0.8 }}>
                Path: {info.path}
                {info.line && ` • Line: ${info.line}`}
                {info.column && ` • Column: ${info.column}`}
              </Typography>
              {suggestions.length > 0 && (
                <Box mt={1}>
                  <Typography variant="caption" fontWeight="medium" display="flex" alignItems="center" gap={0.5}>
                    <LightbulbIcon fontSize="small" />
                    Suggestions:
                  </Typography>
                  <List dense sx={{ py: 0 }}>
                    {suggestions.map((suggestion, idx) => (
                      <ListItem key={idx} sx={{ py: 0.5 }}>
                        <ListItemText
                          primary={
                            <Typography variant="caption" sx={{ opacity: 0.9 }}>
                              • {suggestion}
                            </Typography>
                          }
                        />
                      </ListItem>
                    ))}
                  </List>
                </Box>
              )}
            </Box>
          }
        />
      </ListItem>
    );
  };

  if (isValidationInProgress) {
    return (
      <Alert severity="info" icon={<InfoIcon />}>
        <AlertTitle>Validating...</AlertTitle>
        Please wait while we validate your input.
      </Alert>
    );
  }

  if (!hasIssues && isValid) {
    return (
      <Alert severity="success" icon={<CheckCircleIcon />}>
        <AlertTitle>Valid</AlertTitle>
        All validation checks have passed successfully.
      </Alert>
    );
  }

  if (compact) {
    return (
      <Box>
        {showSummary && (
          <Box sx={{ display: 'flex', gap: 1, mb: 2, alignItems: 'center' }}>
            {hasErrors && (
              <Chip
                icon={<ErrorIcon />}
                label={`${errors.length} error${errors.length !== 1 ? 's' : ''}`}
                color="error"
                size="small"
              />
            )}
            {hasWarnings && (
              <Chip
                icon={<WarningIcon />}
                label={`${warnings.length} warning${warnings.length !== 1 ? 's' : ''}`}
                color="warning"
                size="small"
              />
            )}
            {hasInfo && (
              <Chip
                icon={<InfoIcon />}
                label={`${info.length} info`}
                color="info"
                size="small"
              />
            )}
            {onDismiss && (
              <IconButton size="small" onClick={onDismiss}>
                <CloseIcon />
              </IconButton>
            )}
          </Box>
        )}
        
        {hasErrors && (
          <Alert severity="error" sx={{ mb: 1 }}>
            <Box>
              {displayedErrors.map((error, index) => (
                <Typography key={index} variant="body2">
                  • {error.message}
                </Typography>
              ))}
              {errors.length > maxErrors && !showAllErrors && (
                <Typography variant="body2" sx={{ mt: 1, cursor: 'pointer' }} onClick={() => setShowAllErrors(true)}>
                  Show {errors.length - maxErrors} more errors...
                </Typography>
              )}
            </Box>
          </Alert>
        )}
        
        {hasWarnings && (
          <Alert severity="warning" sx={{ mb: 1 }}>
            <Box>
              {displayedWarnings.map((warning, index) => (
                <Typography key={index} variant="body2">
                  • {warning.message}
                </Typography>
              ))}
              {warnings.length > maxWarnings && !showAllWarnings && (
                <Typography variant="body2" sx={{ mt: 1, cursor: 'pointer' }} onClick={() => setShowAllWarnings(true)}>
                  Show {warnings.length - maxWarnings} more warnings...
                </Typography>
              )}
            </Box>
          </Alert>
        )}
        
        {hasInfo && (
          <Alert severity="info">
            <Box>
              {displayedInfo.map((info, index) => (
                <Typography key={index} variant="body2">
                  • {info.message}
                </Typography>
              ))}
              {info.length > maxInfo && !showAllInfo && (
                <Typography variant="body2" sx={{ mt: 1, cursor: 'pointer' }} onClick={() => setShowAllInfo(true)}>
                  Show {info.length - maxInfo} more info...
                </Typography>
              )}
            </Box>
          </Alert>
        )}
      </Box>
    );
  }

  return (
    <Paper sx={{ p: 2 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
        <Typography variant="h6" component="h3">
          {title}
        </Typography>
        {onDismiss && (
          <IconButton size="small" onClick={onDismiss}>
            <CloseIcon />
          </IconButton>
        )}
      </Box>

      {showSummary && (
        <Box sx={{ display: 'flex', gap: 1, mb: 2 }}>
          {hasErrors && (
            <Chip
              icon={<ErrorIcon />}
              label={`${errors.length} error${errors.length !== 1 ? 's' : ''}`}
              color="error"
            />
          )}
          {hasWarnings && (
            <Chip
              icon={<WarningIcon />}
              label={`${warnings.length} warning${warnings.length !== 1 ? 's' : ''}`}
              color="warning"
            />
          )}
          {hasInfo && (
            <Chip
              icon={<InfoIcon />}
              label={`${info.length} info`}
              color="info"
            />
          )}
        </Box>
      )}

      {hasErrors && (
        <Accordion
          expanded={expanded === 'errors'}
          onChange={handleAccordionChange('errors')}
          sx={{ mb: 1 }}
        >
          <AccordionSummary expandIcon={<ExpandMoreIcon />}>
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
              <ErrorIcon color="error" />
              <Typography variant="subtitle1" color="error">
                Errors ({errors.length})
              </Typography>
            </Box>
          </AccordionSummary>
          <AccordionDetails>
            <List>
              {displayedErrors.map((error, index) => renderErrorItem(error, index))}
              {errors.length > maxErrors && !showAllErrors && (
                <ListItem>
                  <ListItemText
                    primary={
                      <Typography
                        variant="body2"
                        color="primary"
                        sx={{ cursor: 'pointer' }}
                        onClick={() => setShowAllErrors(true)}
                      >
                        Show {errors.length - maxErrors} more errors...
                      </Typography>
                    }
                  />
                </ListItem>
              )}
            </List>
          </AccordionDetails>
        </Accordion>
      )}

      {hasWarnings && (
        <Accordion
          expanded={expanded === 'warnings'}
          onChange={handleAccordionChange('warnings')}
          sx={{ mb: 1 }}
        >
          <AccordionSummary expandIcon={<ExpandMoreIcon />}>
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
              <WarningIcon color="warning" />
              <Typography variant="subtitle1" color="warning.main">
                Warnings ({warnings.length})
              </Typography>
            </Box>
          </AccordionSummary>
          <AccordionDetails>
            <List>
              {displayedWarnings.map((warning, index) => renderWarningItem(warning, index))}
              {warnings.length > maxWarnings && !showAllWarnings && (
                <ListItem>
                  <ListItemText
                    primary={
                      <Typography
                        variant="body2"
                        color="primary"
                        sx={{ cursor: 'pointer' }}
                        onClick={() => setShowAllWarnings(true)}
                      >
                        Show {warnings.length - maxWarnings} more warnings...
                      </Typography>
                    }
                  />
                </ListItem>
              )}
            </List>
          </AccordionDetails>
        </Accordion>
      )}

      {hasInfo && (
        <Accordion
          expanded={expanded === 'info'}
          onChange={handleAccordionChange('info')}
        >
          <AccordionSummary expandIcon={<ExpandMoreIcon />}>
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
              <InfoIcon color="info" />
              <Typography variant="subtitle1" color="info.main">
                Information ({info.length})
              </Typography>
            </Box>
          </AccordionSummary>
          <AccordionDetails>
            <List>
              {displayedInfo.map((info, index) => renderInfoItem(info, index))}
              {info.length > maxInfo && !showAllInfo && (
                <ListItem>
                  <ListItemText
                    primary={
                      <Typography
                        variant="body2"
                        color="primary"
                        sx={{ cursor: 'pointer' }}
                        onClick={() => setShowAllInfo(true)}
                      >
                        Show {info.length - maxInfo} more info...
                      </Typography>
                    }
                  />
                </ListItem>
              )}
            </List>
          </AccordionDetails>
        </Accordion>
      )}
    </Paper>
  );
}; 