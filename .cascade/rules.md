# Robium Project Rules for AI Assistants

## Project Overview
Robium is a comprehensive robotics IDE that enables visual programming through drag-and-drop interfaces. It integrates deeply with ROS (Robot Operating System) and provides a suite of robotics algorithms for easy application development.

## Core Technologies
- **Frontend**: React + TypeScript, Material-UI, React Flow, Monaco Editor
- **Backend**: Node.js/Python, ROS Noetic/Humble, Docker containers
- **Database**: PostgreSQL for user data, Redis for caching
- **AI Integration**: MCP (Model Context Protocol) for chat assistance
- **Deployment**: Kubernetes, Docker Compose for development

## Development Guidelines

### Code Style & Quality
- **TypeScript**: Strict mode enabled, proper type definitions required
- **React**: Functional components with hooks, no class components
- **Naming**: Use descriptive names, camelCase for variables, PascalCase for components
- **File Structure**: Group by feature, not by file type
- **Testing**: Minimum 80% code coverage, unit tests for all components
- **Documentation**: JSDoc for all public APIs, inline comments for complex logic

### Component Development Rules
1. **Interface Compliance**: All robotics components must follow the schema in `docs/COMPONENT_INTERFACE_SPEC.md`
2. **ROS Message Types**: Use standard ROS message types for interfaces
3. **Validation**: Implement input validation for all component parameters
4. **Error Handling**: Graceful error handling with user-friendly messages
5. **Resource Management**: Declare CPU/memory requirements for all components

### Frontend Development
- **Responsive Design**: Mobile-first approach, test on all breakpoints
- **Accessibility**: WCAG 2.1 AA compliance, proper ARIA labels
- **Performance**: Lazy loading, virtual scrolling for large lists
- **State Management**: Use Zustand for global state, React state for local
- **API Integration**: Use Axios with proper error handling and loading states

### Backend Development
- **ROS Integration**: Support both ROS Noetic and ROS2 Humble
- **Container Security**: Non-root containers, minimal base images
- **API Design**: RESTful APIs with proper HTTP status codes
- **Authentication**: JWT tokens with refresh mechanism
- **Resource Limits**: Implement per-user resource quotas

### File Organization
```
robium/
├── frontend/src/
│   ├── components/        # Reusable UI components
│   ├── pages/            # Page-level components
│   ├── hooks/            # Custom React hooks
│   ├── services/         # API services
│   ├── stores/           # Zustand stores
│   ├── types/            # TypeScript type definitions
│   └── utils/            # Utility functions
├── backend/src/
│   ├── controllers/      # Route controllers
│   ├── services/         # Business logic
│   ├── models/           # Data models
│   ├── middleware/       # Express middleware
│   └── utils/            # Utility functions
└── components/           # Robotics component definitions
```

## Specific Implementation Rules

### Drag & Drop Interface
- Use `@dnd-kit/core` for drag and drop functionality
- Implement proper drop zone highlighting and validation
- Provide visual feedback during drag operations
- Support keyboard navigation for accessibility

### Canvas System
- Use React Flow for the main canvas implementation
- Implement custom node types for robotics components
- Support infinite canvas with smooth zoom/pan
- Implement connection validation between components

### Component Registry
- Components must be defined in YAML format following the spec
- Implement component search with fuzzy matching
- Support component categories and tagging
- Validate component interfaces before registration

### Real-time Features
- Use Socket.IO for real-time collaboration
- Implement operational transformation for conflict resolution
- Provide real-time status updates for running components
- Support live data visualization from ROS topics

## AI Assistant Integration

### Chat Interface Guidelines
- Implement context-aware responses based on current project state
- Support natural language component suggestions
- Provide code generation for ROS nodes
- Offer debugging assistance with error resolution

### MCP Integration
- Follow MCP protocol for tool integration
- Implement component search and suggestion tools
- Provide project analysis and optimization suggestions
- Support automated connection recommendations

## Security Requirements

### Authentication & Authorization
- Implement OAuth 2.0 with JWT tokens
- Use role-based access control (RBAC)
- Secure API endpoints with proper middleware
- Implement session management with refresh tokens

### Container Security
- Run containers with non-root users
- Implement resource limits and quotas
- Scan container images for vulnerabilities
- Use network policies for container isolation

### Data Protection
- Encrypt sensitive data at rest and in transit
- Implement proper input validation and sanitization
- Use HTTPS for all communications
- Regular security audits and dependency updates

## Performance Guidelines

### Frontend Performance
- Implement code splitting and lazy loading
- Use React.memo for expensive components
- Optimize bundle size with tree shaking
- Implement proper caching strategies

### Backend Performance
- Use connection pooling for database connections
- Implement caching with Redis
- Optimize database queries with proper indexing
- Use horizontal scaling for high availability

### ROS Performance
- Optimize message passing between nodes
- Use appropriate queue sizes for topics
- Implement proper resource management
- Monitor system performance metrics

## Testing Strategy

### Unit Testing
- Test all component interfaces and validation
- Mock external dependencies (ROS, database)
- Test error conditions and edge cases
- Maintain high code coverage (80%+)

### Integration Testing
- Test component connections and data flow
- Test ROS workspace creation and management
- Test container deployment and scaling
- Test real-time collaboration features

### End-to-End Testing
- Test complete user workflows
- Test drag and drop functionality
- Test project creation and deployment
- Test cross-browser compatibility

## Documentation Requirements

### Code Documentation
- JSDoc comments for all public APIs
- Inline comments for complex algorithms
- README files for each major component
- Architecture decision records (ADRs)

### User Documentation
- Component usage examples and tutorials
- API documentation with examples
- Deployment and setup guides
- Troubleshooting and FAQ sections

## Deployment Guidelines

### Development Environment
- Use Docker Compose for local development
- Implement hot reloading for faster development
- Use environment variables for configuration
- Provide setup scripts for easy onboarding

### Production Deployment
- Use Kubernetes for container orchestration
- Implement blue-green deployment strategy
- Use infrastructure as code (Terraform)
- Implement comprehensive monitoring and logging

## Common Patterns to Follow

### Error Handling
```typescript
// Always provide user-friendly error messages
try {
  await connectComponents(source, target);
} catch (error) {
  if (error instanceof ValidationError) {
    showUserError('Components are not compatible. Please check input/output types.');
  } else {
    showUserError('Failed to connect components. Please try again.');
    logError(error);
  }
}
```

### Component Definition
```yaml
# Always follow the component specification
apiVersion: robium.io/v1
kind: Component
metadata:
  name: component-name
  version: "1.0.0"
  description: "Clear description"
spec:
  interfaces:
    inputs: []
    outputs: []
  parameters: []
  implementation: {}
```

### React Component Structure
```typescript
// Use this pattern for all React components
interface ComponentProps {
  // Define all props with proper types
}

export const Component: React.FC<ComponentProps> = ({ prop1, prop2 }) => {
  // Custom hooks first
  const [state, setState] = useState();
  
  // Event handlers
  const handleEvent = useCallback(() => {
    // Implementation
  }, [dependencies]);
  
  // Render
  return (
    <div>
      {/* JSX content */}
    </div>
  );
};
```

## AI Assistant Behavior

When working on this project, AI assistants should:
1. **Always reference the documentation** in `/docs/` before making architectural decisions
2. **Follow the component interface specification** when creating or modifying robotics components
3. **Implement proper error handling** with user-friendly messages
4. **Consider accessibility** in all UI implementations
5. **Optimize for performance** especially in drag-and-drop interactions
6. **Maintain consistency** with the established patterns and conventions
7. **Test thoroughly** before suggesting code changes
8. **Document changes** appropriately in code and commit messages

## Project-Specific Considerations

### ROS Integration
- Always consider ROS message compatibility when connecting components
- Implement proper ROS node lifecycle management
- Handle ROS master connection failures gracefully
- Support both ROS1 and ROS2 where possible

### Visual Programming
- Prioritize intuitive user experience in drag-and-drop interactions
- Provide clear visual feedback for all user actions
- Implement proper validation for component connections
- Support undo/redo functionality for all canvas operations

### Collaboration Features
- Design for real-time multi-user editing
- Implement conflict resolution for simultaneous edits
- Provide clear indicators of other users' actions
- Support project sharing and permissions management
