import { logger } from './logger';
import { AuthRequest } from '../types';

export interface SecurityAuditEvent {
  eventType:
    | 'ACCESS'
    | 'DATA_RETRIEVAL'
    | 'PERMISSION_CHECK'
    | 'ERROR'
    | 'SECURITY_VIOLATION';
  resource: string;
  action: string;
  userId?: string;
  userEmail?: string;
  userRole?: string;
  ipAddress?: string;
  userAgent?: string;
  requestId?: string;
  timestamp: string;
  success: boolean;
  details?: Record<string, unknown>;
  error?: string;
}

export class SecurityAudit {
  private static instance: SecurityAudit;

  private constructor() {}

  public static getInstance(): SecurityAudit {
    if (!SecurityAudit.instance) {
      SecurityAudit.instance = new SecurityAudit();
    }
    return SecurityAudit.instance;
  }

  public logAccess(
    req: AuthRequest,
    resource: string,
    action: string,
    success: boolean = true,
    details?: Record<string, unknown>
  ): void {
    const event: SecurityAuditEvent = {
      eventType: 'ACCESS',
      resource,
      action,
      userId: req.user?.userId,
      userEmail: req.user?.email,
      userRole: req.user?.role,
      ipAddress: req.ip,
      userAgent: req.get('User-Agent'),
      requestId: req.requestId,
      timestamp: new Date().toISOString(),
      success,
      details,
    };

    this.logEvent(event);
  }

  public logDataRetrieval(
    req: AuthRequest,
    resource: string,
    dataType: string,
    recordCount: number,
    duration: number,
    success: boolean = true
  ): void {
    const event: SecurityAuditEvent = {
      eventType: 'DATA_RETRIEVAL',
      resource,
      action: `RETRIEVE_${dataType.toUpperCase()}`,
      userId: req.user?.userId,
      userEmail: req.user?.email,
      userRole: req.user?.role,
      ipAddress: req.ip,
      userAgent: req.get('User-Agent'),
      requestId: req.requestId,
      timestamp: new Date().toISOString(),
      success,
      details: {
        dataType,
        recordCount,
        duration: `${duration}ms`,
      },
    };

    this.logEvent(event);
  }

  public logPermissionCheck(
    req: AuthRequest,
    permission: string,
    resource: string,
    granted: boolean
  ): void {
    const event: SecurityAuditEvent = {
      eventType: 'PERMISSION_CHECK',
      resource,
      action: `CHECK_PERMISSION_${permission.toUpperCase()}`,
      userId: req.user?.userId,
      userEmail: req.user?.email,
      userRole: req.user?.role,
      ipAddress: req.ip,
      userAgent: req.get('User-Agent'),
      requestId: req.requestId,
      timestamp: new Date().toISOString(),
      success: granted,
      details: {
        permission,
        granted,
      },
    };

    this.logEvent(event);
  }

  public logSecurityViolation(
    req: AuthRequest,
    resource: string,
    action: string,
    violationType: string,
    details?: Record<string, unknown>
  ): void {
    const event: SecurityAuditEvent = {
      eventType: 'SECURITY_VIOLATION',
      resource,
      action,
      userId: req.user?.userId,
      userEmail: req.user?.email,
      userRole: req.user?.role,
      ipAddress: req.ip,
      userAgent: req.get('User-Agent'),
      requestId: req.requestId,
      timestamp: new Date().toISOString(),
      success: false,
      details: {
        violationType,
        ...details,
      },
    };

    this.logEvent(event);
  }

  public logError(
    req: AuthRequest,
    resource: string,
    action: string,
    error: Error,
    details?: Record<string, unknown>
  ): void {
    const event: SecurityAuditEvent = {
      eventType: 'ERROR',
      resource,
      action,
      userId: req.user?.userId,
      userEmail: req.user?.email,
      userRole: req.user?.role,
      ipAddress: req.ip,
      userAgent: req.get('User-Agent'),
      requestId: req.requestId,
      timestamp: new Date().toISOString(),
      success: false,
      error: error.message,
      details: {
        stack: error.stack,
        ...details,
      },
    };

    this.logEvent(event);
  }

  private logEvent(event: SecurityAuditEvent): void {
    // Log to security audit log
    logger.info('Security Audit Event', {
      ...event,
      logCategory: 'SECURITY_AUDIT',
    });

    // For security violations, also log as warning
    if (event.eventType === 'SECURITY_VIOLATION') {
      logger.warn('Security Violation Detected', {
        ...event,
        logCategory: 'SECURITY_VIOLATION',
      });
    }

    // For errors, also log as error
    if (event.eventType === 'ERROR') {
      logger.error('Security Audit Error', {
        ...event,
        logCategory: 'SECURITY_AUDIT_ERROR',
      });
    }
  }
}

// Export singleton instance
export const securityAudit = SecurityAudit.getInstance();
