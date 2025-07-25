"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.UserStatus = exports.WebSocketEventType = void 0;
// WebSocket message types
var WebSocketEventType;
(function (WebSocketEventType) {
    // Connection events
    WebSocketEventType["CONNECT"] = "connect";
    WebSocketEventType["DISCONNECT"] = "disconnect";
    WebSocketEventType["RECONNECT"] = "reconnect";
    // Authentication events
    WebSocketEventType["AUTHENTICATE"] = "authenticate";
    WebSocketEventType["AUTHENTICATION_SUCCESS"] = "authentication_success";
    WebSocketEventType["AUTHENTICATION_FAILED"] = "authentication_failed";
    // User presence events
    WebSocketEventType["USER_ONLINE"] = "user_online";
    WebSocketEventType["USER_OFFLINE"] = "user_offline";
    WebSocketEventType["USER_STATUS_UPDATE"] = "user_status_update";
    // Room/Channel events
    WebSocketEventType["JOIN_ROOM"] = "join_room";
    WebSocketEventType["LEAVE_ROOM"] = "leave_room";
    WebSocketEventType["ROOM_MESSAGE"] = "room_message";
    WebSocketEventType["ROOM_USER_JOINED"] = "room_user_joined";
    WebSocketEventType["ROOM_USER_LEFT"] = "room_user_left";
    // Collaboration events
    WebSocketEventType["COLLABORATION_START"] = "collaboration_start";
    WebSocketEventType["COLLABORATION_UPDATE"] = "collaboration_update";
    WebSocketEventType["COLLABORATION_END"] = "collaboration_end";
    // System events
    WebSocketEventType["HEARTBEAT"] = "heartbeat";
    WebSocketEventType["ERROR"] = "error";
    WebSocketEventType["NOTIFICATION"] = "notification";
    // Data synchronization events
    WebSocketEventType["DATA_SYNC"] = "data_sync";
    WebSocketEventType["DATA_UPDATE"] = "data_update";
    WebSocketEventType["DATA_REQUEST"] = "data_request";
    // Project events
    WebSocketEventType["PROJECT_UPDATE"] = "project_update";
    WebSocketEventType["PROJECT_USER_JOINED"] = "project_user_joined";
    WebSocketEventType["PROJECT_USER_LEFT"] = "project_user_left";
    // Session events
    WebSocketEventType["SESSION_START"] = "session_start";
    WebSocketEventType["SESSION_UPDATE"] = "session_update";
    WebSocketEventType["SESSION_END"] = "session_end";
})(WebSocketEventType = exports.WebSocketEventType || (exports.WebSocketEventType = {}));
// User status
var UserStatus;
(function (UserStatus) {
    UserStatus["ONLINE"] = "online";
    UserStatus["OFFLINE"] = "offline";
    UserStatus["AWAY"] = "away";
    UserStatus["BUSY"] = "busy";
    UserStatus["IN_SESSION"] = "in_session";
})(UserStatus = exports.UserStatus || (exports.UserStatus = {}));
