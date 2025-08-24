const sqlite3 = require('sqlite3').verbose();
const path = require('path');

// Test the exact same database path that the backend should be using
const dbPath = path.join(process.cwd(), 'packages/backend/data/robium.db');
console.log('Testing database path:', dbPath);

const db = new sqlite3.Database(dbPath);

db.get(
  'SELECT id, email, username, password_hash, role, is_active FROM users WHERE email = ?',
  ['admin@robium.com'],
  (err, row) => {
    if (err) {
      console.error('Database error:', err);
    } else {
      console.log('User lookup result:', row);
    }
    db.close();
  }
);
