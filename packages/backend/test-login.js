const bcrypt = require('bcryptjs');
const sqlite3 = require('sqlite3').verbose();
const path = require('path');

async function testLogin() {
  const dbPath = path.join(__dirname, 'data/robium.db');
  console.log('Database path:', dbPath);

  const db = new sqlite3.Database(dbPath);

  try {
    // Test 1: Check if user exists
    const user = await new Promise((resolve, reject) => {
      db.get(
        'SELECT id, email, username, password_hash, role, is_active FROM users WHERE email = ?',
        ['admin@robium.com'],
        (err, row) => {
          if (err) reject(err);
          else resolve(row);
        }
      );
    });

    console.log('User found:', user);

    if (user) {
      // Test 2: Check password
      const isValidPassword = await bcrypt.compare('password123', user.password_hash);
      console.log('Password valid:', isValidPassword);

      // Test 3: Check if user is active
      console.log('User is active:', user.is_active === 1);
    }
  } catch (error) {
    console.error('Error:', error);
  } finally {
    db.close();
  }
}

testLogin();
