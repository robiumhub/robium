const { execSync } = require('child_process');
const fs = require('fs');
const path = require('path');

// Create a simple build directory
const buildDir = path.join(__dirname, 'build');
if (!fs.existsSync(buildDir)) {
  fs.mkdirSync(buildDir, { recursive: true });
}

// Create a simple index.html
const indexHtml = `
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <meta name="theme-color" content="#000000" />
    <title>Robium Frontend</title>
  </head>
  <body>
    <noscript>You need to enable JavaScript to run this app.</noscript>
    <div id="root"></div>
  </body>
</html>
`;

fs.writeFileSync(path.join(buildDir, 'index.html'), indexHtml);

// Create a simple static directory
const staticDir = path.join(buildDir, 'static');
if (!fs.existsSync(staticDir)) {
  fs.mkdirSync(staticDir, { recursive: true });
}

console.log('Simple build completed successfully!');
console.log('Build directory created at:', buildDir); 