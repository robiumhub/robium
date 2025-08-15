const { createProxyMiddleware } = require('http-proxy-middleware');

module.exports = function (app) {
  // Single proxy rule for all API routes
  app.use(
    '/api',
    createProxyMiddleware({
      target: 'http://backend:8000', // Use Docker service name
      changeOrigin: true,
      pathRewrite: {
        '^/api': '', // Remove /api prefix when forwarding to backend
      },
      onError: (err, req, res) => {
        console.error('Proxy error:', err);
        res.status(500).json({
          error: 'Proxy error',
          message: 'Backend server is not responding',
        });
      },
    })
  );

  // WebSocket proxy for real-time communication
  app.use(
    '/ws',
    createProxyMiddleware({
      target: 'ws://backend:8000', // Use Docker service name
      ws: true, // Enable WebSocket proxying
      changeOrigin: true,
      onError: (err, req, res) => {
        console.error('WebSocket proxy error:', err);
      },
    })
  );
};
