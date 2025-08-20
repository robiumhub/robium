const { createProxyMiddleware } = require('http-proxy-middleware');

module.exports = function (app) {
  const httpTarget = process.env.BACKEND_URL || 'http://localhost:8000';
  const wsTarget = httpTarget.replace('http', 'ws');

  // Single proxy rule for all API routes
  app.use(
    '/api',
    createProxyMiddleware({
      target: httpTarget,
      changeOrigin: true,
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
      target: wsTarget,
      ws: true,
      changeOrigin: true,
      onError: (err, req, res) => {
        console.error('WebSocket proxy error:', err);
      },
    })
  );
};
