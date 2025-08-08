const { createProxyMiddleware } = require('http-proxy-middleware');

module.exports = function (app) {
  app.use(
    '/api',
    createProxyMiddleware({
      target: 'http://localhost:8000',
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

  // Proxy API requests to backend (only for actual API calls, not frontend routes)
  app.use(
    '/api/projects',
    createProxyMiddleware({
      target: 'http://localhost:8000',
      changeOrigin: true,
      pathRewrite: {
        '^/api/projects': '/projects', // Remove /api prefix when forwarding to backend
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

  app.use(
    '/api/auth',
    createProxyMiddleware({
      target: 'http://localhost:8000',
      changeOrigin: true,
      pathRewrite: {
        '^/api/auth': '/auth', // Remove /api prefix when forwarding to backend
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

  app.use(
    '/api/admin',
    createProxyMiddleware({
      target: 'http://localhost:8000',
      changeOrigin: true,
      pathRewrite: {
        '^/api/admin': '/admin', // Remove /api prefix when forwarding to backend
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
};
