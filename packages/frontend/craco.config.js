const CracoAlias = require('craco-alias');

module.exports = {
  webpack: {
    configure: (webpackConfig) => {
      // Disable ForkTsCheckerWebpackPlugin to prevent memory issues
      webpackConfig.plugins = webpackConfig.plugins.filter(
        (plugin) => plugin.constructor.name !== 'ForkTsCheckerWebpackPlugin'
      );
      
      // Increase memory limit for other processes
      webpackConfig.optimization = {
        ...webpackConfig.optimization,
        splitChunks: {
          chunks: 'all',
          cacheGroups: {
            vendor: {
              test: /[\\/]node_modules[\\/]/,
              name: 'vendors',
              chunks: 'all',
            },
            // Split Material-UI into separate chunk
            mui: {
              test: /[\\/]node_modules[\\/]@mui[\\/]/,
              name: 'mui',
              chunks: 'all',
              priority: 10,
            },
          },
        },
        // Reduce memory usage
        runtimeChunk: 'single',
      };
      
      // Add memory optimization
      webpackConfig.performance = {
        ...webpackConfig.performance,
        maxEntrypointSize: 512000,
        maxAssetSize: 512000,
        hints: 'warning',
      };
      
      return webpackConfig;
    },
  },
  plugins: [
    {
      plugin: CracoAlias,
      options: {
        source: 'tsconfig',
        baseUrl: './src',
        tsConfigPath: './tsconfig.json',
      },
    },
  ],
}; 