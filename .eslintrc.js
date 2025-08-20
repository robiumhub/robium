module.exports = {
  root: true,
  parser: '@typescript-eslint/parser',
  extends: ['eslint:recommended', '@typescript-eslint/recommended', 'prettier'],
  plugins: ['@typescript-eslint'],
  env: {
    node: true,
    es2020: true,
  },
  overrides: [
    {
      files: ['packages/frontend/**/*'],
      env: {
        browser: true,
      },
      extends: ['plugin:react/recommended'],
      plugins: ['react'],
      settings: {
        react: {
          version: 'detect',
        },
      },
    },
  ],
};
