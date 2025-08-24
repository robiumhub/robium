const axios = require('axios');
require('dotenv').config();

const API_BASE_URL = process.env.API_BASE_URL || 'http://localhost:8000';

async function testGitHubIntegration() {
  try {
    console.log('🧪 Testing GitHub Integration...\n');

    // Test 1: Check if GitHub token is configured
    console.log('1. Checking GitHub token configuration...');
    const hasGitHubToken =
      process.env.GITHUB_TOKEN && process.env.GITHUB_TOKEN !== 'your_github_token_here';
    console.log(`   GitHub Token configured: ${hasGitHubToken ? '✅ Yes' : '❌ No'}`);

    if (!hasGitHubToken) {
      console.log('   ⚠️  Set GITHUB_TOKEN environment variable to test repository creation');
      console.log('   Example: export GITHUB_TOKEN=ghp_your_token_here');
    }

    // Test 2: Check server health
    console.log('\n2. Checking server health...');
    try {
      const healthResponse = await axios.get(`${API_BASE_URL}/health`);
      console.log(`   Server status: ✅ ${healthResponse.status} ${healthResponse.statusText}`);
      console.log(
        `   GitHub integration: ${healthResponse.data.githubIntegration ? '✅ Enabled' : '❌ Disabled'}`
      );
    } catch (error) {
      console.log(`   Server status: ❌ ${error.message}`);
      return;
    }

    // Test 3: Test project creation with GitHub repo (if token is available)
    if (hasGitHubToken) {
      console.log('\n3. Testing project creation with GitHub repository...');

      // First, we need to authenticate (this would normally be done via login)
      console.log('   ⚠️  Note: This test requires authentication. Please login first.');
      console.log('   You can test manually by:');
      console.log('   1. Starting the server: npm run dev');
      console.log('   2. Creating a project via the frontend with GitHub repo option enabled');
      console.log('   3. Or using curl with authentication:');
      console.log(`
   curl -X POST ${API_BASE_URL}/api/projects \\
     -H "Content-Type: application/json" \\
     -H "Authorization: Bearer YOUR_JWT_TOKEN" \\
     -d '{
       "name": "test-github-project",
       "description": "Test project with GitHub integration",
       "github": {
         "createRepo": true,
         "visibility": "public",
         "repoName": "test-github-project"
       }
     }'
      `);
    }

    console.log('\n✅ GitHub integration test completed!');
    console.log('\n📝 Next steps:');
    console.log('1. Set GITHUB_TOKEN environment variable');
    console.log('2. Start the server: npm run dev');
    console.log('3. Create a project via the frontend with GitHub repo option');
    console.log('4. Check that the repository is created on GitHub');
  } catch (error) {
    console.error('❌ Test failed:', error.message);
  }
}

// Run the test
testGitHubIntegration();
