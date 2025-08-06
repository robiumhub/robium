async function debugVolumeItem() {
  console.log('🧪 Debugging Volume Item Conversion...\n');

  const volumeString = '/host/path:/container/path';
  
  console.log('1. Volume string:');
  console.log('   Value:', volumeString);
  console.log('   Type:', typeof volumeString);
  console.log('   String():', String(volumeString));
  console.log('   String() type:', typeof String(volumeString));

  // Test with array
  const volumes = [volumeString];
  
  console.log('\n2. Volumes array:');
  console.log('   Array:', volumes);
  console.log('   Array type:', typeof volumes);
  console.log('   First item:', volumes[0]);
  console.log('   First item type:', typeof volumes[0]);
  console.log('   String(first item):', String(volumes[0]));

  // Test template processing
  const template = `{{#each volumes}}
- {{this}}
{{/each}}`;

  const context = { volumes };
  
  console.log('\n3. Template context:');
  console.log('   Context:', context);
  console.log('   Volumes in context:', context.volumes);
  console.log('   Volumes type:', typeof context.volumes);
  console.log('   Is array:', Array.isArray(context.volumes));

  // Test the template engine
  const { templateEngine } = require('../services/TemplateEngine');
  templateEngine.registerTemplate('debug-volume-item', template);
  
  const result = templateEngine.processTemplate('debug-volume-item', context);
  
  console.log('\n4. Template processing result:');
  console.log('   Content:');
  console.log(result.content);
  console.log('   Errors:', result.errors);
}

debugVolumeItem().catch(console.error); 