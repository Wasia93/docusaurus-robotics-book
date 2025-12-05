/**
 * Translate all documentation files to multiple languages using OpenAI
 *
 * Usage:
 *   node scripts/translate-docs.js
 *
 * Requirements:
 *   - OpenAI API key in .env file
 *   - npm install openai
 */

const fs = require('fs');
const path = require('path');
const OpenAI = require('openai');
require('dotenv').config();

const openai = new OpenAI({
  apiKey: process.env.OPENAI_API_KEY
});

// Languages to translate to
const languages = {
  ur: 'Urdu',
  sd: 'Sindhi',
  pa: 'Punjabi',
  ps: 'Pashto',
  fr: 'French',
  zh: 'Chinese (Simplified)'
};

// Get all markdown files in docs directory
function getAllMarkdownFiles(dir, fileList = []) {
  const files = fs.readdirSync(dir);

  files.forEach(file => {
    const filePath = path.join(dir, file);
    const stat = fs.statSync(filePath);

    if (stat.isDirectory()) {
      getAllMarkdownFiles(filePath, fileList);
    } else if (file.endsWith('.md')) {
      fileList.push(filePath);
    }
  });

  return fileList;
}

// Translate content using OpenAI
async function translateContent(content, targetLanguage) {
  try {
    const response = await openai.chat.completions.create({
      model: 'gpt-4o-mini',
      messages: [
        {
          role: 'system',
          content: `You are a professional translator specializing in technical and educational content. Translate the following Markdown documentation about robotics and AI to ${targetLanguage}.

IMPORTANT RULES:
1. Preserve ALL markdown formatting (headers, code blocks, links, lists)
2. Do NOT translate code blocks, URLs, or technical commands
3. Keep YAML frontmatter unchanged
4. Maintain technical terms accuracy
5. Keep the same structure and formatting
6. For code examples, only translate comments
7. Preserve special characters and emojis`
        },
        {
          role: 'user',
          content: content
        }
      ],
      temperature: 0.3,
      max_tokens: 4000
    });

    return response.choices[0].message.content;
  } catch (error) {
    console.error(`Translation error: ${error.message}`);
    return null;
  }
}

// Translate a single file
async function translateFile(filePath, targetLang, targetLanguageName) {
  console.log(`\nTranslating ${filePath} to ${targetLanguageName}...`);

  // Read source file
  const content = fs.readFileSync(filePath, 'utf-8');

  // Translate
  const translatedContent = await translateContent(content, targetLanguageName);

  if (!translatedContent) {
    console.error(`  ✗ Failed to translate`);
    return false;
  }

  // Determine output path
  const relativePath = path.relative(path.join(__dirname, '../docs'), filePath);
  const outputPath = path.join(
    __dirname,
    '../i18n',
    targetLang,
    'docusaurus-plugin-content-docs/current',
    relativePath
  );

  // Create directory if needed
  const outputDir = path.dirname(outputPath);
  if (!fs.existsSync(outputDir)) {
    fs.mkdirSync(outputDir, { recursive: true });
  }

  // Write translated file
  fs.writeFileSync(outputPath, translatedContent, 'utf-8');
  console.log(`  ✓ Saved to ${outputPath}`);

  return true;
}

// Main function
async function main() {
  console.log('🌍 Starting Documentation Translation...\n');

  // Check for API key
  if (!process.env.OPENAI_API_KEY) {
    console.error('❌ Error: OPENAI_API_KEY not found in .env file');
    process.exit(1);
  }

  // Get all markdown files
  const docsDir = path.join(__dirname, '../docs');
  const markdownFiles = getAllMarkdownFiles(docsDir);

  console.log(`Found ${markdownFiles.length} documentation files\n`);

  let totalTranslated = 0;
  let totalFailed = 0;

  // Translate each file to each language
  for (const [langCode, langName] of Object.entries(languages)) {
    console.log(`\n${'='.repeat(60)}`);
    console.log(`Translating to ${langName} (${langCode})`);
    console.log('='.repeat(60));

    for (const filePath of markdownFiles) {
      const success = await translateFile(filePath, langCode, langName);

      if (success) {
        totalTranslated++;
      } else {
        totalFailed++;
      }

      // Rate limiting: wait 1 second between requests
      await new Promise(resolve => setTimeout(resolve, 1000));
    }
  }

  console.log(`\n${'='.repeat(60)}`);
  console.log('📊 Translation Summary');
  console.log('='.repeat(60));
  console.log(`Total files: ${markdownFiles.length}`);
  console.log(`Languages: ${Object.keys(languages).length}`);
  console.log(`Successful translations: ${totalTranslated}`);
  console.log(`Failed translations: ${totalFailed}`);
  console.log(`\n✅ Translation complete!`);

  // Estimated cost
  const estimatedTokens = markdownFiles.length * Object.keys(languages).length * 2000;
  const estimatedCost = (estimatedTokens / 1000000) * 0.15; // GPT-4o-mini pricing
  console.log(`\n💰 Estimated cost: $${estimatedCost.toFixed(2)}`);
}

// Run
main().catch(console.error);
