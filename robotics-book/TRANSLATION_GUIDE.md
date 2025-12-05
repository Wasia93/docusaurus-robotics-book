# Translation Guide

How to translate all documentation pages to multiple languages.

## Current Status

✅ **Fully Translated:**
- UI elements (navigation, search, buttons)
- Welcome/intro page (7 languages)

❌ **Not Translated:**
- All other documentation pages (35+ pages)
- Module content
- Assessment pages

## Why Only Intro Page is Translated

Docusaurus i18n requires **each page** to be translated individually:
- Translated page exists → shows in that language
- No translation → shows default English (with translated UI)

We only created `intro.md` translations, so other pages show English content.

## Translation Options

### Option 1: AI-Powered Translation (Fastest)

Use the provided script to automatically translate all pages:

**Requirements:**
```bash
npm install openai dotenv
```

**Setup:**
1. Get OpenAI API key from https://platform.openai.com/api-keys
2. Add to `.env` file:
   ```
   OPENAI_API_KEY=sk-your-key-here
   ```

**Run:**
```bash
node scripts/translate-docs.js
```

**What it does:**
- Translates all 35+ documentation files
- Creates translations for all 6 languages
- Preserves markdown formatting
- Keeps code blocks in English
- Translates only the content text

**Cost:** ~$2-5 for all translations (one-time)

**Time:** ~30-60 minutes (with API rate limits)

### Option 2: Docusaurus CLI (Semi-Manual)

Extract strings and translate manually:

```bash
# Extract translatable strings
npm run write-translations -- --locale ur

# This creates i18n/ur/code.json and placeholder files
# You then manually translate each file
```

**Pros:** Free, full control
**Cons:** Very time-consuming (hours of work per language)

### Option 3: Professional Translation Service

Use services like:
- Crowdin (integrates with Docusaurus)
- Lokalise
- Phrase

**Pros:** High quality, professional
**Cons:** Expensive ($500-2000+), requires setup

### Option 4: Partial Translation

Translate only important pages:

```bash
# Translate specific pages manually
# Copy English page, translate content, save to i18n folder
```

For example:
- Module intros
- Getting started pages
- Key concept pages

## Recommended Approach

### For Your Use Case:

**Use AI-Powered Translation (Option 1)**

Why:
- Fast (30-60 minutes)
- Cheap ($2-5 total)
- Good quality for technical content
- Can review and edit afterwards

### Steps:

1. **Install dependencies:**
   ```bash
   cd robotics-book
   npm install openai dotenv
   ```

2. **Get OpenAI API key:**
   - Go to https://platform.openai.com/api-keys
   - Create new key
   - Add to `.env` file

3. **Run translation script:**
   ```bash
   node scripts/translate-docs.js
   ```

4. **Review translations:**
   - Check a few translated pages
   - Fix any formatting issues
   - Edit technical terms if needed

5. **Build and test:**
   ```bash
   npm run build
   npm run serve
   ```

6. **Commit and deploy:**
   ```bash
   git add i18n/
   git commit -m "Add complete translations for all pages"
   git push
   ```

## Manual Translation

If you prefer manual translation, follow this structure:

```
i18n/
├── ur/                                    # Urdu
│   ├── code.json                          # UI translations
│   └── docusaurus-plugin-content-docs/
│       └── current/
│           ├── intro.md                   # Translated pages
│           ├── foundations/
│           │   ├── intro-physical-ai.md
│           │   └── humanoid-landscape.md
│           └── module1-ros2/
│               ├── ros2-overview.md
│               └── ...
└── fr/                                    # French
    └── (same structure)
```

**For each page:**
1. Copy English markdown file
2. Translate content (keep frontmatter, code blocks)
3. Save to corresponding i18n folder
4. Test the page

## Translation Quality Tips

### Do Translate:
✅ Headings and titles
✅ Paragraphs and explanations
✅ Lists and bullet points
✅ Image alt text
✅ Link text
✅ Comments in code

### Don't Translate:
❌ Code syntax
❌ Variable names
❌ Function names
❌ URLs and links
❌ Command-line commands
❌ Technical abbreviations (ROS, AI, API)
❌ YAML frontmatter

### Example:

**English:**
```markdown
## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a framework for robotics.

\`\`\`bash
sudo apt install ros-humble-desktop
\`\`\`
```

**Urdu:**
```markdown
## ROS 2 کا تعارف

ROS 2 (Robot Operating System 2) روبوٹکس کے لیے ایک فریم ورک ہے۔

\`\`\`bash
sudo apt install ros-humble-desktop
\`\`\`
```

## Testing Translations

1. **Build all locales:**
   ```bash
   npm run build
   ```

2. **Test locally:**
   ```bash
   npm run serve
   ```

3. **Check each language:**
   - Switch language in dropdown
   - Navigate to different pages
   - Verify formatting is correct
   - Check RTL languages (Urdu, Sindhi, Pashto)

## Cost Estimation

### AI Translation (Recommended):
- **Total pages**: ~35
- **Languages**: 6
- **Total translations**: 210 pages
- **Average tokens per page**: ~2000
- **Total tokens**: ~420,000
- **Cost**: $0.15 per 1M tokens
- **Total cost**: **~$0.06** (practically free!)

### Professional Translation:
- **Per word rate**: $0.05-0.15
- **Total words**: ~50,000
- **Total cost**: **$2,500-7,500**

## Maintenance

After initial translation:

**New pages:**
- Use AI script for new files
- Or manually translate

**Updates:**
- Re-run script on changed files
- Or manually update translations

**Keeping in sync:**
- Use version control
- Track which files need re-translation
- Consider using Crowdin for ongoing maintenance

## Questions?

See:
- [Docusaurus i18n Docs](https://docusaurus.io/docs/i18n/tutorial)
- `scripts/translate-docs.js` - Translation script
- `.env.example` - Configuration template

---

**Next Steps:**
1. Run the AI translation script
2. Review generated translations
3. Commit and deploy
4. Your entire site will be multilingual! 🌍
