# Quickstart Guide: Physical AI & Humanoid Robotics Course Book

**Date**: 2025-12-02
**Feature**: 001-docusaurus-robotics-book
**Purpose**: Rapid setup guide for developers implementing the Docusaurus robotics book

## Overview

This guide walks you through setting up the development environment, creating the basic Docusaurus structure, implementing custom components, and running tests. Follow the steps in order for a smooth implementation experience.

**Estimated Time**: 60-90 minutes for complete setup

## Prerequisites

Before starting, ensure you have:

- **Node.js**: Version 18 LTS or higher ([Download](https://nodejs.org/))
- **npm**: Version 9+ (comes with Node.js)
- **Git**: For version control ([Download](https://git-scm.com/))
- **Code Editor**: VS Code recommended with extensions:
  - ESLint
  - Prettier
  - MDX
  - TypeScript

**Verify Installation**:
```bash
node --version  # Should show v18.x.x or higher
npm --version   # Should show 9.x.x or higher
git --version   # Should show 2.x.x or higher
```

## Step 1: Initialize Docusaurus Project (10 minutes)

### 1.1 Create Docusaurus Site

```bash
# From repository root
npx create-docusaurus@latest . classic --typescript

# Answer prompts:
# - Template: classic
# - Language: TypeScript
```

This creates the basic Docusaurus structure with TypeScript support.

### 1.2 Verify Basic Setup

```bash
npm start
```

Visit `http://localhost:3000` - you should see the default Docusaurus starter page.

**Success Checkpoint**: Default Docusaurus site loads in browser

### 1.3 Clean Up Starter Content

```bash
# Remove example docs and blog
rm -rf docs/* blog/*
```

## Step 2: Install Additional Dependencies (5 minutes)

### 2.1 Testing Dependencies

```bash
npm install --save-dev \
  @testing-library/react@^14.1.0 \
  @testing-library/jest-dom@^6.1.0 \
  @testing-library/user-event@^14.5.0 \
  jest@^29.7.0 \
  jest-environment-jsdom@^29.7.0 \
  jest-axe@^8.0.0 \
  @playwright/test@^1.40.0 \
  @axe-core/playwright@^4.8.0
```

### 2.2 Search Plugin

```bash
npm install @easyops-cn/docusaurus-search-local
```

### 2.3 Development Tools

```bash
npm install --save-dev \
  eslint@^8.55.0 \
  prettier@^3.1.0 \
  @typescript-eslint/eslint-plugin@^6.15.0 \
  @typescript-eslint/parser@^6.15.0
```

**Success Checkpoint**: All dependencies install without errors

## Step 3: Configure Project Structure (15 minutes)

### 3.1 Create Content Directories

```bash
# Create book part directories
mkdir -p docs/part1-foundations
mkdir -p docs/part2-humanoid
mkdir -p docs/part3-advanced

# Create component directories
mkdir -p src/components/CodeExample
mkdir -p src/components/InteractiveTutorial
mkdir -p src/components/MultimediaEmbed

# Create test directories
mkdir -p tests/e2e
mkdir -p tests/integration
mkdir -p tests/unit/components

# Create static asset directories
mkdir -p static/img/diagrams
mkdir -p static/videos
```

### 3.2 Configure TypeScript

Create `tsconfig.json`:

```json
{
  "extends": "@docusaurus/tsconfig",
  "compilerOptions": {
    "strict": true,
    "noImplicitAny": true,
    "strictNullChecks": true,
    "strictFunctionTypes": true,
    "noUnusedLocals": true,
    "noUnusedParameters": true,
    "noImplicitReturns": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "jsx": "react",
    "module": "ESNext",
    "target": "ES2022",
    "lib": ["ES2022", "DOM"],
    "baseUrl": ".",
    "paths": {
      "@site/*": ["./*"],
      "@theme/*": ["./src/theme/*"]
    }
  },
  "include": ["src/**/*", "docusaurus.config.ts"],
  "exclude": ["node_modules", "build", ".docusaurus"]
}
```

### 3.3 Configure Jest

Create `jest.config.js`:

```javascript
module.exports = {
  preset: '@docusaurus/core/lib/utils/jest/preset',
  testEnvironment: 'jsdom',
  moduleNameMapper: {
    '^@site/(.*)$': '<rootDir>/$1',
    '^@theme/(.*)$': '<rootDir>/node_modules/@docusaurus/theme-classic/src/theme/$1',
    '\\.(css|less|scss|sass)$': 'identity-obj-proxy',
  },
  setupFilesAfterEnv: ['<rootDir>/jest.setup.js'],
  collectCoverageFrom: [
    'src/**/*.{ts,tsx}',
    '!src/**/*.d.ts',
    '!src/**/*.test.{ts,tsx}',
  ],
  coverageThreshold: {
    global: {
      branches: 70,
      functions: 70,
      lines: 70,
      statements: 70,
    },
  },
  testMatch: [
    '<rootDir>/src/**/__tests__/**/*.{ts,tsx}',
    '<rootDir>/src/**/*.{spec,test}.{ts,tsx}',
    '<rootDir>/tests/unit/**/*.{spec,test}.{ts,tsx}',
  ],
};
```

Create `jest.setup.js`:

```javascript
import '@testing-library/jest-dom';
```

### 3.4 Configure Playwright

Create `playwright.config.ts`:

```typescript
import { defineConfig, devices } from '@playwright/test';

export default defineConfig({
  testDir: './tests/e2e',
  fullyParallel: true,
  forbidOnly: !!process.env.CI,
  retries: process.env.CI ? 2 : 0,
  workers: process.env.CI ? 1 : undefined,
  reporter: 'html',
  use: {
    baseURL: 'http://localhost:3000',
    trace: 'on-first-retry',
  },
  projects: [
    {
      name: 'chromium',
      use: { ...devices['Desktop Chrome'] },
    },
    {
      name: 'firefox',
      use: { ...devices['Desktop Firefox'] },
    },
    {
      name: 'mobile',
      use: { ...devices['iPhone 13'] },
    },
  ],
  webServer: {
    command: 'npm run serve',
    url: 'http://localhost:3000',
    reuseExistingServer: !process.env.CI,
  },
});
```

**Success Checkpoint**: All configuration files created

## Step 4: Configure Docusaurus (15 minutes)

### 4.1 Update docusaurus.config.ts

Replace content with:

```typescript
import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Course in Robotics and AI',
  favicon: 'img/favicon.ico',

  url: 'https://your-domain.com',  // TODO: Update with actual domain
  baseUrl: '/',

  organizationName: 'your-org',  // TODO: Update
  projectName: 'robotics-book',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/',  // Docs-only mode
          editUrl: 'https://github.com/your-org/robotics-book/edit/master/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
        blog: false,  // Disable blog
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Robotics Course',
      logo: {
        alt: 'Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/your-org/robotics-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book',
          items: [
            {
              label: 'Part 1: Foundations',
              to: '/part1-foundations/intro-physical-ai',
            },
            {
              label: 'Part 2: Humanoid Robotics',
              to: '/part2-humanoid/bipedal-locomotion',
            },
            {
              label: 'Part 3: Advanced Topics',
              to: '/part3-advanced/deep-learning',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/your-org/robotics-book',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Robotics Course. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'cpp', 'matlab'],
    },
  } satisfies Preset.ThemeConfig,

  plugins: [
    [
      require.resolve('@easyops-cn/docusaurus-search-local'),
      {
        hashed: true,
        indexDocs: true,
        indexBlog: false,
        docsRouteBasePath: '/',
      },
    ],
  ],
};

export default config;
```

### 4.2 Create sidebars.ts

```typescript
import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Part 1: Foundations',
      collapsed: false,
      items: [
        'part1-foundations/intro-physical-ai',
        'part1-foundations/robotics-fundamentals',
        'part1-foundations/sensors-actuators',
        'part1-foundations/control-systems',
      ],
    },
    {
      type: 'category',
      label: 'Part 2: Humanoid Robotics',
      collapsed: true,
      items: [
        'part2-humanoid/bipedal-locomotion',
        'part2-humanoid/motion-planning',
        'part2-humanoid/computer-vision',
        'part2-humanoid/human-robot-interaction',
      ],
    },
    {
      type: 'category',
      label: 'Part 3: Advanced Topics',
      collapsed: true,
      items: [
        'part3-advanced/deep-learning',
        'part3-advanced/reinforcement-learning',
        'part3-advanced/real-world-deployments',
        'part3-advanced/ethics-safety',
      ],
    },
  ],
};

export default sidebars;
```

**Success Checkpoint**: Docusaurus configuration complete

## Step 5: Create Sample Chapter (10 minutes)

### 5.1 Create Introduction Chapter

Create `docs/part1-foundations/intro-physical-ai.md`:

```markdown
---
id: intro-physical-ai
title: Introduction to Physical AI
sidebar_label: Physical AI Intro
sidebar_position: 1
description: An introduction to Physical AI concepts and applications
keywords: [physical ai, robotics, embodied intelligence]
learning_objectives:
  - Understand the definition and scope of Physical AI
  - Identify key differences between traditional AI and Physical AI
  - Recognize applications of Physical AI in robotics
estimated_reading_time: 30 minutes
prerequisites:
  - Basic programming knowledge
  - Interest in robotics and AI
difficulty: beginner
---

# Introduction to Physical AI

## What is Physical AI?

Physical AI refers to artificial intelligence systems that interact with the physical world through sensors and actuators...

## Learning Objectives

By the end of this chapter, you will be able to:

- Define Physical AI and its key components
- Distinguish between traditional AI and Physical AI
- Identify real-world applications of Physical AI

## Key Concepts

### Embodied Intelligence

Embodied intelligence emphasizes that intelligence emerges from the interaction between an agent and its environment...

## Next Steps

Continue to [Robotics Fundamentals](./robotics-fundamentals) to learn about the core principles of robotics systems.
```

### 5.2 Test Chapter Display

```bash
npm start
```

Navigate to `http://localhost:3000` and verify the chapter appears in the sidebar and displays correctly.

**Success Checkpoint**: Sample chapter displays with correct sidebar navigation

## Step 6: Implement Core Components (20 minutes)

### 6.1 CodeExample Component

Create `src/components/CodeExample/CodeExample.tsx`:

```typescript
import React, { useState } from 'react';
import CodeBlock from '@theme/CodeBlock';
import styles from './styles.module.css';

export interface CodeExampleProps {
  code: string;
  language: 'python' | 'javascript' | 'typescript' | 'cpp' | 'bash' | 'yaml';
  title?: string;
  expectedOutput?: string;
  setupInstructions?: string;
  filePath?: string;
  highlightLines?: string;
  isRunnable?: boolean;
}

export default function CodeExample({
  code,
  language,
  title,
  expectedOutput,
  setupInstructions,
  filePath,
  highlightLines,
  isRunnable = false,
}: CodeExampleProps): JSX.Element {
  const [showOutput, setShowOutput] = useState(false);

  return (
    <div className={styles.codeExample}>
      {title && <h4 className={styles.title}>{title}</h4>}
      {filePath && <div className={styles.filePath}>{filePath}</div>}
      {setupInstructions && (
        <div className={styles.setupInstructions}>
          <strong>Setup:</strong> {setupInstructions}
        </div>
      )}
      <CodeBlock
        language={language}
        showLineNumbers
        metastring={highlightLines ? `{${highlightLines}}` : ''}
      >
        {code}
      </CodeBlock>
      {expectedOutput && isRunnable && (
        <div className={styles.outputSection}>
          <button
            onClick={() => setShowOutput(!showOutput)}
            className={styles.outputToggle}
          >
            {showOutput ? 'Hide' : 'Show'} Expected Output
          </button>
          {showOutput && (
            <pre className={styles.expectedOutput}>{expectedOutput}</pre>
          )}
        </div>
      )}
    </div>
  );
}
```

Create `src/components/CodeExample/styles.module.css`:

```css
.codeExample {
  margin: 1.5rem 0;
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: var(--ifm-code-border-radius);
  padding: 1rem;
}

.title {
  margin-top: 0;
  margin-bottom: 0.5rem;
  color: var(--ifm-color-primary);
}

.filePath {
  font-family: var(--ifm-font-family-monospace);
  font-size: 0.875rem;
  color: var(--ifm-color-emphasis-600);
  margin-bottom: 0.5rem;
}

.setupInstructions {
  background-color: var(--ifm-color-warning-contrast-background);
  padding: 0.75rem;
  border-left: 4px solid var(--ifm-color-warning);
  margin-bottom: 1rem;
  border-radius: 4px;
}

.outputSection {
  margin-top: 1rem;
}

.outputToggle {
  background-color: var(--ifm-color-primary);
  color: white;
  border: none;
  padding: 0.5rem 1rem;
  border-radius: 4px;
  cursor: pointer;
}

.outputToggle:hover {
  background-color: var(--ifm-color-primary-dark);
}

.expectedOutput {
  background-color: var(--ifm-code-background);
  padding: 1rem;
  border-radius: 4px;
  margin-top: 0.5rem;
  overflow-x: auto;
}
```

### 6.2 Create Component Test

Create `src/components/CodeExample/CodeExample.test.tsx`:

```typescript
import React from 'react';
import { render, screen } from '@testing-library/react';
import CodeExample from './CodeExample';

describe('CodeExample', () => {
  it('renders code block', () => {
    render(
      <CodeExample
        code="print('Hello, World!')"
        language="python"
        title="Hello World Example"
      />
    );
    expect(screen.getByText('Hello World Example')).toBeInTheDocument();
  });

  it('displays setup instructions when provided', () => {
    render(
      <CodeExample
        code="import numpy as np"
        language="python"
        setupInstructions="Install numpy: pip install numpy"
      />
    );
    expect(screen.getByText(/Install numpy/)).toBeInTheDocument();
  });
});
```

**Success Checkpoint**: Component renders and test passes

## Step 7: Add npm Scripts (5 minutes)

Update `package.json` scripts section:

```json
{
  "scripts": {
    "docusaurus": "docusaurus",
    "start": "docusaurus start",
    "build": "docusaurus build",
    "serve": "docusaurus serve",
    "clear": "docusaurus clear",
    "typecheck": "tsc --noEmit",
    "lint": "eslint src --ext .ts,.tsx",
    "lint:fix": "eslint src --ext .ts,.tsx --fix",
    "format": "prettier --write \"**/*.{ts,tsx,md,mdx,json}\"",
    "format:check": "prettier --check \"**/*.{ts,tsx,md,mdx,json}\"",
    "test": "npm run test:unit && npm run test:e2e",
    "test:unit": "jest",
    "test:unit:watch": "jest --watch",
    "test:unit:coverage": "jest --coverage",
    "test:e2e": "playwright test",
    "test:e2e:ui": "playwright test --ui",
    "test:a11y": "playwright test tests/e2e/accessibility.spec.ts"
  }
}
```

## Step 8: Verify Complete Setup (5 minutes)

### 8.1 Run Type Check

```bash
npm run typecheck
```

Should complete with no errors.

### 8.2 Run Unit Tests

```bash
npm run test:unit
```

Should show passing tests.

### 8.3 Build Production Version

```bash
npm run build
```

Should build successfully to `build/` directory.

### 8.4 Serve Production Build

```bash
npm run serve
```

Visit `http://localhost:3000` and verify site works.

**Success Checkpoint**: All commands run successfully

## Next Steps

You now have a complete Docusaurus development environment ready for implementing the Physical AI & Humanoid Robotics Course Book!

### Recommended Implementation Order

1. **Content First** (User Story 1 - P1):
   - Create all chapter MDX files with frontmatter
   - Add placeholder content for each chapter
   - Verify navigation and routing work

2. **Interactive Components** (User Story 2 - P2):
   - Implement InteractiveTutorial component
   - Implement MultimediaEmbed component
   - Add components to sample chapters

3. **Testing** (TDD Requirement):
   - Write E2E tests for navigation
   - Write accessibility tests
   - Write integration tests for search

4. **Polish** (User Stories 3-5):
   - Optimize search configuration
   - Add multimedia content
   - Implement advanced chapter content

### Helpful Commands Reference

```bash
# Development
npm start                    # Start dev server
npm run typecheck            # Check TypeScript types
npm run lint                 # Check code style
npm run format               # Format code

# Testing
npm run test:unit            # Run Jest unit tests
npm run test:unit:watch      # Run tests in watch mode
npm run test:e2e             # Run Playwright E2E tests
npm run test:a11y            # Run accessibility tests

# Production
npm run build                # Build for production
npm run serve                # Serve production build locally
```

## Troubleshooting

### Port 3000 Already in Use

```bash
# Kill process on port 3000 (Windows)
netstat -ano | findstr :3000
taskkill /PID <PID> /F

# Or use different port
npm start -- --port 3001
```

### TypeScript Errors in node_modules

```bash
npm run typecheck -- --skipLibCheck
```

### Build Fails Due to Broken Links

Check `docusaurus.config.ts`:
```typescript
onBrokenLinks: 'warn',  // Change from 'throw' temporarily
```

## Resources

- [Docusaurus Documentation](https://docusaurus.io/)
- [React Testing Library](https://testing-library.com/react)
- [Playwright Documentation](https://playwright.dev/)
- [TypeScript Handbook](https://www.typescriptlang.org/docs/)

---

**Quickstart Complete!** You're ready to implement the robotics book following the implementation plan and tasks.
