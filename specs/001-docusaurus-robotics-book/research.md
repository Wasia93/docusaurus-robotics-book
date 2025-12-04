# Research: Docusaurus Best Practices for Educational Robotics Book

**Date**: 2025-12-02
**Feature**: 001-docusaurus-robotics-book
**Purpose**: Document technical decisions and best practices for implementing the Physical AI & Humanoid Robotics Course Book

## 1. Docusaurus Configuration

### Decision: Use Docusaurus v3.x with Classic Theme

**Rationale**:
- Docusaurus v3 provides modern React 18 support, improved performance, and better TypeScript integration
- Classic theme offers out-of-the-box features perfect for educational content: navbar, sidebar, search, dark mode
- Active community support and extensive plugin ecosystem

**Alternatives Considered**:
- Custom documentation framework (rejected: reinventing the wheel, more maintenance)
- VitePress (rejected: less mature plugin ecosystem for educational features)
- GitBook (rejected: less customizable, vendor lock-in concerns)

**Implementation Notes**:
- Use `@docusaurus/preset-classic` for standard configuration
- Enable MDX v2 support via `@docusaurus/mdx-loader`
- Configure `docusaurus.config.js` with TypeScript support using JSDoc types or rename to `.ts`

### Decision: Use Local Search Plugin (docusaurus-search-local)

**Rationale**:
- No external dependencies or API keys required (Algolia requires application)
- Works offline and in local development
- Sufficient for ~12 chapters with ~50-100 sections
- Free and open source

**Alternatives Considered**:
- Algolia DocSearch (rejected: requires application/approval, external dependency)
- Custom search implementation (rejected: unnecessary complexity)

**Implementation Notes**:
- Install `@easyops-cn/docusaurus-search-local`
- Configure in plugins array with indexDocs: true, indexBlog: false
- Index size should be <5MB for target content scope

## 2. MDX and Interactive Content

### Decision: MDX v2 with Custom React Components for Code Examples

**Rationale**:
- MDX allows embedding React components directly in Markdown for interactive content
- Preserves content authoring simplicity while enabling rich interactivity
- Supports versioning and git-based workflows

**Alternatives Considered**:
- Pure Markdown with HTML (rejected: limited interactivity)
- Separate component files (rejected: breaks content flow)

**Implementation Pattern**:
```tsx
// src/components/CodeExample/CodeExample.tsx
import React, { useState } from 'react';
import CodeBlock from '@theme/CodeBlock';

interface CodeExampleProps {
  code: string;
  language: string;
  title?: string;
  expectedOutput?: string;
  setupInstructions?: string;
}

export default function CodeExample({code, language, title, expectedOutput, setupInstructions}: CodeExampleProps) {
  const [copied, setCopied] = useState(false);

  return (
    <div className="code-example">
      {title && <h4>{title}</h4>}
      {setupInstructions && <div className="setup-instructions">{setupInstructions}</div>}
      <CodeBlock language={language}>
        {code}
      </CodeBlock>
      {expectedOutput && (
        <div className="expected-output">
          <strong>Expected Output:</strong>
          <pre>{expectedOutput}</pre>
        </div>
      )}
    </div>
  );
}
```

### Decision: Use Prism React Renderer for Syntax Highlighting

**Rationale**:
- Built-in Docusaurus support via `prism-react-renderer`
- Supports Python (primary language for robotics examples)
- Themeable (light/dark mode support)
- Copy-to-clipboard functionality available

**Implementation Notes**:
- Configure supported languages in `docusaurus.config.js` under `themeConfig.prism.additionalLanguages`
- Add Python, YAML, Bash, C++, MATLAB for robotics examples
- Enable line numbering and highlighting via code block meta strings

## 3. Testing Strategy

### Decision: Multi-Layer Testing with Jest, React Testing Library, and Playwright

**Rationale**:
- Jest for unit tests (fast, isolated component testing)
- React Testing Library for component integration (user-centric testing)
- Playwright for E2E (cross-browser, accessibility, navigation flows)
- Aligns with TDD constitution requirement

**Test Configuration**:

**jest.config.js**:
```javascript
module.exports = {
  preset: '@docusaurus/core/lib/utils/jest/preset',
  testEnvironment: 'jsdom',
  moduleNameMapper: {
    '^@site/(.*)$': '<rootDir>/$1',
    '^@theme/(.*)$': '<rootDir>/node_modules/@docusaurus/theme-classic/src/theme/$1',
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
};
```

**playwright.config.ts**:
```typescript
import { defineConfig, devices } from '@playwright/test';

export default defineConfig({
  testDir: './tests/e2e',
  fullyParallel: true,
  forbidOnly: !!process.env.CI,
  retries: process.env.CI ? 2 : 0,
  workers: process.env.CI ? 1 : undefined,
  use: {
    baseURL: 'http://localhost:3000',
    trace: 'on-first-retry',
  },
  projects: [
    { name: 'chromium', use: { ...devices['Desktop Chrome'] } },
    { name: 'firefox', use: { ...devices['Desktop Firefox'] } },
    { name: 'webkit', use: { ...devices['Desktop Safari'] } },
    { name: 'mobile', use: { ...devices['iPhone 13'] } },
  ],
  webServer: {
    command: 'npm run serve',
    url: 'http://localhost:3000',
    reuseExistingServer: !process.env.CI,
  },
});
```

### Decision: Use axe-core for Accessibility Testing

**Rationale**:
- Industry standard for WCAG compliance checking
- Integrates with both Jest (via jest-axe) and Playwright (via @axe-core/playwright)
- Comprehensive rule set for WCAG 2.1 Level AA

**Implementation Notes**:
- Install `jest-axe` for component-level accessibility tests
- Install `@axe-core/playwright` for E2E accessibility validation
- Test keyboard navigation, screen reader compatibility, color contrast

## 4. Project Structure and Content Organization

### Decision: Flat Content Structure with Part-Based Grouping

**Rationale**:
- Clear hierarchy: docs/part1-foundations/, docs/part2-humanoid/, docs/part3-advanced/
- Easy to navigate in file system
- Aligns with book structure (3 parts, 12 chapters)
- Supports independent chapter development

**Sidebar Configuration Pattern** (sidebars.js):
```javascript
module.exports = {
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
```

### Decision: Static Assets in /static with Organized Subdirectories

**Rationale**:
- Docusaurus convention: `/static` maps to root URL in build
- Organized by type: `/static/img/diagrams/`, `/static/videos/`
- Easy to reference in MDX: `![Diagram](/img/diagrams/sensor-architecture.png)`

**Implementation Notes**:
- Keep images optimized (<500KB per image)
- Use WebP format for photos, SVG for diagrams where possible
- External video hosting (YouTube embed) to reduce bundle size

## 5. TypeScript Integration

### Decision: Full TypeScript Support with Strict Mode

**Rationale**:
- Type safety catches errors during development
- Better IDE support and autocomplete
- Aligns with "clean code" constitution requirement
- Docusaurus v3 has excellent TypeScript support

**tsconfig.json Configuration**:
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

**Component Typing Pattern**:
```typescript
import React from 'react';

export interface InteractiveTutorialProps {
  title: string;
  steps: TutorialStep[];
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  estimatedTime: number; // minutes
  prerequisites?: string[];
}

interface TutorialStep {
  id: string;
  instruction: string;
  code?: string;
  expectedOutcome: string;
  hints?: string[];
}

export default function InteractiveTutorial({
  title,
  steps,
  difficulty,
  estimatedTime,
  prerequisites = []
}: InteractiveTutorialProps): JSX.Element {
  // Implementation
}
```

## 6. CI/CD and Deployment

### Decision: GitHub Actions with Vercel Deployment

**Rationale**:
- GitHub Actions free for public repos, integrates seamlessly with GitHub
- Vercel offers best-in-class static site hosting with CDN
- Automatic preview deployments for PRs
- Zero configuration deployment for Docusaurus

**Alternatives Considered**:
- Netlify (similar features, chose Vercel for better DX and speed)
- GitHub Pages (rejected: slower builds, no automatic preview deployments)
- Self-hosted (rejected: unnecessary operational overhead for static site)

**GitHub Actions Workflow** (.github/workflows/ci.yml):
```yaml
name: CI/CD Pipeline

on:
  push:
    branches: [master, '00*-*']
  pull_request:
    branches: [master]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: '18'
          cache: 'npm'

      - name: Install dependencies
        run: npm ci

      - name: Type check
        run: npm run typecheck

      - name: Lint
        run: npm run lint

      - name: Unit tests
        run: npm run test:unit

      - name: Build
        run: npm run build

      - name: E2E tests
        run: npm run test:e2e

      - name: Accessibility tests
        run: npm run test:a11y

  deploy:
    needs: test
    if: github.ref == 'refs/heads/master'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: '18'
      - run: npm ci
      - run: npm run build
      - uses: amondnet/vercel-action@v25
        with:
          vercel-token: ${{ secrets.VERCEL_TOKEN }}
          vercel-org-id: ${{ secrets.VERCEL_ORG_ID }}
          vercel-project-id: ${{ secrets.VERCEL_PROJECT_ID }}
          vercel-args: '--prod'
```

**Build Optimization**:
- Enable Docusaurus production build optimizations (minification, tree shaking)
- Use `swizzle` sparingly (only when necessary to customize theme components)
- Implement code splitting for larger components
- Configure caching in CI (cache node_modules, .docusaurus directory)

### Decision: Preview Deployments on Pull Requests

**Rationale**:
- Enable stakeholder review before merge (instructors, subject matter experts)
- Test content changes in production-like environment
- Catch accessibility and responsive design issues early

**Implementation Notes**:
- Vercel automatically creates preview URLs for PRs
- Add GitHub Action comment with preview URL
- Configure branch protection to require passing CI before merge

## 7. Package Management and Dependencies

### Decision: Use npm with package-lock.json

**Rationale**:
- Standard Node.js package manager, widely supported
- Lock file ensures consistent installs across environments
- npm workspaces support if project grows

**Alternatives Considered**:
- pnpm (rejected: added complexity for single project)
- Yarn (rejected: npm sufficient for this use case)

**Core Dependencies** (package.json):
```json
{
  "dependencies": {
    "@docusaurus/core": "^3.0.0",
    "@docusaurus/preset-classic": "^3.0.0",
    "@easyops-cn/docusaurus-search-local": "^0.40.0",
    "react": "^18.2.0",
    "react-dom": "^18.2.0",
    "prism-react-renderer": "^2.1.0"
  },
  "devDependencies": {
    "@docusaurus/module-type-aliases": "^3.0.0",
    "@docusaurus/tsconfig": "^3.0.0",
    "@docusaurus/types": "^3.0.0",
    "@playwright/test": "^1.40.0",
    "@testing-library/react": "^14.1.0",
    "@testing-library/jest-dom": "^6.1.0",
    "@types/react": "^18.2.0",
    "@types/node": "^20.10.0",
    "jest": "^29.7.0",
    "jest-environment-jsdom": "^29.7.0",
    "jest-axe": "^8.0.0",
    "@axe-core/playwright": "^4.8.0",
    "typescript": "^5.3.0",
    "eslint": "^8.55.0",
    "prettier": "^3.1.0"
  },
  "scripts": {
    "docusaurus": "docusaurus",
    "start": "docusaurus start",
    "build": "docusaurus build",
    "serve": "docusaurus serve",
    "typecheck": "tsc --noEmit",
    "lint": "eslint src --ext .ts,.tsx",
    "format": "prettier --write \"**/*.{ts,tsx,md,mdx}\"",
    "test:unit": "jest",
    "test:e2e": "playwright test",
    "test:a11y": "playwright test tests/e2e/accessibility.spec.ts"
  }
}
```

## 8. Accessibility Implementation

### Decision: WCAG 2.1 Level AA Compliance with Automated and Manual Testing

**Rationale**:
- Constitution requirement and spec success criteria (SC-007)
- Legal compliance for educational institutions
- Better user experience for all learners

**Implementation Strategy**:
1. Use semantic HTML in MDX content
2. Provide alt text for all images
3. Ensure keyboard navigation works for all interactive elements
4. Maintain sufficient color contrast (4.5:1 for normal text)
5. Provide captions/transcripts for videos
6. Test with screen readers (NVDA, JAWS, VoiceOver)

**Automated Testing**:
```typescript
// tests/e2e/accessibility.spec.ts
import { test, expect } from '@playwright/test';
import { injectAxe, checkA11y } from '@axe-core/playwright';

test.describe('Accessibility Tests', () => {
  test('homepage meets WCAG 2.1 Level AA', async ({ page }) => {
    await page.goto('/');
    await injectAxe(page);
    await checkA11y(page, null, {
      detailedReport: true,
      detailedReportOptions: { html: true }
    });
  });

  test('keyboard navigation works', async ({ page }) => {
    await page.goto('/');
    await page.keyboard.press('Tab');
    const focused = await page.locator(':focus');
    await expect(focused).toBeVisible();
  });
});
```

## Summary of Key Decisions

| Area | Decision | Rationale |
|------|----------|-----------|
| Framework | Docusaurus v3.x | Modern, educational-focused, excellent DX |
| Search | docusaurus-search-local | No external dependencies, sufficient for scope |
| Content Format | MDX v2 + React Components | Interactive while maintaining content simplicity |
| Testing | Jest + React Testing Library + Playwright | Multi-layer testing aligns with TDD |
| TypeScript | Full support with strict mode | Type safety, better DX, constitution alignment |
| CI/CD | GitHub Actions + Vercel | Best-in-class automation and hosting |
| Accessibility | WCAG 2.1 AA with axe-core | Constitution requirement, legal compliance |
| Package Manager | npm | Standard, sufficient for project scope |

All decisions support the constitution principles: educational content first, code as teaching tool, TDD mandatory, observability, and simplicity.
