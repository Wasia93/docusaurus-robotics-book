# Data Model: Physical AI & Humanoid Robotics Course Book

**Date**: 2025-12-02
**Feature**: 001-docusaurus-robotics-book
**Purpose**: Define content structure, component interfaces, and data relationships

## Overview

This data model defines the structure of educational content and interactive components for the Docusaurus-based robotics book. Since this is a static site with no backend database, the "data" consists of:
1. **Content Files**: MDX/Markdown files representing book chapters
2. **Component Props**: TypeScript interfaces for React components
3. **Configuration Data**: Sidebar structure, metadata, navigation

## Content Entities

### 1. Chapter (MDX File)

Represents a major content division (e.g., "Bipedal Locomotion").

**File Location**: `docs/part[N]-[category]/[chapter-slug].md`

**Frontmatter Schema**:
```yaml
---
id: chapter-unique-id
title: Chapter Title
sidebar_label: Short Label
sidebar_position: 1
description: Brief chapter description for SEO
keywords: [robotics, ai, specific-topic]
---

# Chapter Content Metadata
learning_objectives:
  - Objective 1
  - Objective 2
estimated_reading_time: 30 minutes
prerequisites:
  - Previous Chapter Link
  - Required Knowledge
difficulty: beginner | intermediate | advanced
```

**Relationships**:
- Belongs to: Part (via directory structure)
- Contains: Sections, Code Examples, Multimedia Assets, Exercises

**Validation Rules**:
- `id` must be unique across all chapters
- `title` required, max 100 characters
- `estimated_reading_time` must be positive integer
- `difficulty` must be one of: beginner, intermediate, advanced

### 2. Part (Directory + Sidebar Category)

Top-level book division (Foundations, Humanoid Robotics, Advanced Topics).

**File Location**: `docs/part[N]-[name]/` directory

**Sidebar Configuration** (sidebars.js):
```javascript
{
  type: 'category',
  label: 'Part 1: Foundations',
  collapsed: false,  // First part expanded by default
  items: [
    'part1-foundations/chapter-id-1',
    'part1-foundations/chapter-id-2',
    // ...
  ],
}
```

**Attributes**:
- `label`: Display name (e.g., "Part 1: Foundations")
- `collapsed`: Boolean - whether category starts collapsed
- `items`: Array of chapter IDs in order

**Contains**: Multiple Chapters

### 3. Section (Markdown Heading within Chapter)

Subdivision within a chapter covering a specific subtopic.

**Representation**: Markdown heading (## or ###) within MDX file

**Structure**:
```markdown
## Section Title {#section-anchor}

Section content with text, code examples, diagrams...

### Subsection Title {#subsection-anchor}

More detailed content...
```

**Attributes**:
- Heading level (H2, H3, H4)
- Anchor ID (for deep linking)
- Content (text, components, media)

**Auto-generated**:
- Table of contents entry
- URL fragment (#section-anchor)

## Component Data Models

### 4. Code Example Component

Interactive code block with syntax highlighting, copy functionality, and educational context.

**TypeScript Interface**:
```typescript
// src/components/CodeExample/types.ts
export interface CodeExampleProps {
  /**
   * Source code to display
   */
  code: string;

  /**
   * Programming language for syntax highlighting
   * Supported: python, javascript, typescript, cpp, bash, yaml
   */
  language: 'python' | 'javascript' | 'typescript' | 'cpp' | 'bash' | 'yaml';

  /**
   * Optional title describing what the code does
   */
  title?: string;

  /**
   * Expected output when code is run
   */
  expectedOutput?: string;

  /**
   * Setup instructions before running code
   */
  setupInstructions?: string;

  /**
   * File path if code represents a complete file
   */
  filePath?: string;

  /**
   * Line numbers to highlight (e.g., "3-5,10")
   */
  highlightLines?: string;

  /**
   * Whether code is runnable standalone
   */
  isRunnable?: boolean;
}
```

**Usage in MDX**:
```mdx
import CodeExample from '@site/src/components/CodeExample';

<CodeExample
  code={`
def inverse_kinematics(target_position, robot_config):
    """Calculate joint angles for target end-effector position."""
    # Implementation details...
    return joint_angles
  `}
  language="python"
  title="Inverse Kinematics Function"
  expectedOutput="Joint angles: [0.52, 1.23, -0.87, 1.45, 0.12, 0.98]"
  setupInstructions="Install numpy: pip install numpy"
  isRunnable={true}
/>
```

**Validation Rules**:
- `code` required, non-empty string
- `language` must be supported language
- If `isRunnable` true, `expectedOutput` should be provided

### 5. Interactive Tutorial Component

Step-by-step guided exercise with instructions and validation.

**TypeScript Interface**:
```typescript
// src/components/InteractiveTutorial/types.ts
export interface TutorialStep {
  /**
   * Unique step identifier
   */
  id: string;

  /**
   * Step number (1, 2, 3...)
   */
  stepNumber: number;

  /**
   * Instruction text for this step
   */
  instruction: string;

  /**
   * Optional code snippet for this step
   */
  code?: string;

  /**
   * Expected outcome after completing step
   */
  expectedOutcome: string;

  /**
   * Optional hints if user struggles
   */
  hints?: string[];

  /**
   * Programming language if code present
   */
  language?: string;
}

export interface InteractiveTutorialProps {
  /**
   * Tutorial title
   */
  title: string;

  /**
   * Brief tutorial description
   */
  description: string;

  /**
   * Difficulty level
   */
  difficulty: 'beginner' | 'intermediate' | 'advanced';

  /**
   * Estimated completion time in minutes
   */
  estimatedTime: number;

  /**
   * Prerequisites knowledge/chapters
   */
  prerequisites?: string[];

  /**
   * Tutorial steps in order
   */
  steps: TutorialStep[];

  /**
   * Learning objectives
   */
  learningObjectives?: string[];
}
```

**Usage in MDX**:
```mdx
import InteractiveTutorial from '@site/src/components/InteractiveTutorial';

<InteractiveTutorial
  title="Building a Simple Robot Controller"
  description="Learn to control a simulated robot using Python"
  difficulty="beginner"
  estimatedTime={45}
  prerequisites={["Robotics Fundamentals", "Python Basics"]}
  steps={[
    {
      id: "step-1",
      stepNumber: 1,
      instruction: "Import required libraries and initialize robot",
      code: "import robot_sim\nrobot = robot_sim.Robot()",
      expectedOutcome: "Robot object created successfully",
      hints: ["Check that robot_sim is installed"]
    },
    // More steps...
  ]}
/>
```

**State Management**:
- Current step index (controlled component)
- Completion status per step
- Hints revealed status
- User progress saved in localStorage (optional)

### 6. Multimedia Embed Component

Embeds videos, images, and diagrams with proper accessibility.

**TypeScript Interface**:
```typescript
// src/components/MultimediaEmbed/types.ts
export type MediaType = 'image' | 'video' | 'diagram';

export interface MultimediaEmbedProps {
  /**
   * Type of media
   */
  type: MediaType;

  /**
   * URL or file path to media
   */
  src: string;

  /**
   * Alternative text for accessibility (required for images)
   */
  alt: string;

  /**
   * Caption displayed below media
   */
  caption?: string;

  /**
   * Width (CSS value: '100%', '600px', etc.)
   */
  width?: string;

  /**
   * Height (CSS value: 'auto', '400px', etc.)
   */
  height?: string;

  /**
   * Whether media can be clicked to enlarge
   */
  zoomable?: boolean;

  /**
   * For videos: transcript URL or text
   */
  transcript?: string;

  /**
   * For diagrams: SVG allows CSS styling
   */
  inlineSvg?: boolean;
}
```

**Usage in MDX**:
```mdx
import MultimediaEmbed from '@site/src/components/MultimediaEmbed';

<MultimediaEmbed
  type="video"
  src="https://youtube.com/embed/robot-walking-demo"
  alt="Bipedal robot walking demonstration"
  caption="Figure 3.1: Humanoid robot performing dynamic walking"
  width="100%"
  height="400px"
  transcript="/transcripts/walking-demo.txt"
/>

<MultimediaEmbed
  type="diagram"
  src="/img/diagrams/sensor-architecture.svg"
  alt="Robot sensor system architecture diagram"
  caption="Figure 2.3: Sensor data pipeline architecture"
  zoomable={true}
  inlineSvg={true}
/>
```

**Validation Rules**:
- `alt` required for accessibility
- `src` must be valid URL or file path
- If `type` is 'video' and not self-hosted, should provide `transcript`

## Configuration Data Models

### 7. Docusaurus Configuration

**File**: `docusaurus.config.js` or `docusaurus.config.ts`

**Key Configuration Sections**:

```typescript
// Simplified type definition
interface DocusaurusConfig {
  title: string;
  tagline: string;
  url: string;
  baseUrl: string;
  onBrokenLinks: 'throw' | 'warn' | 'ignore';
  favicon: string;

  themeConfig: {
    navbar: {
      title: string;
      logo: { alt: string; src: string };
      items: NavbarItem[];
    };
    footer: {
      style: 'dark' | 'light';
      links: FooterLinkCategory[];
      copyright: string;
    };
    prism: {
      theme: PrismTheme;
      darkTheme: PrismTheme;
      additionalLanguages: string[];
    };
    algoliaOrLocalSearch: SearchConfig;
  };

  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: string;
          editUrl?: string;
          showLastUpdateTime: boolean;
        };
        theme: {
          customCss: string;
        };
      }
    ]
  ];

  plugins: Plugin[];
}
```

**Example Configuration Values**:
```javascript
module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Course in Robotics and AI',
  url: 'https://robotics-book.example.com',
  baseUrl: '/',
  onBrokenLinks: 'throw',  // Fail build on broken links
  favicon: 'img/favicon.ico',

  themeConfig: {
    navbar: {
      title: 'Robotics Course',
      items: [
        { type: 'doc', docId: 'intro', position: 'left', label: 'Book' },
        { to: '/exercises', label: 'Exercises', position: 'left' },
        { href: 'https://github.com/org/repo', label: 'GitHub', position: 'right' },
      ],
    },
    prism: {
      additionalLanguages: ['python', 'bash', 'yaml', 'cpp', 'matlab'],
    },
  },
};
```

### 8. Sidebar Configuration

**File**: `sidebars.js`

**Type Definition**:
```typescript
type SidebarItem =
  | string  // Document ID
  | {
      type: 'category';
      label: string;
      collapsed?: boolean;
      items: SidebarItem[];
    }
  | {
      type: 'link';
      label: string;
      href: string;
    };

type Sidebar = {
  [sidebarId: string]: SidebarItem[];
};
```

**Example**:
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
    // Additional parts...
  ],
};
```

## Data Relationships Diagram

```
Part
├── Chapter (MDX file)
│   ├── Frontmatter (metadata)
│   ├── Section (H2/H3 headings)
│   │   ├── Content (text)
│   │   ├── CodeExample (component)
│   │   ├── MultimediaEmbed (component)
│   │   └── InteractiveTutorial (component)
│   └── Section
└── Chapter
    └── ...

Configuration
├── docusaurus.config.js
│   ├── Theme Config
│   ├── Plugin Config
│   └── Preset Config
└── sidebars.js
    └── Sidebar Items (references Chapters by ID)
```

## Validation and Constraints

### Content Validation

1. **Chapter Frontmatter**:
   - All required fields present (id, title, description)
   - No duplicate IDs across chapters
   - Valid difficulty level

2. **Component Props**:
   - TypeScript ensures type safety at compile time
   - Runtime prop validation via PropTypes (if added)

3. **Sidebar Configuration**:
   - All referenced chapter IDs exist
   - No circular references
   - Valid hierarchy (category depth ≤ 3)

4. **Build-Time Checks**:
   - Broken links detected (`onBrokenLinks: 'throw'`)
   - Missing alt text in images (via ESLint plugin)
   - Unused assets detected (via build warnings)

### Performance Constraints

1. **File Sizes**:
   - Individual MDX files: <100KB
   - Images: <500KB each
   - Total static assets: <50MB

2. **Bundle Sizes**:
   - Main bundle: <200KB (gzipped)
   - Per-page bundle: <50KB (gzipped)
   - Code splitting for components >30KB

## State Management

Since this is a static documentation site, state management is minimal:

1. **React Component State**:
   - Local component state (useState) for interactive elements
   - No global state management needed (no Redux/MobX)

2. **Browser Storage** (optional enhancements):
   - localStorage: User progress in tutorials
   - sessionStorage: Current reading position
   - cookies: Theme preference (light/dark mode)

3. **URL State**:
   - Current chapter (from pathname)
   - Section anchor (from hash)
   - Search query (from query params)

## Summary

This data model defines:
- **6 Content Entities**: Chapter, Part, Section, Code Example, Tutorial, Multimedia
- **3 Configuration Entities**: Docusaurus Config, Sidebar Config, Theme Config
- **TypeScript Interfaces**: Full type safety for all components
- **Validation Rules**: Build-time and runtime checks
- **Relationships**: Clear hierarchy and references

All entities support the educational content-first principle from the constitution, with TypeScript providing type safety (clean code principle) and structured validation (TDD-compatible).
