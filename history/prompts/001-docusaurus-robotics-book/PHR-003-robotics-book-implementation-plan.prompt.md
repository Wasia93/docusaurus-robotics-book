---
id: PHR-003
title: Robotics Book Implementation Plan
stage: plan
date: 2025-12-02
surface: claude-code
model: claude-sonnet-4-5
feature: 001-docusaurus-robotics-book
branch: 001-docusaurus-robotics-book
user: B.COM
command: /sp.plan
labels: [plan, architecture, docusaurus, testing, tdd]
links:
  spec: specs/001-docusaurus-robotics-book/spec.md
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: specs/001-docusaurus-robotics-book/plan.md
    action: updated
    lines: 245
  - path: specs/001-docusaurus-robotics-book/research.md
    action: created
    lines: 450
  - path: specs/001-docusaurus-robotics-book/data-model.md
    action: created
    lines: 400
  - path: specs/001-docusaurus-robotics-book/contracts/content-schema.yaml
    action: created
    lines: 350
  - path: specs/001-docusaurus-robotics-book/quickstart.md
    action: created
    lines: 650
tests:
  - N/A (planning phase)
---

## Prompt

Create implementation plan for Physical AI & Humanoid Robotics Course Book using Docusaurus v3.x. User guidance:
- Start with basic Docusaurus setup and configuration
- Create folder structure for book chapters
- Implement core navigation and sidebar
- Build chapter templates with MDX
- Add interactive code examples
- Set up testing infrastructure
- Create deployment configuration
- Document setup and contribution guidelines

Execute full planning workflow: Technical Context, Constitution Check, Phase 0 Research, Phase 1 Design (data model, contracts, quickstart).

## Response snapshot

### Planning Artifacts Created

**plan.md (245 lines)**:
- **Technical Context**: Docusaurus v3.x, React 18+, TypeScript 5.x, Jest 29+, Playwright, Node.js 18+
- **Performance Goals**: <2s page load, <1s search, Lighthouse >90
- **Constitution Check**: All 7 principles PASS (Educational Content First, Code as Teaching Tool, TDD, Integration Testing, Practical Examples, Observability, Simplicity)
- **Project Structure**: Web application with `/docs` (12 chapters in 3 parts), `/src/components` (CodeExample, InteractiveTutorial, MultimediaEmbed), `/tests` (unit/integration/e2e)
- **Post-Phase 1 Re-evaluation**: Design artifacts confirm full constitution compliance

**research.md (450 lines)**:
- **Framework**: Docusaurus v3.x with Classic theme (mature, educational-focused)
- **Search**: docusaurus-search-local (no external dependencies, sufficient for scope)
- **Content Format**: MDX v2 with React components for interactivity
- **Testing**: Multi-layer (Jest for unit, React Testing Library for components, Playwright for E2E)
- **TypeScript**: Full support with strict mode for type safety
- **CI/CD**: GitHub Actions + Vercel (preview deployments, automatic builds)
- **Accessibility**: WCAG 2.1 Level AA with axe-core automated testing
- **Package Management**: npm with lock file, standard Node.js ecosystem

**data-model.md (400 lines)**:
- **6 Content Entities**: Chapter (MDX frontmatter), Part (sidebar category), Section (H2/H3), Code Example, Interactive Tutorial, Multimedia Embed
- **TypeScript Interfaces**: Full type definitions for all component props
- **Chapter Frontmatter Schema**: id, title, learning_objectives, estimated_reading_time, prerequisites, difficulty, description
- **Component Props**: CodeExample (code, language, expectedOutput, setupInstructions), InteractiveTutorial (steps, difficulty, learningObjectives), MultimediaEmbed (type, src, alt, transcript)
- **Validation Rules**: Build-time checks, accessibility requirements, performance constraints

**contracts/content-schema.yaml (350 lines)**:
- **Chapter Frontmatter Schema**: Required fields, regex patterns, min/max constraints
- **Component Prop Schemas**: Type definitions, enums, validation rules for all components
- **Sidebar Configuration Schema**: Category/link/doc types, hierarchy rules
- **Docusaurus Config Schema**: Site metadata, theme config, plugin config
- **Validation Rules Summary**: Content, components, configuration, performance, accessibility

**quickstart.md (650 lines)**:
- **8-Step Setup Guide**: Prerequisites → Initialize Docusaurus → Install dependencies → Configure structure → Configure Docusaurus → Create sample chapter → Implement components → Verify setup
- **Complete Configuration**: tsconfig.json, jest.config.js, playwright.config.ts, docusaurus.config.ts, sidebars.ts
- **Component Examples**: Full CodeExample component with TypeScript, styles, tests
- **npm Scripts**: Development, testing, production commands
- **Troubleshooting**: Common issues and solutions
- **Estimated Time**: 60-90 minutes for complete setup

### Technical Decisions

| Area | Decision | Rationale |
|------|----------|-----------|
| Framework | Docusaurus v3.x | Modern, educational-focused, excellent DX, React 18 support |
| Search | docusaurus-search-local | No external dependencies, works offline, sufficient for 12 chapters |
| Content | MDX v2 + React Components | Interactive while maintaining content authoring simplicity |
| Testing | Jest + React Testing Library + Playwright | Multi-layer testing aligns with TDD constitution requirement |
| TypeScript | Full support with strict mode | Type safety for clean code, catches errors at compile time |
| CI/CD | GitHub Actions + Vercel | Best-in-class automation, preview deployments, CDN hosting |
| Accessibility | WCAG 2.1 AA with axe-core | Constitution requirement, legal compliance for education |

### Constitution Alignment Verification

**Pre-Research**: All 7 principles passed initial check
**Post-Design**: All 7 principles confirmed in implementation artifacts

Key alignments:
- **TDD**: Jest/Playwright configured with 70% coverage threshold, test files created before components in quickstart
- **Educational Content First**: Chapter frontmatter includes learning_objectives, prerequisites, estimated_reading_time
- **Code as Teaching Tool**: CodeExample component has expectedOutput, setupInstructions, highlighting
- **Observability**: Components support debugging (output display, alt text, captions)
- **Simplicity**: Incremental 8-step quickstart, flat project structure, client-side only

## Outcome

- ✅ Impact: Complete architectural plan with 5 design artifacts ready for implementation
- 🧪 Tests: Test infrastructure fully designed (Jest, Playwright, axe-core configurations)
- 📁 Files: plan.md, research.md, data-model.md, content-schema.yaml, quickstart.md (2,095 total lines)
- 🔁 Next prompts: /sp.tasks to generate dependency-ordered implementation tasks following TDD workflow
- 🧠 Reflection: Plan balances comprehensive design (450 lines research, 400 lines data model) with actionable quickstart (650 lines, 60-90 min setup). All decisions traced to constitution principles. TypeScript provides type safety for clean code. Multi-layer testing (unit/integration/e2e) ensures TDD compliance. Static site approach (no backend) maintains simplicity. Quickstart demonstrates TDD workflow: test configuration before implementation, component test files shown alongside component code.

## Evaluation notes (flywheel)

- Failure modes observed: Initial research agent response exceeded token limit (32k), resolved by creating research.md directly
- Graders run and results (PASS/FAIL): Constitution Check - ALL 7 PRINCIPLES PASS (pre and post-design)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A

## Architectural Decision Candidates

Based on planning artifacts, the following decisions meet ADR significance criteria (long-term impact, multiple alternatives, cross-cutting scope):

1. **Framework Selection: Docusaurus v3.x vs VitePress vs Custom**
   - Decision: Docusaurus v3.x
   - Impact: Affects entire site architecture, plugin ecosystem, performance characteristics
   - Alternatives: VitePress (newer, Vue-based), Custom framework (maximum control)
   - Cross-cutting: Impacts content authoring, search, navigation, deployment

2. **Testing Strategy: Multi-Layer vs Single Framework**
   - Decision: Jest (unit) + React Testing Library (component) + Playwright (E2E)
   - Impact: Long-term testing approach, CI/CD pipeline, developer workflow
   - Alternatives: Single framework (Playwright only, Cypress only)
   - Cross-cutting: Affects all components, content validation, accessibility compliance

3. **Content Format: MDX + React Components vs Pure Markdown**
   - Decision: MDX v2 with custom React components
   - Impact: Content authoring complexity vs interactivity capabilities
   - Alternatives: Pure Markdown (simpler but less interactive), JSX-in-JS (more complex authoring)
   - Cross-cutting: Affects all chapters, component reusability, content maintainability

**Suggestion**: Document these architectural decisions? Run `/sp.adr docusaurus-framework-selection` or `/sp.adr testing-strategy-multilayer` to create ADRs capturing rationale and tradeoffs for future reference.
