<!--
Sync Impact Report:
- Version change: [INITIAL] → 1.0.0
- New constitution initialized for Physical AI & Humanoid Robotics Teaching Book project
- Added sections: Core Principles (7 principles), Documentation Standards, Development Workflow, Governance
- Templates requiring updates: ✅ All templates reviewed and align with principles
- Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Teaching Book Constitution

## Core Principles

### I. Educational Content First

Educational materials MUST be structured as standalone, comprehensible modules. Each chapter,
section, or code example MUST be independently understandable and testable. Content MUST
progress from foundational concepts to advanced topics in a logical learning path.

**Rationale**: Students learn best when content builds systematically. Modular structure enables
instructors to adapt material for different course formats and allows learners to navigate at
their own pace.

### II. Code as Teaching Tool

Every code example MUST demonstrate a clear concept, be fully runnable, and include inline
documentation explaining the "why" not just the "what". Code MUST be production-quality but
optimized for readability and pedagogical clarity over cleverness.

**Rationale**: Students learn programming best through working examples. Clean, well-documented
code teaches best practices while illustrating robotics and AI concepts. Runnable examples
enable hands-on experimentation.

### III. Test-Driven Development (NON-NEGOTIABLE)

TDD is MANDATORY for all code examples and supporting libraries. Tests MUST be written before
implementation. The Red-Green-Refactor cycle MUST be strictly followed. Every code example
MUST include corresponding test cases that validate both correctness and teach testing practices.

**Rationale**: TDD ensures code quality and reliability, critical for robotics systems. More
importantly, teaching TDD to students instills professional software engineering discipline from
the start. Tests serve dual purpose: validation and teaching material.

### IV. Integration Testing for Robotics Systems

Integration tests MUST cover: hardware-software interfaces, sensor data pipelines, actuator
control loops, multi-component systems, and safety-critical operations. Mock hardware
interfaces MUST be provided where physical hardware is unavailable.

**Rationale**: Robotics and physical AI inherently involve complex system integration. Students
must learn to test beyond unit level, understanding how components interact. Safety-critical
nature of physical systems demands comprehensive integration testing.

### V. Practical Examples with Real-World Context

Examples MUST be grounded in realistic robotics scenarios (manipulation, locomotion, perception,
human-robot interaction). Each example MUST explain real-world considerations: sensor noise,
latency, safety constraints, failure modes. Theory MUST connect to practice.

**Rationale**: Students need to understand the gap between textbook theory and real-world
implementation. Robotics education requires bridging simulation and physical reality, preparing
students for challenges in actual systems.

### VI. Observability and Debugging

All code examples MUST include proper logging, visualization of robot state, sensor data
visualization, and debugging tools. Educational content MUST teach systematic debugging
approaches for robotics systems.

**Rationale**: Debugging robotics systems is fundamentally different from traditional software.
Students must learn to diagnose issues involving physical hardware, sensor data, timing, and
safety. Observability is not optional—it's essential for learning and development.

### VII. Simplicity and Incremental Complexity

Start with simplest viable implementation. Add complexity only when pedagogically justified.
Each concept MUST be introduced in isolation before combining. YAGNI (You Aren't Gonna Need It)
principles apply—avoid premature abstractions that obscure learning.

**Rationale**: Cognitive load management is critical in education. Students must master
fundamentals before tackling complex systems. Incremental complexity enables students to
understand each layer thoroughly.

## Documentation Standards

### Content Organization

- Each chapter MUST have: learning objectives, prerequisites, estimated time, hands-on exercises
- Code examples MUST include: purpose statement, setup instructions, expected output, common issues
- Diagrams MUST accompany complex concepts (system architecture, control flow, data flow)
- API documentation MUST be complete, with examples for all public interfaces

### Accessibility

- Content MUST be accessible to learners with diverse backgrounds
- Prerequisites MUST be clearly stated for each section
- Supplementary materials MUST be provided for advanced topics
- Code examples MUST run on commonly available hardware/software platforms

### Version Control

- All content versions MUST be tagged with semantic versioning
- Breaking changes in code APIs MUST be documented with migration guides
- Deprecated examples MUST be marked clearly with replacement recommendations

## Development Workflow

### Content Development Cycle

1. **Specification**: Define learning objectives and outcomes for content module
2. **Test Design**: Create validation exercises and test cases students will use
3. **Content Draft**: Write explanatory content and code examples
4. **Code Implementation**: Implement examples following TDD principles
5. **Technical Review**: Verify technical accuracy and code quality
6. **Pedagogical Review**: Verify clarity, appropriate difficulty, learning effectiveness
7. **Integration**: Ensure consistency with surrounding content

### Quality Gates

- **Technical Gate**: All code MUST pass tests, linting, type checking
- **Pedagogical Gate**: Content MUST meet learning objectives, appropriate difficulty
- **Accessibility Gate**: Prerequisites clear, concepts well-explained
- **Safety Gate**: Robotics examples MUST include safety considerations
- **Completeness Gate**: Exercises, solutions, and assessment materials included

### Review Process

- All content changes MUST be peer-reviewed by subject matter expert
- Code examples MUST be reviewed by software engineer for best practices
- Pedagogical approach MUST be reviewed by educator
- Technical accuracy MUST be verified through testing and validation

## Governance

This constitution supersedes all other development practices. Amendments require:

1. Documented rationale explaining need for change
2. Review by project maintainers
3. Approval from educational and technical leads
4. Migration plan for affected content

All content contributions MUST verify compliance with these principles. Complexity beyond
these principles MUST be explicitly justified with pedagogical rationale.

Consult CLAUDE.md for runtime development guidance specific to workflow commands and
project management.

**Version**: 1.0.0 | **Ratified**: 2025-12-02 | **Last Amended**: 2025-12-02
