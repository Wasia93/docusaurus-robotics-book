# Project History

This directory maintains the complete history of architectural decisions, prompt exchanges, and feature implementations for the Physical AI & Humanoid Robotics course project.

## Directory Structure

```
history/
├── prompts/              # Prompt History Records (PHRs)
│   ├── rag-chatbot/     # RAG Chatbot feature implementation
│   ├── constitution/     # Constitution-related exchanges
│   └── general/         # General feature discussions
├── adr/                 # Architecture Decision Records
│   └── 001-*.md        # ADRs numbered sequentially
└── README.md           # This file
```

## Prompt History Records (PHRs)

PHRs document AI-assisted development exchanges for learning and traceability. Each PHR captures:

- **Context**: What prompted the exchange
- **Requirements**: What was requested
- **Approach**: How it was implemented
- **Decisions**: Key choices made
- **Learnings**: Insights gained
- **Follow-ups**: Next steps

### PHR Naming Convention
```
<number>-<brief-description>.md

Examples:
001-initial-implementation.md
002-feature-decisions.md
003-deployment-strategy.md
```

## Architecture Decision Records (ADRs)

ADRs document significant architectural decisions that have long-term impact. Each ADR includes:

- **Status**: Proposed/Accepted/Deprecated/Superseded
- **Context**: Problem being solved
- **Decision**: What was decided
- **Consequences**: Positive and negative outcomes
- **Alternatives**: Other options considered

### When to Create an ADR

Create an ADR when a decision meets ALL criteria:
- **Impact**: Long-term consequences (framework, data model, API, security, platform)
- **Alternatives**: Multiple viable options with trade-offs
- **Scope**: Cross-cutting, influences system design

### ADR Template

Use this structure:
```markdown
# ADR NNN: Title

**Status**: [Proposed | Accepted | Deprecated | Superseded]
**Date**: YYYY-MM-DD
**Deciders**: [List of decision makers]

## Context and Problem Statement
## Decision Drivers
## Considered Options
## Decision Outcome
## Consequences
## Validation
## References
```

## Current Features Documented

### RAG Chatbot (2025-12-05)
- **PHR**: `prompts/rag-chatbot/001-initial-implementation.md`
- **ADR**: `adr/001-rag-chatbot-architecture.md`
- **Status**: ✅ Completed
- **Key Decisions**: Cloud-based stack, text selection feature, FastAPI architecture

## Contributing to History

### Adding a PHR

1. Determine the appropriate subdirectory:
   - `rag-chatbot/` - RAG chatbot feature
   - `constitution/` - Project principles
   - `general/` - Other features

2. Create numbered file:
   ```bash
   history/prompts/<feature>/<next-number>-<description>.md
   ```

3. Use PHR template (see examples in existing files)

### Adding an ADR

1. Get next number:
   ```bash
   ls history/adr/ | tail -1
   ```

2. Create ADR file:
   ```bash
   history/adr/<number>-<brief-title>.md
   ```

3. Use ADR template above

4. Link from related PHRs

## History Statistics

- **Total PHRs**: 3
- **Total ADRs**: 1
- **Features Documented**: 1 (RAG Chatbot)
- **Last Updated**: 2025-12-05

## Benefits of Maintaining History

### For Current Development
- ✅ Understand why decisions were made
- ✅ Avoid revisiting settled issues
- ✅ Onboard new team members faster
- ✅ Learn from past successes/failures

### For Future Maintenance
- ✅ Evaluate if decisions still apply
- ✅ Understand context for changes
- ✅ See evolution of architecture
- ✅ Identify patterns across features

### For Learning
- ✅ Document AI-assisted workflows
- ✅ Share prompting strategies
- ✅ Build institutional knowledge
- ✅ Improve future implementations

## Review Schedule

### PHRs
- Review after feature completion
- Update with lessons learned
- Add follow-up tasks

### ADRs
- Review quarterly (or when revisiting decision)
- Mark as Superseded if replaced
- Update consequences based on experience

## Related Documentation

- `.specify/memory/constitution.md` - Project principles
- `specs/<feature>/spec.md` - Feature specifications
- `specs/<feature>/plan.md` - Implementation plans
- `CHATBOT_SETUP.md` - RAG Chatbot setup guide

---

**Maintained By**: Project Team
**Last Review**: 2025-12-05
**Next Review**: 2025-03-05
