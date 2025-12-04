# ADR 001: RAG Chatbot Architecture

**Status**: Accepted
**Date**: 2025-12-05
**Deciders**: Project Team
**Technical Story**: Integrated RAG Chatbot for Robotics Book

## Context and Problem Statement

The Physical AI & Humanoid Robotics course needed an interactive learning assistant to help students understand complex concepts. Traditional static documentation doesn't provide immediate answers to student questions or context-specific explanations.

**Key Requirements**:
- Answer questions about course content
- Support text-selection-based queries
- Cost-effective operation
- Easy deployment
- Maintainable codebase

## Decision Drivers

- **Student Experience**: Immediate, context-aware answers
- **Cost Constraints**: Must operate on free/low-cost tiers
- **Maintainability**: Clear separation of concerns
- **Scalability**: Handle growing content and user base
- **Integration**: Seamless embedding in Docusaurus

## Considered Options

### Option 1: Custom Open-Source RAG Stack
**Components**: Ollama + ChromaDB + Local deployment
- **Pros**: No API costs, full control, data privacy
- **Cons**: Requires GPU server, maintenance overhead, slower responses

### Option 2: Cloud-Based RAG with OpenAI
**Components**: OpenAI + Qdrant Cloud + Neon Postgres
- **Pros**: High quality, fast, scalable, minimal maintenance
- **Cons**: API costs, external dependencies

### Option 3: Hybrid Approach
**Components**: OpenAI embeddings + Local inference
- **Pros**: Lower inference costs
- **Cons**: Complex setup, quality trade-offs

## Decision Outcome

**Chosen option**: **Option 2 - Cloud-Based RAG with OpenAI**

### Rationale

1. **Quality**: OpenAI's GPT-4o-mini provides superior comprehension and responses
2. **Cost-Effective**: Free tiers + ~$0.50/month for typical usage
3. **Simplicity**: Minimal infrastructure requirements
4. **Speed**: Fast response times improve UX
5. **Scalability**: Automatically scales with demand

### Implementation Details

#### Technology Stack

**Backend: FastAPI**
- **Why**: Modern, fast, automatic API documentation, async support
- **Alternatives Considered**: Flask (too simple), Django (too heavy)
- **Trade-off**: Learning curve vs. features → FastAPI wins

**AI Provider: OpenAI**
- **Embedding Model**: text-embedding-3-small
  - Cost: $0.00002 per 1K tokens
  - Dimension: 1536
  - Quality: Excellent for semantic search
- **Chat Model**: GPT-4o-mini
  - Cost: $0.00015 per 1K input tokens
  - Quality: High comprehension, good explanations
  - Speed: Fast response times

**Vector Database: Qdrant Cloud**
- **Why**: Free tier (1GB), excellent performance, managed service
- **Alternatives Considered**:
  - Pinecone: Better known, but limited free tier
  - Weaviate: Good, but more complex setup
  - ChromaDB: Local-only, deployment complexity
- **Trade-off**: Vendor lock-in vs. ease of use → Qdrant wins

**Database: Neon Serverless Postgres**
- **Why**: Free tier (512MB), serverless, excellent DX
- **Alternatives Considered**:
  - Supabase: Similar, slightly more complex
  - PlanetScale: MySQL, prefer Postgres
  - Railway Postgres: Good, but limited free tier
- **Trade-off**: Storage limits vs. zero maintenance → Neon wins

#### Architecture Patterns

**Service Layer Pattern**
```
Routers → Services → External APIs/DBs
```
- **Benefit**: Clear separation, easy testing, swappable components
- **Cost**: More boilerplate
- **Decision**: Worth it for maintainability

**RAG Pipeline Design**
```
Query → Embed → Search → Retrieve → Augment → Generate → Store
```
- **Chunking Strategy**: 500 words + 50 overlap
  - **Why**: Balance between context and granularity
  - **Alternative**: Semantic chunking (more complex)
- **Top-K Retrieval**: 5 documents
  - **Why**: Sufficient context without overwhelming the model
  - **Configurable**: Can adjust based on performance

**Text Selection Feature**
- **Implementation**: Browser selection API + context passing
- **Why**: Enables precise, context-specific questions
- **Innovation**: Not common in course chatbots
- **UX Impact**: Significantly improves relevance

### Consequences

#### Positive
✅ **High Quality Responses**: GPT-4o-mini provides excellent answers
✅ **Low Cost**: ~$0.50/month for 1000 questions
✅ **Fast Development**: Managed services reduce complexity
✅ **Good UX**: Fast responses, relevant answers
✅ **Scalable**: Automatically handles traffic spikes
✅ **Maintainable**: Clear architecture, good documentation

#### Negative
❌ **External Dependencies**: Reliance on third-party services
❌ **API Costs**: Scales with usage (mitigated by free tiers)
❌ **Data Privacy**: Course content sent to OpenAI (acceptable for public content)
❌ **Vendor Lock-in**: Switching costs for OpenAI/Qdrant

#### Neutral
⚖️ **Internet Required**: Can't work offline
⚖️ **Rate Limits**: Free tiers have limits (sufficient for MVP)
⚖️ **Customization**: Less control than self-hosted

### Risks and Mitigation

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| OpenAI API costs spike | High | Low | Rate limiting, usage monitoring, alerts |
| Qdrant service downtime | Medium | Low | Retry logic, graceful degradation |
| Neon database limits | Low | Medium | Monitor usage, upgrade if needed |
| Poor answer quality | High | Low | Prompt engineering, context tuning |
| CORS issues | Medium | Low | Proper configuration, testing |

## Validation

### Success Criteria
- [x] Backend deploys successfully
- [x] Frontend integrates without breaking builds
- [x] Text selection feature works
- [x] Answers are relevant and accurate
- [x] Cost stays under $1/month
- [x] Response time under 3 seconds

### Performance Metrics
- **Target Response Time**: < 3 seconds
- **Target Accuracy**: > 85% relevant answers
- **Cost Target**: < $1/month (1000 questions)
- **Uptime Target**: > 99%

## Future Considerations

### Short-term (1-3 months)
- [ ] Add caching for common queries
- [ ] Implement rate limiting
- [ ] Add usage analytics
- [ ] Fine-tune chunk sizes based on feedback

### Medium-term (3-6 months)
- [ ] Consider fine-tuning on course content
- [ ] Add authentication for power users
- [ ] Support file uploads (PDFs, code)
- [ ] Multi-modal support (images, diagrams)

### Long-term (6+ months)
- [ ] Evaluate switching to open-source models (if costs grow)
- [ ] Consider self-hosted deployment (if scale justifies)
- [ ] Add voice interface
- [ ] Personalized learning paths

## Related Decisions

- **ADR-002**: Deployment strategy (Railway vs Render vs Docker)
- **ADR-003**: Authentication and rate limiting approach
- **ADR-004**: Multi-language support for chatbot

## References

- [RAG Best Practices](https://www.pinecone.io/learn/retrieval-augmented-generation/)
- [OpenAI Embeddings Guide](https://platform.openai.com/docs/guides/embeddings)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [FastAPI Best Practices](https://fastapi.tiangolo.com/tutorial/)

## Notes

This ADR represents a significant architectural decision that impacts the entire learning experience. The choice of cloud-based RAG over self-hosted solutions prioritizes developer experience and student experience over cost optimization, which is appropriate for an MVP/early-stage product.

The text selection feature is a key differentiator that wasn't in the original requirements but adds significant value. This should be highlighted in marketing materials.

---

**Review Date**: 2025-03-05 (3 months)
**Reviewers**: Project Team, Early Users
**Approval**: Accepted by project stakeholders
