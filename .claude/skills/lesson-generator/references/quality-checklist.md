# Quality Checklist

Run this checklist before delivering any lesson. Every item must pass.

## Frontmatter

- [ ] All required fields present (id, title, sidebar_position, sidebar_label, description, duration_minutes, proficiency_level, layer, hardware_tier, tier_1_path, learning_objectives, keywords, prerequisites, chapter, module)
- [ ] `id` is lowercase, hyphenated, unique
- [ ] `description` is under 160 characters
- [ ] `learning_objectives` each start with a Bloom's taxonomy verb
- [ ] `keywords` has 4-8 entries
- [ ] `tier_1_path` is non-empty
- [ ] `hardware_tier` is an integer 1-4

## Clarity

- [ ] Every sentence understood on first read (read aloud test)
- [ ] No sentence exceeds 30 words
- [ ] No paragraph exceeds 5 sentences
- [ ] All jargon defined on first use (bolded)
- [ ] No banned phrases ("simply", "just", "obviously", "easy", "trivial", "as everyone knows")
- [ ] Active voice throughout

## Structure

- [ ] Learning objectives present and use Bloom's verbs
- [ ] Hook section present (2-3 sentences, real-world connection)
- [ ] Every H2 section is 300-500 tokens
- [ ] H2 headers are descriptive and keyword-rich (not clever)
- [ ] Key Takeaways present (3-5 factual bullet points)
- [ ] Check Your Understanding present (3-5 questions, mix recall and application)
- [ ] Next Steps present (1-2 sentences)
- [ ] Hardware tier note present at bottom

## Code Examples

- [ ] Every code block follows the four-part pattern (context, code, output, explanation)
- [ ] All imports shown (never assumed)
- [ ] Max 40 lines per code block
- [ ] Filename hint in first comment
- [ ] Expected output shown
- [ ] Meaningful variable names (no `x`, `temp`, `foo`)
- [ ] Comments explain "why" not "what"

## Safety (required for motor control / hardware lessons)

- [ ] `:::danger` admonition present for physical safety warnings
- [ ] Emergency stop patterns documented
- [ ] Joint/velocity limits shown in code
- [ ] Simulation-first approach emphasized

## Accessibility

- [ ] Alt text on all images (describes content, not decoration)
- [ ] Tables have clear headers
- [ ] Code blocks have surrounding context
- [ ] No information conveyed by color alone

## RAG Optimization

- [ ] Each H2 section is self-contained (makes sense if retrieved independently)
- [ ] Headers include relevant keywords for semantic search
- [ ] Key Takeaways written as standalone retrievable facts
- [ ] Terminology matches project glossary (no synonyms)
