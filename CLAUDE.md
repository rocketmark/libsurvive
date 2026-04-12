## Linked-Intent Development (MANDATORY)

**Consult the `linked-intent-dev` skill for ALL code changes.** All changes start with intent:

```
HLD → LLDs → EARS → Tests → Code
```

- **New features**: Full workflow (HLD → LLD → EARS → Plan)
- **Bug fixes**: Coherence check only (verify existing specs/tests/code align)
- **If unsure**: Use the full workflow

Mutation, not accumulation — docs reflect current intent, not history.

### Navigation

| What you need | Where to look |
|---|---|
| High-level design | `docs/high-level-design.md` |
| Low-level designs | `docs/llds/` |
| EARS specs | `docs/specs/` |
| Implementation plans | `docs/planning/` |

### Terminology

- **LLD**: Low-level design — detailed component design docs in `docs/llds/`
- **EARS**: Easy Approach to Requirements Syntax — structured requirements in `docs/specs/`. Markers: `[x]` implemented, `[ ]` active gap, `[D]` deferred

### Code Annotations

Annotate code with `@spec` comments linking to EARS IDs:

```
// @spec AUTH-UI-001, AUTH-UI-002
```

Test files also reference specs for traceability.
