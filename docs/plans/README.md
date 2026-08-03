# Plans — numbered design records

This folder holds **design plans**: the internal thinking behind a chunk of
work — *how* it was built and, more importantly, *why*. They exist so we (and
AI agents) can recover the rationale behind a subsystem without burning tokens
re-researching the code. Read the relevant plan before changing the area it
covers. Same convention as DovesDataViewer's `docs/plans/` — the number doubles
as a lightweight "ticket number" across both repos' histories.

## Naming — numbered, sequential

Every plan is prefixed with a **zero-padded sequence number** and a short slug:

```
0000-firmware-ota-phase0.md
0001-insta360-ble-current-implementation.md
0002-sprint-mode.md
```

**To add a plan:** take the **next number after the highest one in this
folder** (don't reuse or backfill gaps), pick a short kebab-case slug, and
write `NNNN-slug.md`. The number is permanent — renaming would break
references, so it stays even if the slug later feels dated.

Numbers are per-repo: this folder's `0002` and DovesDataViewer's `0002` are
unrelated. When a piece of work spans repos (like sprint mode), each repo gets
its own plan and they cross-link by full name.

## Keeping plans current

- **Update a plan while you execute it** — as decisions change, the plan
  should reflect what was actually built, not just the original intent.
- **Only revisit an older plan later if you're working in code that references
  it.** Don't sweep through and "refresh" plans speculatively; touch a plan
  when its area is in play.

## Commit messages must cite the plan number

Any commit that is part of executing a plan **must reference the plan number**
in its message — `plan 0002:` as a prefix, or `(plan 0002)` inline. That's
what lets someone reading `git log` jump straight from a change back to the
reasoning behind it.

## What a plan should contain

No rigid template, but a good plan covers:
- **Goal / problem** — what we're solving and why it matters here.
- **Approach & key decisions** — the design chosen and the alternatives
  rejected, with the *why*. This is the most valuable part.
- **Touch points** — the files/modules/subsystems involved.
- **Status / phasing** — what's done, what's pending, any follow-ups.

Plans are referenced from `CLAUDE.md`, `ARCHITECTURE.md`, `CHANGELOG.md`, and
code comments — when you move or renumber one, update those references too
(and keep `CLAUDE.md`'s File Map in sync, per its maintainers note).
