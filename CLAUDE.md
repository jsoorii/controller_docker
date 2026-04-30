# Claude Code Rules

## File Exclusion: `kr_` prefix

Files whose names begin with `kr_` must be **completely ignored** in all operations unless the user explicitly references the exact file name in their prompt.

**What "ignored" means:**
- Do not read, search, or open these files unless explicitly asked
- Do not include them in directory listings, project analyses, or summaries
- Do not suggest, reference, or mention them during exploration or refactoring
- Do not apply changes to them as a side effect of working on other files

**Exception — explicit call only:**
The rule is lifted only when the user's prompt contains the exact file name (e.g., `kr_something.py`). In that case, treat the file normally for that request only.

---

## Pre-Execution Token Check

For any operation estimated to consume **1,000 or more tokens**, follow this procedure before executing:

1. **Check available tokens** — determine how many tokens remain in the current context.
2. **Check feasibility within 70%** — assess whether the operation can be completed within 70% of the remaining tokens.
3. **Decision:**
   - If it **can** be completed within 70%: proceed with execution.
   - If it **cannot**: do NOT execute. Instead, create a `RESUME.md` file and record the current progress.

**Requirements for `RESUME.md`:**
- Must be self-contained: reading this file alone should give a full picture of what was being worked on.
- Must specify **which file**, **which section**, and **what change** is still needed.
- Must describe **how the work was progressing** so it can be resumed without additional context.

**Cleanup:**
Once all remaining tasks listed in `RESUME.md` have been completed in a later session, **delete `RESUME.md`**.
