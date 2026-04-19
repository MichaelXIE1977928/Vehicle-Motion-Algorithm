# Security policy

## Supported versions

Security fixes are applied to the **default branch** of this repository (currently the active development line). Older tags or branches may not receive backports unless explicitly stated in a release note.

## Reporting a vulnerability

Please **do not** open a public GitHub issue for undisclosed security problems.

**Preferred:** use [GitHub Security Advisories](https://docs.github.com/en/code-security/security-advisories/guidance-on-reporting-and-writing-information-about-vulnerabilities/privately-reporting-a-security-vulnerability) for this repository (“Security” tab → “Report a vulnerability”).

If that is not available, open a **private** communication channel agreed with the maintainer (for example, a maintainer email or organization process, once published in the repository or maintainer profile).

Include:

- A short description of the issue and its impact
- Steps to reproduce (proof of concept if possible)
- Affected components (e.g. `vma.gui`, `vma.api`, dependencies)

You should receive an acknowledgment within a reasonable time; resolution time depends on severity and maintainer availability.

## Scope

This project is research and engineering tooling. It is **not** hardened for hostile network exposure. Treat any future HTTP API or networked deployment as requiring a separate security review.
