# PR, Promotion, and Release Workflow

This repository uses two workflows:

- `build_latest`: validation builds on pushes and PRs.
- `build_release`: tag-driven release builds that create draft GitHub releases.

## 1) PRs into `develop` (feature and fix intake)

This is the primary point where release categorization is defined.

### Goals

- Keep changes reviewable and scoped.
- Set release category labels once on the original PR.
- Use clear PR titles so release notes are customer-readable.

### Branch naming best practice

- Use short, purpose-first names.
- Examples: `fix/snapcast-reconnect-race`, `feature/radar-autodetect`, `improvement/wifi-buffer-tuning`.

### PR title best practice

- Write titles in customer-facing language.
- Good: `Fix snapcast reconnect race that could stall playback`
- Good: `Add LD2450 auto-detect fallback for startup recovery`
- Avoid: `wip`, `misc changes`, `cleanup`.

### Required labels on PRs to `develop`

Add at least one release category label:

- `feature` or `enhancement`
- `improvement` or `performance` or `hardware`
- `fix` or `bug`
- `breaking` or `migration`
- `internal` (for non-customer-facing changes)

Optional label:

- `skip-changelog` for CI/admin/sync-only work that should not appear in customer notes.

### Merge best practice for `develop`

- Wait for `build_latest` checks to pass.
- Prefer merge behavior that preserves commit-to-PR traceability.
- Avoid unnecessary history rewrites on release-path commits.

Why this matters: release notes are generated later from tag commit ranges, and the changelog action resolves commit SHAs back to their original PR labels.

## 2) Promotion PRs (`develop -> staging` and `staging -> main`)

Promotion PRs should not duplicate feature categorization.

- Categories come from the original PRs merged into `develop`.
- Promotion PRs are operational wrappers and should be labeled accordingly.

### Promote `develop` to `staging` (beta lane)

- Open PR: `develop -> staging`.
- Ensure `build_latest` checks are green on the PR.
- Label as `promotion` or `sync`.
- Add `skip-changelog` if the PR is only a branch promotion wrapper.
- Use a clear title such as `Promote develop to staging for beta candidate`.

### Promote `staging` to `main` (production lane)

- Open PR: `staging -> main`.
- Ensure `build_latest` checks are green on the PR.
- Label as `promotion` or `sync`.
- Add `skip-changelog` when appropriate.
- Use a clear title such as `Promote staging to main for production candidate`.

### If GitHub says `staging` is out of date with `main`

Create a dedicated sync PR from `main` back into `staging`:

```bash
git fetch origin
git checkout -b sync/staging-with-main origin/staging
git merge --no-ff origin/main
git push -u origin sync/staging-with-main
```

Then open PR `sync/staging-with-main -> staging` and merge with `[skip ci]` when allowed by branch protection.

## 3) Release flow (tag-driven)

Releases are triggered by tags, not by PR labels.

### Beta release from `staging`

1. Confirm latest `build_latest` on `staging` is green.
2. Tag the exact release commit:

```bash
git fetch origin
git checkout staging
git pull
git tag -a vX.Y.Z-beta.N -m "vX.Y.Z-beta.N"
git push origin vX.Y.Z-beta.N
```

3. `build_release` runs from the tag and creates a draft prerelease.
4. Review/edit notes, then publish.

### Production release from `main`

1. Confirm latest `build_latest` on `main` is green.
2. Tag the exact release commit:

```bash
git fetch origin
git checkout main
git pull
git tag -a vX.Y.Z -m "vX.Y.Z"
git push origin vX.Y.Z
```

3. `build_release` runs from the tag and creates a draft release.
4. Review/edit notes, then publish.

## 4) ESPHome version handling

- `requirements.txt` is the single source of truth for ESPHome version.
- Workflows derive the build version from `requirements.txt`.
- Release notes include resolved ESPHome version in `Build Info`.


## 5) Quick troubleshooting

- Release workflow fails with "must run on a tag ref": run release from a pushed `v*` tag, not a branch.
- Changelog is noisy: verify original PR labels and apply `skip-changelog` to sync/CI/admin PRs.
- `staging -> main` says out of date: run a dedicated `main -> staging` sync PR first.
