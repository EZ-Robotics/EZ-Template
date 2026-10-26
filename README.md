[![Deployment to Pages](https://github.com/EZ-Robotics/EZ-Template/actions/workflows/deploy-to-github-pages.yml/badge.svg?branch=website)](https://github.com/EZ-Robotics/EZ-Template/actions/workflows/deploy-to-github-pages.yml)
# Website

This website is built using [Docusaurus 3](https://docusaurus.io/), a modern static website generator.

### Installation

```
$ npm install
```

### Local Development

```
$ npm run start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

### Build

```
$ npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

### API reference from the headers

Some entries take their text from the library's header comments. The text is saved in this repo as a snapshot (`api/data/`), so editing pages needs only `npm run start`, no Doxygen.

**Place a function.** Keep the page's `### heading()`, then add a tag under it. Put the example inside the tag (a `\code` example in the header wins if there is one). `args` lists every parameter type, to pick an overload. Three more attributes are optional:

- `args="a | b"` shows several overloads in one entry, like a short and a long form. The text comes from the one with the most parameters.
- `also="x, y"` covers sibling functions, like the forward and backward versions. Their signatures are listed too, and each keeps its own link. When a sibling is overloaded, the overloads with the same parameter types as the ones the entry shows are listed (so `args="ez::QLength, int | ez::QAngle, int"` lists both forms of each sibling).
- `summary="..."` replaces the header's one-line summary. Use it rarely, because that text then lives on the page and not in the header.

```
<ApiFunction name="pid_wait" />
<ApiFunction name="Drive::initialize" />
<ApiFunction name="pid_drive_set" args="ez::QLength, int | ez::QLength, int, bool, bool" />
<ApiFunction name="pid_drive_constants_set" also="pid_drive_constants_forward_set, pid_drive_constants_backward_set" />
```

A tag that does not match fails the build with the file, line and the candidates.

Keep example code to 100 columns or fewer, comments included, so it fits on screen without scrolling sideways. Put a long comment on its own line above the code.

**Preview header edits.** Install Doxygen (`brew install doxygen`, or `winget install DimitriVanHeesch.Doxygen`), then `npm run api:update -- ../EZ-Template --use` and `npm run start`. Before committing, delete `api/data/local.json` and undo the change to `api/config.json`.

**Release day.** Use Doxygen 1.18.0 so snapshots only change when the headers do. Run `npm run api:update -- vX.Y.Z --use`. It lists the functions added, removed or reworded and any placed tag that would break. Commit `api/`, open the PR into `website-dev`, and promote it as below. The live text only changes when a new snapshot is committed.

**Cutting a docs version.** After `npm run docusaurus docs:version X.Y.Z`, add `"X.Y.Z": "vX.Y.Z"` under `versions` in `api/config.json`. A tag in a version that is not mapped fails the build.

### Branches and promoting docs

| Branch | Role |
| --- | --- |
| `website-dev` | Staging. Docs changes land here first, through PRs from topic branches (squash is fine). Test here. |
| `website` | The live site. Every push to it deploys to GitHub Pages. |

Changes only flow one way, `website-dev` into `website`. Do not commit to `website` directly, do not merge `website` back into `website-dev`, and do not force-push either branch.

**Test on staging.** Check out `website-dev`, run `npm ci`, then `npm run start` for the dev server, or `npm run build && npm run serve` for the real output. `onBrokenLinks` is `throw`, so a bad link fails the build.

**Promote to the live site.**

1. Make sure anything the docs link to is live, such as a release page or a download.
2. Open a PR with `website-dev` as the head and `website` as the base. The **Docs build check** runs on it and a checklist comment is posted.
3. Merge it with **Create a merge commit**. Do not squash or rebase it.
4. Merging deploys. Watch the **Deploy static content to Pages** workflow.

**Why a merge commit.** A squash puts one new commit on `website` that `website-dev` does not have. The branches diverge, and the next promotion conflicts in every file both sides touched. The squash of #395 caused this once.

**If a promotion PR shows conflicts.** `website` has a commit that `website-dev` does not. List it with `git log origin/website-dev..origin/website`. If its changes are already in `website-dev`, promote through a branch made from `website-dev` after `git merge -s ours origin/website`, which keeps the `website-dev` content and only joins the histories. If they are not in `website-dev`, bring them in with a normal PR into `website-dev` first.

### Deployment

`.github/workflows/deploy-to-github-pages.yml` builds the site and deploys it to GitHub Pages on every push to `website`. Nothing needs to be run by hand. `.github/workflows/docs-build-check.yml` builds it on every PR into `website-dev` or `website`, so a build failure shows up before the merge.
