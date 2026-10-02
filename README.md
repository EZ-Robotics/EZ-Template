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
