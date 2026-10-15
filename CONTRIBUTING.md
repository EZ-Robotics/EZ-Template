# Contributing to EZ-Template

You don't need to write code to contribute. If you report a bug or test a fix on your robot, you get the Contributor role in the [Discord](https://discord.gg/EHjXBcK2Gy) and get added to the contributors list. GitHub and the Discord both require you to be 13 or older.

Questions go in the Discord. Bugs and ideas go in a GitHub issue.

## Report a bug

1. Check the robot first. Charge the battery, let hot motors cool down, unplug and replug every cable at both ends, and make sure the ports in your code match the robot.
2. Try a fresh example project from the [releases page](https://github.com/EZ-Robotics/EZ-Template/releases). Change only your ports, wheel size and rpm. If the problem goes away, it's in your setup or code, and the Discord can help.
3. Open an issue with the Bug report form. Tell us what you expected, what the robot did, and the shortest code that shows it. A video or the terminal output helps the most.

If an AI tool found the bug, try it on a real robot before you report it. AI tools often find bugs that never happen on a robot.

Bugs in PROS go to [PROS's GitHub](https://github.com/purduesigbots/pros/issues). VEXos and hardware problems go to VEX support.

## Test a fix

We can't test on every robot, so this helps a lot.

1. Find a fix to try in the issue for a bug you've had, or ask in the Discord what needs testing.
2. Install it on a copy of your project, not your competition code. The issue or PR says how. If it doesn't, ask there.
3. Try what the bug report describes, then run your normal auton to make sure nothing else changed.
4. Comment what happened, even if it didn't work or broke something else. Include your motors, wheel size, rpm, and whether you have tracking wheels. A video helps.

## Fix it yourself

Typos, docs, the example project, and small bug fixes outside `src/EZ-Template/drive/` can go straight to a PR.

Everything else starts with an issue, and waits for a maintainer to OK it. That means anything in `src/EZ-Template/drive/` (the code that drives the robot), a new function teams can call, or more than 30 changed lines in the library.

[FIRST-PR.md](FIRST-PR.md) walks through your first PR with [GitHub Desktop](https://desktop.github.com/). In short
1. Fork the repo to your own GitHub account. Uncheck **Copy the main branch only**.
2. Make a branch from `dev`. Use `website-dev` for the docs site and `example-project` for the example project.
3. Make your change. Turn on Format On Save in VS Code so it gets formatted.
4. Open the PR into the branch you started from. GitHub picks `main` for you, so change it.
5. Fill in the PR form.

Before you start on an issue, comment "I'd like to work on this" so two people don't make the same fix. Look for issues labeled `good first issue`.

To fix a typo on the docs site, click **Edit this page** at the bottom of the page and GitHub walks you through it.

## If AI helped

Any AI tool is fine if you follow its age rules. You're responsible for what you send, same as if you typed it.

- Ask it to explain errors and code instead of writing the fix for you. In [one study](https://doi.org/10.1073/pnas.2422633122), high school students who used a chatbot for answers did worse once it was gone. Students who only got hints didn't.
- Say which tool you used and what it did.
- You should be able to explain every line you send.
- Don't paste names, passwords, or another team's private code into an AI tool.

Your competition has its own AI rules, and they change during the season. Read the current [GRSF Student-Centered Policy](https://library.globalrobotics.org/hc/en-us/articles/51372675292820-Student-Centered-Policy) for V5RC, or the [RECF Student-Centered Policy](https://kb.roboticseducation.org/hc/en-us/articles/5449868745367) for Achieve.

If you run an AI agent on this repo, point it at [AGENTS.md](AGENTS.md).

## After you open a PR

A maintainer reviews it and leaves comments, usually a lot on a first PR. Reply to each one by changing the code or saying why you didn't.

If nobody replies in 7 days, comment on the PR. After 14, ask in the Discord.

Your change reaches teams in the next release.

## More details

You don't need anything below here for your first issue or PR.

<details>
<summary>Branches</summary>

| Branch | What goes there |
| --- | --- |
| `dev` | The library, tests and tools. Pre-releases are tagged here |
| `main` | Stable releases, through a release PR from `dev`. Also the process docs and `.github/`, because GitHub reads them from `main` |
| `website-dev` | Docs site changes |
| `website` | The live docs site. Updated from `website-dev` |
| `example-project` | The project teams download. Never merged into `dev` or `main` |
| `release/X.Y` | Fixes for an older stable release, like `release/3.2` |

If your team is on an older release like 3.2.2 and needs a fix there, say so in the PR. Fixes go to `dev` first, and a maintainer decides whether they also go to the older release.

</details>

<details>
<summary>When releases happen</summary>

As the season goes on, fewer risky changes get merged, and releases avoid the 7 days before a major event like States or Worlds.

</details>

<details>
<summary>Your team's code</summary>

The [EZ-Template-Corpus](https://github.com/EZ-Robotics/EZ-Template-Corpus) is a public list of links to public projects that use EZ-Template. Its README says how to take your project off.

</details>

<details>
<summary>How we decide what gets fixed, and how code is written</summary>

See [WHAT-IS-A-BUG.md](WHAT-IS-A-BUG.md) and [CODING-STYLE.md](CODING-STYLE.md).

</details>

## License

By contributing, you agree your code is released under EZ-Template's [MPL 2.0](LICENSE.md) license.
