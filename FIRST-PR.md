# Your first PR

This page walks through sending a change to EZ-Template with [GitHub Desktop](https://desktop.github.com/) and VS Code. The rules for what needs an issue first are in [CONTRIBUTING.md](CONTRIBUTING.md).

You need a free GitHub account (you have to be 13 or older), GitHub Desktop, and VS Code with the PROS extension. A fork is your own copy of the repo on GitHub. A clone is that copy on your computer.

## Make the change

1. **Fork it.** On the [EZ-Template GitHub page](https://github.com/EZ-Robotics/EZ-Template), click **Fork**. Pick your own account as the owner, so maintainers can help on your PR. A team organization won't work for that, so if you already have a fork under your team, make a new one. Uncheck **Copy the main branch only**, so your fork gets `dev` too, then click **Create fork**.
2. **Clone it.** In GitHub Desktop, choose **File**, then **Clone repository**, and pick your fork. If it asks how you plan to use the fork, choose **To contribute to the parent project**.
3. **Pick the starting branch.** Click **Current branch** and switch to `dev` for the library, `website-dev` for the docs site, or `example-project` for the example project.
4. **Make your branch.** Choose **Branch**, then **New branch**. Name it after what it does, like `bug/selector-page-skip` or `docs/fix-typo`. Under **Create branch based on**, pick the branch from step 3. It starts on `upstream/main`, so change it.
5. **Turn on Format On Save.** In VS Code, open Settings, click the **Workspace** tab, search for "format on save", and check it. It may not match GitHub's format check exactly. If only that check fails, a maintainer fixes it.
6. **Make your change.** In GitHub Desktop, choose **Repository**, then **Open in Visual Studio Code**. Fix one problem, and save.

## Try it on a robot

The repo is a full PROS project. `dev` needs PROS kernel 4.2.2 and ARM GCC 14 or newer. If the build fails with an error about `gnu++26`, your toolchain is too old, so ask in the Discord.

1. In `src/main.cpp`, set up the drive for your robot, like ports, wheel size and rpm. Comment out anything your robot doesn't have.
2. Put an auton that uses your change in `src/autons.cpp`. Code from an older EZ-Template version may need small changes to build on `dev`.
3. Build and upload with the PROS buttons.
4. Before you commit, undo your edits to `src/main.cpp` and `src/autons.cpp`. In GitHub Desktop, right-click each file and choose **Discard changes**.

Write what you saw before and after the fix in the PR. A video helps a lot.

## Open the PR

1. **Commit.** In GitHub Desktop, write a short summary like `Fix the auton selector skipping a page` and click **Commit**. Then click **Push origin**, or **Publish branch** the first time.
2. **Open it.** Choose **Branch**, then **Preview Pull Request**. It picks `main` as the base, so change the base to the branch from step 3, then click **Create pull request**.
3. **Finish in the browser.** Check the base branch again. Start the title with a gitmoji from the table below, like `🐛 Fix the auton selector skipping a page`. Fill in the form, check only what you did, and leave **Allow edits from maintainers** checked so a maintainer can add a test or fix formatting.

| Gitmoji | Use it for |
| --- | --- |
| 🐛 | A bug fix |
| ✏️ | A typo |
| 📝 | Docs |
| ✨ | A new feature |

The rest are at [gitmoji.dev](https://gitmoji.dev).

## Fixing a typo in the browser

Click **Edit this page** at the bottom of any docs page. Click the pencil icon, make the change, and click **Propose changes**. GitHub makes the fork for you. On the next two pages, click **Create pull request**, and check the base branch is `website-dev`. Write "none" for the robot questions in the form.

## Prefer the terminal?

```
git clone https://github.com/<your-username>/EZ-Template.git
cd EZ-Template
git remote add upstream https://github.com/EZ-Robotics/EZ-Template.git
git fetch upstream
git switch -c bug/selector-page-skip upstream/dev
sh tools/format.sh          # needs clang-format 18.1.8: pip install clang-format==18.1.8
pros make                   # optional, builds the library
make -C test                # optional, runs the host tests
git push -u origin bug/selector-page-skip
```

On Windows, run these from Git Bash. Then open the PR on GitHub and change the base branch to `dev`.
