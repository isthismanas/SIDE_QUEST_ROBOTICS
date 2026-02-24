# 🚀 Side Quest – Daily Git Checklist

This is the daily startup and shutdown ritual for the SIDE_QUEST repository.

Follow this every day before writing code.

---

# 🌅 START OF DAY

## 0. Go to the repository

cd ~/SIDE_QUEST_ROBOTICS

---

## 1. Check current state (ALWAYS FIRST)

git status

Check:

- What branch am I on?
- Are there uncommitted changes?
- Am I ahead or behind origin?

If you see:
- "Changes not staged for commit" → decide whether to commit or stash.
- "Your branch is behind..." → you need to pull.

---

## 2. Confirm correct branch

git branch

If needed:

git checkout albert/arm-on-connect

(Replace with whatever branch you're actively working on.)

Never work directly on main.

---

## 3. Fetch latest remote updates

git fetch origin

This does NOT change your working files.
It only updates your knowledge of the remote.

---

## 4. If behind, rebase pull

git pull --rebase

Why --rebase?
- Keeps history clean
- Avoids unnecessary merge commits
- Easier to debug later

If conflicts occur:
- Resolve immediately
- git add <file>
- git rebase --continue

Do not ignore conflicts.

---

## 5. Confirm clean working tree

git status

You want to see:

"nothing to commit, working tree clean"

Now you are safe to begin work.

---

# 🧠 DURING THE DAY

Before major changes:

git status

If dirty → commit or stash before structural edits.

Do not stack risky changes on top of uncommitted work.

---

# 🌙 END OF DAY

## 1. Check what changed

git status
git diff

## 2. Stage and commit

git add .
git commit -m "Dev 10: drift tuning + admin TCP improvements"

Use clear, descriptive commit messages.

## 3. Push to remote

git push origin albert/arm-on-connect

Never leave important work unpushed overnight.

---

# 🧯 SAFETY / RECOVERY COMMANDS

## View commit history

git log --oneline --graph --decorate --all

## See file changes

git diff

## Temporarily stash work

git stash
git stash pop

## Undo uncommitted changes (DANGEROUS)

git restore .

Use only if you are sure.

---

# 🏷️ BEFORE IMPORTANT DEMOS

Create a recovery tag:

git tag demo-2026-02-25
git push origin demo-2026-02-25

This lets you instantly roll back if something breaks.

---

# 🔥 THINGS NOT TO DO

- Do not git pull with uncommitted changes.
- Do not work directly on main.
- Do not ignore merge conflicts.
- Do not leave large changes uncommitted.
- Do not push broken code without labeling it clearly.

---

# 🏁 MINIMAL DAILY CORE (If in a rush)

git status
git fetch origin
git pull --rebase

This alone prevents most Git disasters.