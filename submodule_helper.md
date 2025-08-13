# 🛠 Submodule Survival Cheat Sheet

This repository uses **Git submodules**.  
Follow these steps to work with them **without losing changes**.

---

## 1️⃣ Navigating to the submodule
```bash
cd path/to/submodule
````

Example:

```bash
cd turtlebot3_multi_robot
```

---

## 2️⃣ Make sure you are on a branch

If you see:

```
HEAD detached at <commit>
```

Switch to a branch **before making edits**:

```bash
git checkout main
# or create a feature branch
git checkout -b my-feature
```

---

## 3️⃣ Committing changes inside the submodule

```bash
git add .
git commit -m "Describe what you changed"
git push origin main
```

(Replace `main` with your branch name if different.)

---

## 4️⃣ Updating the parent repository to point to the new commit

After committing changes in the submodule:

```bash
cd /path/to/parent/repo
git add path/to/submodule
git commit -m "Update submodule to latest commit"
git push origin main
```

> ⚠ If you skip this step, others will not see your changes when they pull the parent repo.

---

## 5️⃣ Pulling updates for both parent and submodules

```bash
git pull --recurse-submodules
git submodule update --init --recursive
```

---

## 6️⃣ Creating a new branch in a submodule

```bash
cd path/to/submodule
git checkout -b my-feature
git push origin my-feature
```

Then update the parent repo pointer:

```bash
cd /path/to/parent/repo
git add path/to/submodule
git commit -m "Point submodule to branch my-feature"
git push origin main
```

---

## 7️⃣ Backing up changes before risky commands

If you think a command might overwrite your work:

```bash
git diff > ../my_submodule_backup.patch
```

Later you can restore:

```bash
git apply ../my_submodule_backup.patch
```

---

✅ Following this guide ensures:

* Your submodule changes are committed and pushed safely
* The parent repo always points to the right commit
* No more “detached HEAD” surprises