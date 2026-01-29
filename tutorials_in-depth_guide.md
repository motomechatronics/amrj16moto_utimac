# How to Read This Tutorial

## Important notes for students

This document is **not a traditional Linux manual** and it is **not a list of commands to memorize**.

It is written to show you **how a junior engineer should think and act** when working on a real robotics project like **J16MOTO**.

---

## 1. Linux is not the goal

Linux commands are **not the subject of this tutorial**.  
They are **tools** used to solve practical problems in a real working context.

If you read this document asking yourself:

> *“What does this command do?”*

you are missing the main point.

Instead, you should always ask:

- **Why is this command needed at this moment?**
- **What problem does it solve in the workflow?**
- **What would happen if I skipped this step?**

---

## 2. Follow the story, not just the commands

The tutorial is written as a **guided narrative**.

Each section exists because the **previous one created a necessity**:

- You verify the system → because assumptions are dangerous  
- You clone the repository → because you need a clean working copy  
- You explore directories → because you must not get lost  
- You read documentation → because acting blindly causes errors  
- You search → because projects are too large to scan manually  
- You execute a script → because a tutorial must be interactive and verifiable  

👉 **Nothing is accidental.**  
If a step feels “obvious”, ask yourself **why it is still there**.

---

## 3. Interactivity is mandatory

A tutorial that does not work exactly as described is **wrong by definition**.

For this reason:

- every command in this document produces a **real output**
- the script `test_robot.sh` is **contextual**, not artificial
- you are encouraged to **change keywords**, re-run commands, and observe differences

If something does not work:

- do not assume it is “your fault”
- analyze **where the reasoning chain broke**

This is how engineers debug.  
This is **not** how students memorize.

---

## 4. This document is a quality reference

This tutorial is provided as an **example of how technical documentation can be written better**, not as the only correct solution.

When you write your own tutorials, you should aim to:

- create a clear context
- explain *why* before *how*
- connect each section to the next one
- guide the reader’s reasoning, not just their typing

If your reader can follow your tutorial **without getting lost or confused**, you have done your job well.

---

## 5. What you should learn from this file

By the end of this tutorial, you should not only know **more Linux**.

You should understand:

- how Linux supports professional robotics work
- how engineers approach unknown projects
- how to write documentation that others can actually use

> **Good engineers write code.  
> Great engineers make their work understandable to others.**

Keep this mindset while reading — and while writing your own tutorials.


## 6. Practical suggestion
# Basic Linux Tutorial (Narrative & Interactive)  
## Applied to the **J16MOTO** project (`amrj16moto_utimac`)

This tutorial is written as a **guided story**: every step creates the need for the next one.  
You will apply Linux commands directly to a real robotics repository, so **everything you run produces a real, verifiable result**.

---

## 1) The first day on the J16MOTO team

You just joined the J16MOTO engineering team. Your mentor gives you one instruction:

> “Before you touch anything, prove you can work safely in our Linux + robotics workflow.”

That means:
- verify your environment
- obtain a clean project copy
- explore structure without breaking anything
- learn to search and validate
- run a small, meaningful script that interacts with the repository

---

## 2) Environment preparation: “What system am I on?”

In robotics, compatibility matters. The first professional reflex is to **verify the OS**.

```bash
cat /etc/os-release
```

**Why we do it:**  
Because tool availability, versions, and reproducibility depend on the OS.

**Bridge to the next step:**  
Now that you know your environment, you can safely bring the project onto your machine.

---

## 3) Getting the project: “Give me a clean copy”

Your mentor warns you:

> “Never work on a dirty folder. Always start clean.”

So you remove any old copy (if it exists) and clone the repository again.

```bash
rm -rf amrj16moto_utimac
git clone https://github.com/motomechatronics/amrj16moto_utimac.git
```

**Why we do it:**  
Because leftover files can create false errors and confusion.

**Bridge to the next step:**  
Now the repository exists locally, but you are still outside it. You must **enter** and orient yourself.

---

## 4) Orientation: “Where am I and what’s here?”

Enter the project directory:

```bash
cd amrj16moto_utimac
```

Confirm your current path:

```bash
pwd
```

List the contents in a way that highlights folders:

```bash
ls -F
```

**What you should notice:**  
You’ll see multiple directories. In a robotics repo, each folder usually maps to a subsystem (description, simulation, messages, navigation, etc.).

**Bridge to the next step:**  
Seeing a list is not enough—you must identify the **main components** quickly.

---

## 5) Project exploration: “What are the main components?”

List only directories:

```bash
ls -d */
```

This gives you a clean view of the top-level “modules” of the robot software.

Now, practice moving around **deliberately** (so you don’t get lost later):

```bash
cd amrj16moto_utimac
pwd
cd ..
```

**Why we do it:**  
Navigation mistakes are the #1 cause of “I ran the command but nothing happened.”

**Bridge to the next step:**  
Now you can move safely—time to perform controlled file operations (without touching critical files).

---

## 6) File & folder management: “Work without risking the project”

A good engineer creates a temporary workspace for experiments.

```bash
mkdir my_logs
touch my_logs/session1.log
```

Before changing anything important, you create a backup:

```bash
cp README.md README_backup.md
```

Then you move your backup into your workspace:

```bash
mv README_backup.md my_logs/README_old.md
```

Verify the result:

```bash
ls -R my_logs
```

Clean up (only what you created):

```bash
rm -r my_logs
```

**Bridge to the next step:**  
You now know how to manipulate files safely. But you still need to **read** and extract information quickly.

---

## 7) Reading files: “Read before you act”

Start with the main documentation:

```bash
cat README.md
```

Sometimes you only need the beginning:

```bash
head -n 5 README.md
```

**Why we do it:**  
Documentation tells you how the project is meant to be used.

**Bridge to the next step:**  
Real projects are large. You can’t open every file manually. You must learn to **search**.

---

## 8) Search: “Find what matters, fast”

Find all Python scripts:

```bash
find . -name "*.py"
```

Search for a keyword in the README:

```bash
grep -i "robot" README.md
```

**Why we do it:**  
In robotics, you often know *what concept* you need (“navigation”, “slam”, “gazebo”), but not where it lives.

**Bridge to the next step:**  
Once you can find information, you’re ready to run a controlled experiment—but Linux will only let you execute what is explicitly executable.

---

## 9) Permissions & execution: “Make it real and interactive”

A tutorial becomes convincing when it is **interactive** and **works exactly as expected**.

Instead of a fake `Hello Robot` script with no context, we create a small repository health-check script.
It will:
- verify you are in the repo root
- list the main `amrj16_*` packages (if present)
- locate ROS package manifests (`package.xml`) if available
- search for a keyword inside the repo

### 9.1 Create `test_robot.sh` (contextual, real, reproducible)

From the repo root (`amrj16moto_utimac`), create the script with a heredoc:

```bash
cat > test_robot.sh <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

info()  { echo "[INFO]  $*"; }
ok()    { echo "[OK]    $*"; }
warn()  { echo "[WARN]  $*"; }
fail()  { echo "[ERROR] $*" >&2; exit 1; }

# Must be run from repo root
if [[ ! -d ".git" ]]; then
  fail "Run this from the root of a Git repository ('.git' not found)."
fi
if [[ ! -f "README.md" ]]; then
  fail "README.md not found. Are you in the amrj16moto_utimac repo root?"
fi

info "J16MOTO repo check started in: $(pwd)"
ok "README.md found."

info "Repository header (README.md, first lines):"
head -n 20 README.md | sed '/^[[:space:]]*$/d' | head -n 5
echo

info "Discovering main packages (amrj16_*):"
packages=( amrj16_* )
if [[ ${#packages[@]} -eq 0 || "${packages[0]}" == "amrj16_*" ]]; then
  warn "No 'amrj16_*' directories found at root (repo layout may differ)."
else
  ok "Found ${#packages[@]} package(s):"
  for p in "${packages[@]}"; do
    [[ -d "$p" ]] && echo "  - $p"
  done
fi
echo

info "Searching for ROS package manifests (package.xml) within depth 3..."
pkgxml_count=$(find . -maxdepth 3 -name "package.xml" 2>/dev/null | wc -l | tr -d ' ')
if [[ "$pkgxml_count" -gt 0 ]]; then
  ok "Found $pkgxml_count package.xml file(s). Sample:"
  find . -maxdepth 3 -name "package.xml" 2>/dev/null | head -n 5
else
  warn "No package.xml found within depth 3."
fi
echo

KEYWORD="${1:-navigation}"
info "Searching keyword '$KEYWORD' (case-insensitive) in README.md and project files..."
matches=$(grep -RIn --exclude-dir=.git -i "$KEYWORD" README.md . 2>/dev/null | head -n 10 || true)

if [[ -n "$matches" ]]; then
  ok "Matches found (showing up to 10):"
  echo "$matches"
else
  warn "No matches found for '$KEYWORD'. Try: slam, gazebo, map, robot."
fi
echo

info "Top-level folders snapshot:"
ls -d */ 2>/dev/null || true
echo

ok "Repo check completed successfully."
info "Tip: run './test_robot.sh slam' or './test_robot.sh gazebo' to search other topics."
EOF
```

### 9.2 Check default permissions

```bash
ls -l test_robot.sh
```

### 9.3 Make it executable

```bash
chmod +x test_robot.sh
```

Verify the `x` flag appears:

```bash
ls -l test_robot.sh
```

### 9.4 Run it (interactive!)

```bash
./test_robot.sh
./test_robot.sh gazebo
./test_robot.sh slam
```

### 9.5 Cleanup

```bash
rm test_robot.sh
```

---

## 10) Final conclusions: Linux as professional practice

You did not just “learn commands.” You learned a workflow:

1) verify the environment  
2) obtain a clean repository  
3) explore safely and deliberately  
4) read documentation  
5) search efficiently  
6) test permissions and execute controlled scripts  

> This is how Linux supports real robotics work on J16MOTO: step by step, with intention.

---

## Command summary (quick reference)

| Goal | Commands used |
|---|---|
| Verify OS | `cat /etc/os-release` |
| Get project | `rm -rf`, `git clone` |
| Orientation | `cd`, `pwd`, `ls -F`, `ls -d */` |
| File ops | `mkdir`, `touch`, `cp`, `mv`, `ls -R`, `rm -r` |
| Read files | `cat`, `head` |
| Search | `find`, `grep` |
| Permissions | `ls -l`, `chmod +x`, `./script.sh` |
