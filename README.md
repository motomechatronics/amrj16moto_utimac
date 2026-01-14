# Basic Linux Course for Robotics

Applied to the project **amrj16moto_utimac (ROS2 Galactic)**

Public reference repository:\
https://github.com/motomechatronics/amrj16moto_utimac

The repository is public and accessible to all candidates.

This test is intended for candidates who wish to approach the world of
mobile robotics using Linux as the reference operating system.

(\*) ROS2 installation is not required.

------------------------------------------------------------------------

# Preface -- Internship Program in Mobile Robotics

The main goal of our internship program is to select promising,
motivated, and reliable students who wish to grow professionally in the
field of mobile robotics.

Through this program, students will have access to our learning platform
based on **The Construct**, where they will find a wide range of ROS
2--based courses, from introductory topics to advanced subjects related
to autonomous robotics.

As a demonstration of learning progress, technical skills, and
organizational abilities, each participant will be required to develop
technical tutorials based on our AMR robot **J16MOTO**, following a
clear, structured, and professional style.

The tutorial topics will be defined in advance and will cover:

-   Linux fundamentals\
-   ROS 2\
-   Gazebo simulation\
-   Navigation\
-   Perception

Both Python and C++ will be used, with a strong focus on practical
application.

The internship will start on **February 1st** and will last **6
months**.\
It will be conducted entirely in **full remote mode**.

Tutorial deadlines will be defined by the intern and reported in a
personal calendar.

However:

-   Missing the first deadline ends the internship.\
-   Missing the second or third still allows one final opportunity.

This rule is necessary to select truly reliable candidates.

Candidates must be able to dedicate at least **5 effective hours per
week**.

A pre-selection technical case study on **The Construct** platform is
required.

No participation fees are required.

### Required background

-   Basic Linux\
-   Basic Python and/or C++\
-   Basic to intermediate ROS2

### During the internship

-   Communication via Slack\
-   Project management via SCRUM on Taiga\
-   NDA signature required

To apply, candidates must send their CV to:\
**Salvatore.volpe@mtmt.it**

------------------------------------------------------------------------

# Index

1.  Test dates\
2.  Test objective\
3.  Required working method\
4.  File name and submission\
5.  Technical validation\
6.  Required skills\
7.  Importance of documentation\
8.  Study support\
9.  Optional Use of Docker
10. Mandatory tutorial structure\
11. Official tutorial index\
12. Tutorial writing rules\
13. Evaluation grid\
14. Passing threshold\
15. Score interpretation\
16. Conclusion

------------------------------------------------------------------------

## Test Dates

-   Test release date: **January 14th**\
-   Submission deadline: **January 27th at 23:59**

Submissions after the deadline will not be accepted.

------------------------------------------------------------------------

## Test Objective

Through this activity, the candidate must demonstrate the ability to:

-   know the fundamentals of Linux\
-   use the terminal\
-   understand the structure of a real software project\
-   apply Linux commands to a robotics project\
-   document work clearly and in an organized manner\
-   create a technical tutorial in Jupyter Notebook

The objective is not only to use Linux, but to learn how to explain
Linux clearly to other students.

------------------------------------------------------------------------

## Required Working Method

The candidate must:

-   Install Linux on their system\
-   Clone the amrj16moto_utimac repository from GitHub\
-   Explore the project structure\
-   Apply the main Linux commands directly to the project\
-   Observe the effects of the commands\
-   Document everything in a Jupyter Notebook tutorial

Each command must be explained by indicating:

-   what it does\
-   what it is applied to\
-   why it is used\
-   what result it produces

The tutorial must be written as if addressed to another beginner
student.

------------------------------------------------------------------------

## File Name and Submission

The Jupyter Notebook file must be named exactly:

**Mtmt_basic_linux_tutorial.ipynb**

and sent to:

**salvatore.volpe@mtmt.it**

Files with different names may not be considered valid.

------------------------------------------------------------------------

## Technical Validation

The test will be considered valid if:

-   the file is correctly submitted\
-   the tutorial is clear and well organized\
-   the commands are coherent\
-   the tutorial can be executed by the commission\
-   the obtained results match the explanations

------------------------------------------------------------------------

## Required Skills

The candidate must demonstrate the ability to:

-   navigate the filesystem\
-   read text files\
-   create, copy, move, and delete files and folders\
-   search for files and contents\
-   understand permissions\
-   interpret a software project structure\
-   document work correctly

------------------------------------------------------------------------

## Importance of Documentation

In robotics and engineering, documentation is an integral part of
technical work.

The test therefore evaluates:

-   clarity\
-   order\
-   precision\
-   communication skills

------------------------------------------------------------------------

## Study Support

Candidates are encouraged to create a free account on:

https://www.theconstruct.ai

and follow the course **Linux for Robotics**, which represents an
excellent example of didactic structure and explanation style.

The course is only a methodological reference: the submitted tutorial
must be original and applied specifically to the amrj16moto_utimac
project.

------------------------------------------------------------------------

## Optional Use of Docker

If the candidate chooses to use Docker to run Ubuntu Linux instead of installing Linux directly on their system, they must document the entire procedure in a complete and detailed manner, including:

- installation of Docker on their operating system
- verification of correct Docker operation
- download of the Ubuntu image
- container creation and startup
- access to the container terminal
- configuration of the Ubuntu environment
- mounting of any volumes required to work with the repository

The entire Docker procedure must be included in the tutorial with:

- commands
- explanations
- outputs
- personal comments

The use of Docker is considered a valid technical deepening and will be positively evaluated only if it is correctly documented and fully functional.

If Docker is used, all required Linux commands must be executed inside the Ubuntu container.
------------------------------------------------------------------------

## Mandatory Tutorial Structure

To allow an objective comparison among candidates, all tutorials must
follow the index reported below.

Personal in-depth sections are allowed, but the minimum index must be
respected.

------------------------------------------------------------------------

## Official Tutorial Index

**Mtmt_basic_linux_tutorial.ipynb**

### 1. Introduction

-   Tutorial objective\
-   Brief project description

**Description of the J16MOTO robot and project**

J16MOTO is an Autonomous Mobile Robot (AMR -- Autonomous Mobile Robot)
designed for research and educational purposes, mainly aimed at:

-   universities\
-   research centers\
-   technical institutes and robotics laboratories

The robot was created as a didactic tool to allow students to apply
mobile robotics learning in a real and structured context.

J16MOTO is conceived as an industrial-like platform, since it uses
typical industrial automation components. This design choice allows
students to work on a platform that faithfully reflects industrial
architectures, bridging the gap between academic training and real
applications.

------------------------------------------------------------------------

### 2. Environment Preparation

-   Operating system\
-   Linux version\
-   Repository cloning

------------------------------------------------------------------------

### 3. Project Exploration

-   `ls`, `tree`, `pwd`\
-   Folder structure

------------------------------------------------------------------------

### 4. Filesystem Navigation

-   `cd`, `..`, `~`\
-   Absolute and relative paths

------------------------------------------------------------------------

### 5. File and Folder Management

-   `mkdir`, `rmdir`\
-   `touch`\
-   `cp`, `mv`, `rm`

------------------------------------------------------------------------

### 6. File Reading

-   `cat`, `less`, `head`, `tail`

------------------------------------------------------------------------

### 7. Search

-   `find`\
-   `grep`

------------------------------------------------------------------------

### 8. Permissions

-   `ls -l`\
-   `chmod`\
-   Meaning of permissions

------------------------------------------------------------------------

### 9. Additional Useful Commands

-   `wc`, `sort`, `uniq`, `echo`\
-   Text editors

------------------------------------------------------------------------

### 10. Command Summary Table

  Command   Description   Application in the project
  --------- ------------- ----------------------------

------------------------------------------------------------------------

### 11. Personal Conclusions

------------------------------------------------------------------------

## Tutorial Writing Rules

Each command must include:

-   explanation\
-   executed command\
-   output\
-   personal comment

The notebook must alternate:

-   Markdown cells\
-   Code cells

------------------------------------------------------------------------

## Evaluation Grid

  Area                    Score
  ----------------------- -----------
  Linux commands usage    0--2
  Filesystem navigation   0--2
  Command understanding   0--2
  Jupyter structure       0--2
  Documentation quality   0--2
  **Total**               **0--10**

------------------------------------------------------------------------

## Passing Threshold

The test is passed with a score ≥ **6/10**

------------------------------------------------------------------------

## Score Interpretation

  Score   Meaning
  ------- ----------------
  9--10   Excellent
  7--8    Good
  6       Sufficient
  \<6     Not sufficient

------------------------------------------------------------------------

## Conclusion

Linux is the operational language of modern robotics.

This test does not only evaluate what you can do, but how clearly you
can explain what you do.

**Learning Linux also means learning how to communicate it.**
