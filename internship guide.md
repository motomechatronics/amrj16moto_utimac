# Internship Description  
## ROS2 Simulation & Tutorial Development for Autonomous Mobile Robot

**Company:** MOTO Mechatronics  
**Platform:** TheConstruct.ai  
**Reference Repository:**  
https://github.com/motomechatronics/amrj16moto_utimac

---

## Table of Contents

1. Internship Goal  
2. How to Organize Your Time and Work During the Internship  
3. Learning Path – TheConstruct Courses  
4. Reference Simulation Environment (Mandatory)  
5. What the Candidate Is Required To Do  
   - 5.1 Environment Update  
   - 5.2 Tutorial Development (Core Activity)  
6. Tutorial Reference Constraint (Critical Requirement)  
7. Reproducibility & Quality Standard  
8. Expected Deliverables  
9. Evaluation Criteria  

---

## 1. Internship Goal

The goal of this internship is to train the candidate in **ROS2-based mobile robotics** through a structured learning path that combines:

- selected courses on **TheConstruct.ai**,  
- hands-on work on a **realistic robot simulation environment**,  
- the production of **high-quality, reproducible technical tutorials**.

The internship outcome is not only technical learning, but also the creation of **clear and reliable educational material** that can be reused by other students or team members.

---

## 2. How to Organize Your Time and Work During the Internship

Time management and work organization are **core requirements** of this internship.  
Planning your work, estimating effort, and respecting deadlines are considered **engineering skills**, not optional soft skills.

Please read and follow the instructions below **carefully and precisely**.

### 2.1 Estimating the Time Required for a Course

Each course provided by **TheConstruct.ai** includes a declared estimated duration.

To obtain a realistic estimate of the effort required:

1. Take the declared duration of the course.
2. **Multiply it by 1.5**.

This adjusted value represents the realistic time needed to:
- study the material,
- run and understand examples,
- complete exercises,
- review unclear topics.

**Example:**  
If a course is declared as 20 hours, you must plan **30 hours**.

---

### 2.2 Converting the Estimate into a Weekly Plan

Once you have the total estimated time:

1. Decide how many hours per week you can **realistically and consistently** dedicate to the internship.
2. Divide the total estimated time by your weekly availability.

This gives you the **number of weeks** required to complete the course.

**Example:**  
- Total estimated time: 30 hours  
- Weekly availability: 6 hours/week  
- Required duration: 5 weeks  

This planning step must be completed **before starting** the course.

---

### 2.3 Allocating Time for the Tutorial

After completing a course, you must allocate **an additional 5 hours** to produce a tutorial.

⚠️ **Important clarification**

- The course **ROS2 in 5 Days (Python)** represents a **solid learning foundation** and a **useful guide** for understanding ROS2 concepts *and* for learning how to structure a tutorial.
- It is **not**, under any circumstances, material to be copied.
- It serves as a **learning reference**, not as a deliverable template.

The educational objective of this internship is different.

---

### 2.4 Our Educational Objective

The real educational goal is to:

- build **original tutorials centered on J16MOTO**,  
- use the **real project code** from `amrj16moto_utimac`,  
- write documentation from the perspective of an engineer working on this robot and this codebase.

In other words:

> You are not learning ROS2 in the abstract.  
> You are learning ROS2 **through the J16MOTO robot**.

---

### 2.5 Calendar, First Deadline, and SCRUM Workflow

Using the steps above, you must define:

- a weekly work schedule,
- a clear delivery date for your **first tutorial**.

This date represents your **first official deadline**.

⚠️ **Critical rule**  
If you miss the first delivery deadline, **the internship will be terminated**.  
No deadline extensions will be granted due to poor planning.

Once the calendar and first deadline are defined, you will be introduced to:

- the **SCRUM methodology**,  
- the online project management tool **Taiga**.

Taiga will be used to:
- organize tasks and user stories,
- plan weekly work,
- track progress,
- manage deadlines in a professional engineering workflow.

---

### 2.6 Mandatory Deliverables for the First Deadline

At the first delivery date, you must submit **both**:

1. **ROS2 for J16MOTO** tutorial  
   (conceptually extracted from *ROS2 in 5 Days – Python*, but fully rewritten and adapted)
2. **Linux Basics** tutorial

Submitting only one of them is considered **incomplete work**.

---

## 3. Learning Path – TheConstruct Courses

During the internship, the candidate is required to follow and complete selected courses on **TheConstruct.ai**, including (but not limited to):

- ROS2 in 5 Days (Python)  
- ROS2 Basics (nodes, topics, messages)  
- ROS2 Services and Actions  
- ROS2 URDF/XACRO  
- ROS2 Navigation (Nav2)  
- Simulation with Gazebo / Ignition  

These courses provide the theoretical and practical background necessary to develop the requested tutorials.

---

## 4. Reference Simulation Environment (Mandatory)

All internship activities **must be based on the AMRJ16MOTO robot simulation environment**, available at:

https://github.com/motomechatronics/amrj16moto_utimac

At the moment, the simulation environment is functional; however, **a mandatory part of the internship consists in updating and aligning it with:**

- **ROS2 Humble**
- **Gazebo Ignition**

This update activity is considered an **integral part of the internship tasks**.

---

## 5. What the Candidate Is Required To Do

### 5.1 Environment Update

- Analyze the current state of the AMRJ16MOTO simulation  
- Update packages, launch files, and configurations to be
