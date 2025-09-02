# GitHub Task Management Guide for FRC Team 360

This document outlines how we use GitHub Issues as our task management system, replacing traditional tools like Todoist.

## Why GitHub Issues?

GitHub Issues provides several advantages for our FRC team:
- **Integrated with Code**: Issues link directly to code changes and pull requests
- **Collaborative**: Team members can comment, assign, and track progress together
- **Organized**: Labels, milestones, and projects help categorize and prioritize work
- **Accessible**: Free and already part of our development workflow
- **Trackable**: Full history of discussions and progress

## Issue Types

We use several issue templates for different types of work:

### 🐛 Bug Reports
Use for reporting problems with robot code, hardware, or systems.
- Includes steps to reproduce, expected vs actual behavior
- Environment details and priority levels
- Links to related code or hardware

### ✨ Feature Requests
Use for new capabilities or enhancements to the robot.
- Problem statement and proposed solution
- Implementation considerations and success criteria
- Priority and timeline information

### 📋 Tasks/Todos
Use for general tasks that need to be completed.
- Clear acceptance criteria and dependencies
- Effort estimation and skill requirements
- Due dates and priority levels

### 🔧 Build/Hardware Tasks
Use for mechanical, electrical, and hardware-related work.
- Component/system identification and materials needed
- Safety considerations and documentation requirements
- Testing and validation procedures

## Labeling System

We use labels to categorize and filter issues:

### Type Labels
- `bug` - Something isn't working correctly
- `enhancement` - New feature or improvement
- `task` - General task or todo item
- `build` - Hardware/mechanical work
- `hardware` - Hardware-related
- `documentation` - Documentation improvements

### Priority Labels
- `priority-critical` - Must be fixed immediately
- `priority-high` - Important for upcoming milestones
- `priority-medium` - Normal priority work
- `priority-low` - Nice to have, low urgency

### Status Labels
- `needs-triage` - Needs initial review and assignment
- `needs-assignment` - Ready for someone to take ownership
- `in-progress` - Currently being worked on
- `blocked` - Cannot proceed due to dependencies
- `testing` - In testing/validation phase

### Team Labels
- `team-programming` - Software/code work
- `team-mechanical` - Mechanical engineering work
- `team-electrical` - Electrical work
- `team-design` - CAD/design work

## Workflow

### Creating Issues
1. Choose the appropriate template for your issue type
2. Fill out all relevant sections thoroughly
3. Add appropriate labels for type, priority, and team
4. Assign to team members if you know who should handle it
5. Link to related issues or milestones if applicable

### Working on Issues
1. Assign yourself to issues you're working on
2. Add the `in-progress` label when you start work
3. Comment with progress updates and questions
4. Reference the issue number in commit messages (`Fixes #123`)
5. Move to `testing` label when ready for validation

### Closing Issues
- Issues are automatically closed when linked pull requests are merged
- For non-code issues, manually close when work is complete
- Add a final comment summarizing what was accomplished

## Projects and Milestones

### Milestones
Use milestones to group issues by:
- Competition deadlines (e.g., "Week 1 Competition")
- Build phases (e.g., "Robot Assembly Complete")
- Major features (e.g., "Autonomous Functionality")

### GitHub Projects
Consider using GitHub Projects for:
- Kanban-style boards (To Do, In Progress, Done)
- Sprint planning and tracking
- Visualizing team workload

## Tips for Success

### Writing Good Issues
- **Be specific**: Clear titles and detailed descriptions
- **Include context**: Why is this needed? What's the impact?
- **Add criteria**: How do we know when it's done?
- **Link related work**: Reference other issues, PRs, or documentation

### Staying Organized
- Review and triage new issues regularly
- Update labels and assignees as work progresses
- Close completed issues promptly
- Use comments to track progress and decisions

### Team Communication
- Use @mentions to notify specific team members
- Reference issues in commit messages and PR descriptions
- Link issues to pull requests for automatic closure
- Comment with updates, questions, and blockers

## Comparison to Todoist

| Feature | Todoist | GitHub Issues |
|---------|---------|---------------|
| Task Creation | ✅ | ✅ |
| Due Dates | ✅ | ✅ (milestones) |
| Labels/Categories | ✅ | ✅ |
| Collaboration | ✅ | ✅ |
| File Attachments | ✅ | ✅ |
| Comments/Notes | ✅ | ✅ |
| Code Integration | ❌ | ✅ |
| Free for Teams | ❌ | ✅ |
| Progress Tracking | ✅ | ✅ |
| Notifications | ✅ | ✅ |

## Getting Started

1. **Explore the issue templates** - Try creating different types of issues
2. **Set up notifications** - Configure GitHub to email you about assigned issues
3. **Practice with labels** - Use labels consistently to keep things organized
4. **Create a project board** - Set up a kanban board for visual task tracking
5. **Link everything** - Connect issues to code changes and documentation

## Questions?

If you have questions about using GitHub for task management:
- Create an issue using the "Task/Todo" template
- Ask in our [Chief Delphi build thread](https://www.chiefdelphi.com/t/frc-360-the-revolution-2025-build-thread-open-alliance/479679)
- Check the [GitHub documentation](https://docs.github.com/en/issues)

---

*This system replaces our previous Todoist workflow and provides better integration with our code development process.*