# Mission Control

Version: `1.1.0`

The Mission Control node is responsible for the main mission logic.
It gathers all required information from the different nodes and chooses the next action.

For most things it will not execute these actions directly but rather delegate tasks to specialized nodes for the task and coordinate those nodes.

Additionally, Mission Control implements several Failsafe Checks to ensure that all the nodes are working correctly.

## Documentation

See the full documentation for details: [Mission Control Documentation](https://docs.google.com/document/d/1BV6CUhAO0_3A8xje5DsFQydK13K3FuRfkojx6XGCMnk/edit?usp=drive_link)
