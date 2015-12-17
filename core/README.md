TODO remove this unnecessary middleman

Why?
- The agent chooses when to connect to the "robot device"; although complicated, it actually makes much more sense than this setup

For now this is used as a test suite for the following reasons:

<ol>
  <li>We cannot find a way to incorporate the user/server level layer with the artificial agent, right now its a one way pass, but somehow they need to coorperate</li>
  <li>The robot's communication relies on the management of the pose3d that is being sent in between. It's a terrible middle man data object, but it works for the current situation</li>
</ol>

Issues:
<ul>
  <li>sensor non-generalization</li>
</ul>

Use the message model as an inspiration, but do not rely on it.
