# Contributing Guidelines
Thank you for your interest in contributing to `ros2_roam_bot`.
Whether it's a bug report, new feature, correction, or additional
documentation, we greatly value feedback and contributions from our community.

Please read through this document before submitting any issues or pull requests to ensure we have all the necessary
information to effectively respond to your bug report or contribution.


## Reporting Bugs/Feature Requests
We welcome you to use the GitHub issue tracker to report bugs or suggest features.

When filing an issue, please check [existing open][issues], or [recently closed][closed-issues], issues to make sure
 somebody else hasn't already reported the issue.
Please try to include as much information as you can. Details like these are incredibly useful:

* A reproducible test case or series of steps
* The version of our code being used
* Any modifications you've made relevant to the bug
* Anything unusual about your environment or deployment


## Contributing via Pull Requests
Contributions via pull requests are much appreciated.
Before sending us a pull request, please ensure that:

1. You are working against the latest source on the *master* branch.
2. You check existing open, and recently merged, pull requests to make sure someone else hasn't addressed the problem already.
3. You open an issue to discuss any significant work - we would hate for your time to be wasted.

To send us a pull request, please:

1. Fork the repository.
2. Modify the source; please focus on the specific change you are contributing.
  If you also reformat all the code, it will be hard for us to focus on your change.
3. Ensure local tests pass: `colcon test` and `pre-commit run` (requires you to install pre-commit by `pip3 install pre-commit`).
4. Commit to your fork using clear commit messages.
5. Send a pull request, answering any default questions in the pull request interface.
6. Pay attention to any automated CI failures reported in the pull request, and stay involved in the conversation.

GitHub provides additional documentation on [forking a repository](https://help.github.com/articles/fork-a-repo/) and
[creating a pull request](https://help.github.com/articles/creating-a-pull-request/).


## Finding contributions to work on
Looking at the existing issues is a great way to find something to contribute on.
As this project, by default, uses the default GitHub issue labels
  (enhancement/bug/duplicate/help wanted/invalid/question/wontfix), looking at any ['help wanted'][help-wanted] issues
  is a great place to start.


## Licensing
Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

[issues]: https://github.com/ros-controls/ros2_roam_bot/issues
[closed-issues]: https://github.com/ros-controls/ros2_roam_bot/issues?utf8=%E2%9C%93&q=is%3Aissue%20is%3Aclosed%20
[help-wanted]: https://github.com/ros-controls/ros2_roam_bot/issues?q=is%3Aopen+is%3Aissue+label%3A%22help+wanted%22
[license]: http://www.apache.org/licenses/LICENSE-2.0.html
