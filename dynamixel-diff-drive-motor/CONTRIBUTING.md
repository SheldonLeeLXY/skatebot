# Contributing

Thank you for taking an interest in this project.

Any contribution that you make to this repository will
be under the MIT license, as dictated by that
[license](https://opensource.org/licenses/MIT).

## Pull requests

Before submitting a pull request, all submitted code must pass the following tests.

* The code must build without warnings or errors.
* Running `colcon test` must generate 0 warnings or errors.
* The code must also work, i.e.
  * The server must run and generate zero runtime errors.
  * The server must run and output zero warning or error messages.
  * The launch file `template.launch.py` must start the server with zero
    warnings or errors.
