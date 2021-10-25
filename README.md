

# TO DO

1. Combinatoric optimization.
1. Move class into header.
1. Update this README.
1. Fix CLion TODOs inside gtest & benchmark submodules, forking those (including CMake warnings).


# word_counting with google test and google benchmark

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

See also the following template from which this repo was adapted:
https://github.com/jingleman/cmake-gtest-gbench-starter
The README there explains the submodules, organizations, usage, etc.


# Debugging in VS Code

Finally got this working by simplifying like in

https://medium.com/@divyendu.narayan/visual-studio-code-setup-in-mac-os-to-build-and-debug-c-cmake-projects-45a78b29e49

You may need to disable the CMake Tools (and other) extension(s) besides the C/C++ one.
Then modify the now-added .vscode launch.json to point to the desired target in the
workspace directory. (No need to Add Folder to Workspace. Just OpenFolder.)
And add args there if applicable. Then launch using Run --> Start Debugging or
correponding buttons (top left and bottom left). This works when you run
cmake -DCMAKE_BUILD_TYPE=Debug and make from outside of VS Code, and hopefully
will from within too. May want to add preBuild command back into this .json but
it's ommitted for now.

I haven't found a way to make the CMake Tools Debug button (bottom right) launch the
existing config or prompt for a new config. It seems to simply hang at llvm --version.
Can hopefully figure this out by googling or editing CMake Tools extension settings
someday but for now just remember not to use
that but instead the two buttons or menu path described above. You can also use fn + F5,
as indicated in the menu.


# Word Counting

The purpose of this particular repository is to experiment with basic word counting
functionality and a simple combinatoric optimization problem inspired by a toy.

The toy is a set of five consonant-vowel-consonant (CVC) words made out of wooden
blocks that spin to combine four beginning consonants with four ending consonants (for
each vowel).

See also https://docs.google.com/spreadsheets/d/1QRt20rpTCyJyiO0vnKrDAx5-xznL2l8qVX4qDVxbEyg/edit?usp=sharing (not public).


# License

[MIT](http://opensource.org/licenses/MIT)

