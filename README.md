### cv1617-60623

# Computer Vision


This repository is destined to be used in the development of projects and exercises associated with the subject Computer Vision, taught in Universidade de Aveiro, Portugal.

These projects use C++ as a programming language, taking advantage of a extensive function library used specifically for computer vision, OpenCV.

## Tools
* cvmake
	- Small shell script that automatically builds, compiles and cleans up projects and exercises.
	- Needs to be stored somewhere, symlinked to `/usr/bin` and have its path added in the bash config file for it to work properly.
	- Can be executed by typing `cvmake -c` in the terminal. The argument `-c` is optional and using it cleans the excess files created during the build.

## Aula 1
* DisplayImage
    - Displays selected image in a window.

* Color2Gray
	- Converts selected image to grayscale, saving the it and displaying both the original and new images.