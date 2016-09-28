### cv1617-60623

# Computer Vision


This repository is destined to be used in the development of projects and exercises associated with the subject Computer Vision, taught at Universidade de Aveiro, Portugal.

These projects use C++ as a programming language, taking advantage of a extensive function library used specifically for computer vision, OpenCV.

## Tools
* cvmake
	- Small shell script that automatically builds, compiles and cleans up projects and exercises.
	- Needs to be stored somewhere, symlinked to `/usr/bin` (eg. `ln -s ~/scripts/cvmake /usr/bin/cvmake`) and have its path added in the bash config file for it to work properly (eg. `PATH=~/scripts:$PATH`).
	- Can be executed by typing `cvmake -c` in the terminal. The argument `-c` is optional and using it cleans the excess files created during the build.

## Aula 1
* DisplayImage
    - Displays selected image in a window.

## Aula 2
* Color2Gray
	- Converts selected image to grayscale, saving it and displaying both the original and new images.
* AddImages
	- Adds two images with the same pixel size to output a new image. Needs the input of the alpha of the first image.
* ChangeBrigtCront
	- Reads and image, takes alpha and beta as input values for levels of brightness and constrast and outputs new image with those levels modified.
	- Reads images from computer camera feed.