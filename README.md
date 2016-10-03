### cv1617-60623

# Computer Vision


This repository is destined to be used in the development of projects and exercises associated with the subject Computer Vision, taught at Universidade de Aveiro, Portugal.

These projects use C++ as a programming language, taking advantage of a extensive function library used specifically for computer vision, OpenCV.

The folders are organized as follows:
```
Home Dir (eg. Aula1)
  |
  |_____ src (source files)
  |         |
  |         |_____ CMakeLists.txt
  |
  |_____ bin (binary files)
  |
  |_____ img (image files)
  |
  |_____ CMakeLists.txt
```

## Tools
* cvmake
	- Small shell script that automatically builds, compiles and cleans up projects and exercises.
	- Needs to be stored somewhere, symlinked to `/usr/bin` (eg. `ln -s ~/scripts/cvmake /usr/bin/cvmake`) and have its path added in the bash config file for it to work properly (eg. `PATH=~/scripts:$PATH`).
	- Can be executed by typing `cvmake -c` in the terminal. The argument `-c` is optional and using it cleans the excess files created during the build.
* cvdir
	- Shell script that automatically creates directories and populates them with the needed "CMakelists" files.
	- Needs to be stored somewhere, symlinked to `/usr/bin` (eg. `ln -s ~/scripts/cvdir /usr/bin/cvdir`) and have its path added in the bash config file for it to work properly (eg. `PATH=~/scripts:$PATH`).
	- Can be executed by typing `cvdir` in the terminal followed by the directories' (that will be created) names as arguments (eg. `cvdir folder1 folder2`).

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

## Aula 3
* VideoCapture
	- Reads images from computer camera feed, outputting them sequentially to form a video feed.
	- Manipulates the video frames, displaying them as a grayscale image or as a black & white image, with several options of thresholding.
* DetectSkin
	- Outputs computer camera information as a video feed.
	- Converts video output into several colorspaces (selected by the user), followed by extracting the pixels of the image that fall within a certain range, with the intent of finding objects with the color of skin.
	- Superimposes the pixels found with the previous method on the original video feed.
* ImageFilter
	- Filters a video feed with several filters, chosen by the user.
* GetHist
	- Reads an image from the computer camera feed and analyzes it, displaying its histograms.
	- Capable of displaying BGR histograms, grayscale histogram, or both at the same time, for comparison.
