#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <boost/filesystem.hpp>

using namespace cv;
using namespace std;
using namespace boost::filesystem;

int main(int argc, char **argv) {

	// Code taken in part from: http://stackoverflow.com/questions/67273/how-do-you-iterate-through-every-file-directory-recursively-in-standard-c

	// For now, use the current directory as the searchPath
	string searchPath = ".";
	path p(searchPath);

	// Create "end-of-list" iterator
	// (basically, the default constructor gives you what will
	// be returned by the iterator when there are no more elements left)
	directory_iterator end_itr;

	// Create directory iterator for this path, and then
	// loop through everything in directory
	for (directory_iterator itr(p); itr != end_itr; ++itr) {

		// Print path to file
		string folderpath = itr->path().parent_path().string();
		cout << folderpath << "/";

		// Print filename
		string filename = itr->path().filename().string();
		cout << filename << " ";

		// Print extension
		string extension = itr->path().extension().string();
		cout << extension << " ";

		// Is this element a directory?
		if (is_directory(itr->status())) {
			cout << "DIRECTORY" << endl;
		}
		else if (is_regular_file(itr->status())) {
			cout << "REGULAR FILE" << endl;

			// Is this an image file?
			if (extension == ".png" || extension == ".PNG") {
				// Load image as OpenCV Mat
				Mat image;
				cout << "\tLoading image: " << filename << endl;
				image = imread(folderpath + "/" + filename, IMREAD_GRAYSCALE);
				
				// Check if data is invalid
				// NOTE: Shouldn't be a problem, since we already know the file is there, but perhaps
				// it's corrupted or something else went wrong.
				if (!image.data) {
					cout << "ERROR: Could not open or find the image!" << endl;
					return -1;
				}
				
				// Show our image (with the filename as the window title)
				imshow(filename, image);

				// Wait for a keystroke to close the window
				waitKey(-1);

				// Cleanup this window
				destroyWindow(filename);
				// If we wanted to get rid of ALL windows: destroyAllWindows();			
			}
		}
		else {
			cout << "SOMETHING" << endl;
		}
	}

	return 0;
}


