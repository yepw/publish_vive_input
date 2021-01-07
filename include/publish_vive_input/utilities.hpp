#ifndef __UTILITIES_HPP__
#define __UTILITIES_HPP__

#include <iostream>


/**
 * Print a string to the screen.
 * 
 * Params:
 * 		text - text to print
 * 		newlines - number of newlines to print at the end of input string
 * 		flush - whether to flush text. Generally, only needed when
 * 			printing a large body of text with manual newlines
 */
void printText(std::string text="", int newlines=1, bool flush=false)
{
    // TODO: Consider adding param for width of text line
    std::cout << text;

    for (int i = 0; i < newlines; i++) {
        if (i > 1) {
            std::cout << "\n";
        }
        else {
            std::cout << std::endl;
        }
    }

    if (flush) {
        std::cout.flush();
    }

    return;
}


#endif // __UTILITIES_HPP__