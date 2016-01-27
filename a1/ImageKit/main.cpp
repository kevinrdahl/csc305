#include "Image.h"


int main(int, char**){

    int width = 512;
    int height = 512;
    int checkerSize = 32;

    Image image(width, height);

    Pixel black(0, 0, 0);
    Pixel white(255, 255, 255);

    bool isBlack;

    for (int i = 0; i < width; ++ i)
    {
        for (int j = 0; j < height; ++ j)
        {
            int col = i / checkerSize;
            int row = j / checkerSize;

            isBlack = (col % 2) ^ (row % 2);

            if (isBlack) {
                image(i, j) = black;
            } else {
                image(i, j) = white;
            }
        }
    }
    
    image.show("GLFW3+Libpng Image Window Demo");
    image.save("/Users/kdahl/Desktop/demo.png");
	
}
