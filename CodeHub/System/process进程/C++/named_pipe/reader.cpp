//
// Created by sin on 2019/8/22.
//

#include <fstream>
#include <sys/stat.h>  // mkfifo
#include <iostream>

int main(int argc, char** argv){
    if((mkfifo("fifo_1", 0666)) == -1){
        printf("file exists\n");
    };

    std::fstream file;
    file.open("fifo_1",std::ios::in);
    for(int i=0;i<5;i++){
        std::string data;
        file >> data;
        std::cout << "reader receive message " << data << std::endl;
    }
    file.close();
    return 0;
}