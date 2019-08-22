//
// Created by sin on 2019/8/22.
//

#include <unistd.h>
#include <fstream>
#include <sys/stat.h>  // mkfifo

int main(int argc, char** argv){
    if((mkfifo("fifo_1", 0666)) == -1){
        printf("file exists\n");
    };

    std::fstream file;
    file.open("fifo_1",std::ios::out);
    for(int i=0;i<5;i++){
        file << i << std::endl;
        printf("writer send message %d\n", i);
        sleep(1);
    }
    file.close()
    return 0;
}
