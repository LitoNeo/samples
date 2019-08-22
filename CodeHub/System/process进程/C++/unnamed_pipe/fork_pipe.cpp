//
// Created by sin on 2019/8/22.
//

#include <iostream>
#include <unistd.h>
#include <fstream>
#include <sys/types.h>

int main(int argc, char** argv){
    int fd[2];
    pid_t pid;
    char data[4096];
    int p = pipe(fd);

    if(p == -1){
        perror("Create pipe error");
        exit(-1);
    }

    pid = fork();
    if(pid < 0){
        perror("Create child process error");
        exit(-1);
    }else if(pid == 0){
        // child process, read
        close(fd[1]);
        read(fd[0],data,4096);
        printf("child process received %s\n",data);
    }else{
        // father process, write
        close(fd[0]);
        std::string sig = "father sig";
        write(fd[1], sig.c_str(), sig.size());
        printf("father process send %s\n", sig.c_str());
    }
    return 0;
}