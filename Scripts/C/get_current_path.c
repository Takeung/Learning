#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <libgen.h>
#include <string.h>

static char current_path[PATH_MAX];

int get_current_path_prefix() {
    char exe_path[PATH_MAX];  // 保存可执行文件的完整路径
    ssize_t count = readlink("/proc/self/exe", exe_path, PATH_MAX);

    if (count == -1) {
        perror("readlink");
        return -1;
    }

    exe_path[count] = '\0';  // 确保路径字符串以空字符结尾

    // 获取可执行文件所在目录的路径
    char *dir_path = dirname(exe_path);
    strcat(current_path, dir_path);
    strcat(current_path, "/");

    return 0;
}

int main() {
    get_current_path_prefix();

    // 打印目录路径
    printf("当前可执行文件所在目录的绝对路径: %s\n", current_path);

    return 0;
}
