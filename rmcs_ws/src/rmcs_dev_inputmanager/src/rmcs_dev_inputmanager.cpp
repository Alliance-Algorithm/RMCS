
#include <stdio.h>
#include <stdlib.h>

#define TTY_PATH "/dev/tty"
#define STTY_US "stty raw -echo -F "
#define STTY_DEF "stty -raw echo -F "

int get_char();

int get_char() {
  fd_set rfds;
  struct timeval tv;
  int ch = 0;

  FD_ZERO(&rfds);
  FD_SET(0, &rfds);
  tv.tv_sec = 0;
  tv.tv_usec = 10; //设置等待超时时间

  //检测键盘是否有输入
  if (select(1, &rfds, NULL, NULL, &tv) > 0) {
    ch = getchar();
  }
  return ch;
}

int main() {
  int ch = 0;
  system(STTY_US TTY_PATH);

  while (1) {
    ch = get_char();
    if (ch != 0) {
      printf("%d\n\r", ch);
    }
    if (ch == 3) {
      system(STTY_DEF TTY_PATH);
      return 0;
    }
  }
}